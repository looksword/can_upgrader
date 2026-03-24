// can_upgrader.cpp
#include "can_upgrader.h"
#include <QDebug>
#include <QThread>
#include <algorithm>

// 静态成员变量定义（只定义一次）
const uint32_t CANUpgrader::CAN_ID_COMMAND = 0x500;
const uint32_t CANUpgrader::CAN_ID_DATA = 0x501;
const uint32_t CANUpgrader::CAN_ID_TEST = 0x502;
const int CANUpgrader::MAX_DATA_PER_FRAME = 8;
const int CANUpgrader::MAX_RETRY_COUNT = 3;
const int CANUpgrader::TIMEOUT_MS = 500;

// 解码表（与下位机decode_canBasic对应）
// 如果bin文件已经encode过，这里应该设置为实际解码表的值
// 例如：decode_canBasic = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22}
const uint8_t CANUpgrader::DECODE_XOR_TABLE[8] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07  // 根据实际下位机设置
};

CANUpgrader::CANUpgrader(QObject *parent) : QObject(parent)
{
    m_timeoutTimer = new QTimer(this);
    m_timeoutTimer->setSingleShot(true);
    connect(m_timeoutTimer, &QTimer::timeout, this, &CANUpgrader::onTimeout);
}

CANUpgrader::~CANUpgrader()
{
    if (m_canDevice && m_canDevice->state() == QCanBusDevice::ConnectedState) {
        m_canDevice->disconnectDevice();
    }
    delete m_canDevice;
}

bool CANUpgrader::initCAN(const QString &interfaceName, int bitrate)
{
    // 创建CAN设备
    m_canDevice = QCanBus::instance()->createDevice("socketcan", interfaceName);
    if (!m_canDevice) {
        log("无法创建CAN设备，请检查socketcan插件");
        return false;
    }

    // 配置CAN接口
    QCanBusDevice::ConfigurationKey key = QCanBusDevice::BitRateKey;
    m_canDevice->setConfigurationParameter(key, bitrate);
    
    // 连接信号槽
    connect(m_canDevice, &QCanBusDevice::framesReceived, 
            this, &CANUpgrader::onFramesReceived);
    connect(m_canDevice, &QCanBusDevice::errorOccurred,
            this, &CANUpgrader::onErrorOccurred);

    // 连接设备
    if (!m_canDevice->connectDevice()) {
        log("无法连接CAN设备: " + m_canDevice->errorString());
        return false;
    }

    log(QString("CAN设备初始化成功: %1 @ %2 bps").arg(interfaceName).arg(bitrate));
    return true;
}

bool CANUpgrader::loadFirmware(const QString &filePath)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        log("无法打开固件文件: " + filePath);
        return false;
    }

    QByteArray data = file.readAll();
    file.close();
    
    m_ctx.firmwareData.clear();
    m_ctx.firmwareData.reserve(data.size());
    for (int i = 0; i < data.size(); i++) {
        m_ctx.firmwareData.push_back(static_cast<uint8_t>(data[i]));
    }

    if (m_ctx.firmwareData.empty()) {
        log("固件文件为空");
        return false;
    }
    
    // 检查固件大小是否在有效范围内（根据下位机限制）
    uint32_t maxSize = 0x08128000 - 0x08004000;  // 最大128KB
    if (m_ctx.firmwareData.size() > maxSize) {
        log(QString("固件文件过大: %1 字节，最大支持 %2 字节")
            .arg(m_ctx.firmwareData.size())
            .arg(maxSize));
        return false;
    }

    log(QString("固件加载成功，大小: %1 字节，起始地址: 0x%2")
        .arg(m_ctx.firmwareData.size())
        .arg(m_ctx.writeAddress, 8, 16, QChar('0')));
    return true;
}

bool CANUpgrader::startUpgrade()
{
    if (!m_canDevice || m_canDevice->state() != QCanBusDevice::ConnectedState) {
        log("CAN设备未连接");
        return false;
    }

    if (m_ctx.firmwareData.empty()) {
        log("未加载固件文件");
        return false;
    }

    // 重置上下文
    m_ctx.currentOffset = 0;
    m_ctx.expectedSeq = 0;
    m_ctx.waitingAck = false;
    m_ctx.retryCount = 0;
    
    // 发送擦除命令
    sendEraseCommand();
    return true;
}

void CANUpgrader::sendEraseCommand()
{
    QByteArray payload(8, 0);
    payload[0] = CMD_ERASE_APP;  // 擦除APP区域
    
    QCanBusFrame frame = QCanBusFrame(CAN_ID_COMMAND, payload);
    frame.setExtendedFrameFormat(false);
    
    if (m_canDevice->writeFrame(frame)) {
        log("发送擦除APP区域命令...");
        m_ctx.waitingAck = true;
        m_timeoutTimer->start(TIMEOUT_MS);
    } else {
        log("发送擦除命令失败");
        emit upgradeFinished(false);
    }
}

void CANUpgrader::sendDataFrame(const uint8_t *data, int len)
{
    if (len > MAX_DATA_PER_FRAME) len = MAX_DATA_PER_FRAME;
    
    QByteArray payload(8, 0);
    
    // bin文件已经encode过了，直接发送原始数据，不需要再编码
    // 下位机会用 decode_canBasic 进行解码
    for (int i = 0; i < len; i++) {
        payload[i] = data[i];
    }
    
    QCanBusFrame frame = QCanBusFrame(CAN_ID_DATA, payload);
    frame.setExtendedFrameFormat(false);
    
    if (!m_canDevice->writeFrame(frame)) {
        log("发送数据帧失败");
        m_ctx.retryCount++;
        if (m_ctx.retryCount >= MAX_RETRY_COUNT) {
            log("重试次数超限，升级失败");
            emit upgradeFinished(false);
        } else {
            m_timeoutTimer->start(TIMEOUT_MS);
        }
    } else {
        if (m_verbose) {
            // 显示原始数据（已经encode过的）
            QString hexData;
            for (int i = 0; i < len; i++) {
                hexData += QString("%1 ").arg(data[i], 2, 16, QChar('0'));
            }
            log(QString("发送数据: 地址=0x%1, 数据=[%2]")
                .arg(m_ctx.writeAddress + m_ctx.currentOffset, 8, 16, QChar('0'))
                .arg(hexData));
        }
        m_ctx.waitingAck = true;
        m_timeoutTimer->start(TIMEOUT_MS);
    }
}

void CANUpgrader::sendJumpCommand()
{
    QByteArray payload(8, 0);
    payload[0] = CMD_JUMP_TO_APP;  // 跳转到APP
    
    QCanBusFrame frame = QCanBusFrame(CAN_ID_COMMAND, payload);
    frame.setExtendedFrameFormat(false);
    
    if (m_canDevice->writeFrame(frame)) {
        log("发送跳转到APP命令...");
        m_ctx.waitingAck = true;
        m_timeoutTimer->start(TIMEOUT_MS);
    } else {
        log("发送跳转命令失败");
        emit upgradeFinished(false);
    }
}

void CANUpgrader::encodeData(uint8_t *output, const uint8_t *input, int len)
{
    // 由于bin文件已经encode过了，这个函数不再使用
    // 保留但为空实现，以备后用
    for (int i = 0; i < len && i < 8; i++) {
        output[i] = input[i];
    }
}

void CANUpgrader::onFramesReceived()
{
    if (!m_canDevice) return;
    
    while (m_canDevice->framesAvailable()) {
        QCanBusFrame frame = m_canDevice->readFrame();
        
        if (!frame.isValid()) continue;
        if (frame.frameType() != QCanBusFrame::DataFrame) continue;
        
        processResponse(frame);
    }
}

void CANUpgrader::processResponse(const QCanBusFrame &frame)
{
    uint32_t id = frame.frameId();
    QByteArray payload = frame.payload();
    
    if (payload.isEmpty()) return;
    
    // 只处理响应帧
    if (id != CAN_ID_COMMAND && id != CAN_ID_DATA && id != CAN_ID_TEST) return;
    
    if (m_verbose) {
        log(QString("收到响应: ID=0x%1, Data=%2")
            .arg(id, 3, 16, QChar('0'))
            .arg(QString(payload.toHex().toUpper())));
    }
    
    // 重置超时和重试计数
    m_timeoutTimer->stop();
    m_ctx.waitingAck = false;
    m_ctx.retryCount = 0;
    
    // 处理不同类型的响应
    if (id == CAN_ID_COMMAND) {
        uint8_t cmd = static_cast<uint8_t>(payload[0]);
        
        switch (cmd) {
        case CMD_ERASE_APP:
            log("APP区域擦除成功");
            // 开始发送固件数据
            if (!m_ctx.firmwareData.empty()) {
                int sendLen = std::min(MAX_DATA_PER_FRAME, 
                                      static_cast<int>(m_ctx.firmwareData.size() - m_ctx.currentOffset));
                sendDataFrame(&m_ctx.firmwareData[m_ctx.currentOffset], sendLen);
                m_ctx.currentOffset += sendLen;
            }
            break;
            
        case CMD_JUMP_TO_APP:
            log("跳转到APP成功，升级完成");
            emit upgradeFinished(true);
            break;
            
        default:
            // 检查错误码
            if (payload.size() >= 1) {
                uint8_t error = static_cast<uint8_t>(payload[0]);
                switch (error) {
                case ERR_FLASH_ERROR:
                    log("Flash错误，请检查Flash配置");
                    emit upgradeFinished(false);
                    break;
                case ERR_WRP_ERROR:
                    log("写保护错误，请解除Flash写保护");
                    emit upgradeFinished(false);
                    break;
                default:
                    log(QString("未知错误: 0x%1").arg(error, 2, 16, QChar('0')));
                    emit upgradeFinished(false);
                    break;
                }
            }
            break;
        }
    }
    else if (id == CAN_ID_DATA) {
        // 数据写入确认
        if (payload.size() >= 1) {
            uint8_t status = static_cast<uint8_t>(payload[0]);
            
            if (status == ERR_NONE) {
                // 写入成功，继续发送下一帧
                if (m_ctx.currentOffset < static_cast<int>(m_ctx.firmwareData.size())) {
                    int sendLen = std::min(MAX_DATA_PER_FRAME,
                                          static_cast<int>(m_ctx.firmwareData.size() - m_ctx.currentOffset));
                    sendDataFrame(&m_ctx.firmwareData[m_ctx.currentOffset], sendLen);
                    m_ctx.currentOffset += sendLen;
                    updateProgress();
                } else {
                    // 所有数据发送完成，发送跳转命令
                    log("所有数据发送完成，准备跳转...");
                    sendJumpCommand();
                }
            } else {
                // 写入失败
                switch (status) {
                case ERR_ADDRESS_OVERFLOW:
                    log("地址溢出错误");
                    break;
                case ERR_WRITE_ERROR:
                    log("Flash写入错误");
                    break;
                case ERR_VERIFY_FAILED:
                    log("数据校验失败");
                    break;
                default:
                    log(QString("未知错误码: 0x%1").arg(status, 2, 16, QChar('0')));
                    break;
                }
                emit upgradeFinished(false);
            }
        }
    }
    else if (id == CAN_ID_TEST) {
        log(QString("测试响应: %1").arg(QString(payload.toHex().toUpper())));
    }
}

void CANUpgrader::onTimeout()
{
    m_ctx.retryCount++;
    
    if (m_ctx.retryCount >= MAX_RETRY_COUNT) {
        log("超时重试次数超限，升级失败");
        emit upgradeFinished(false);
    } else {
        log(QString("等待响应超时，重试 %1/%2...")
            .arg(m_ctx.retryCount)
            .arg(MAX_RETRY_COUNT));
        
        // 重发当前操作
        if (m_ctx.currentOffset == 0) {
            // 重发擦除命令
            sendEraseCommand();
        } else if (m_ctx.currentOffset < static_cast<int>(m_ctx.firmwareData.size())) {
            // 重发当前数据帧
            int retryOffset = m_ctx.currentOffset - MAX_DATA_PER_FRAME;
            if (retryOffset < 0) retryOffset = 0;
            int sendLen = std::min(MAX_DATA_PER_FRAME,
                                  static_cast<int>(m_ctx.firmwareData.size() - retryOffset));
            sendDataFrame(&m_ctx.firmwareData[retryOffset], sendLen);
            m_ctx.currentOffset = retryOffset + sendLen;
        } else {
            // 重发跳转命令
            sendJumpCommand();
        }
    }
}

void CANUpgrader::onErrorOccurred(QCanBusDevice::CanBusError error)
{
    QString errorMsg;
    switch (error) {
    case QCanBusDevice::ReadError:
        errorMsg = "读取错误: " + m_canDevice->errorString();
        break;
    case QCanBusDevice::WriteError:
        errorMsg = "写入错误: " + m_canDevice->errorString();
        break;
    case QCanBusDevice::ConnectionError:
        errorMsg = "连接错误: " + m_canDevice->errorString();
        break;
    default:
        errorMsg = "未知错误: " + m_canDevice->errorString();
        break;
    }
    
    log(errorMsg);
    if (error == QCanBusDevice::ConnectionError) {
        emit upgradeFinished(false);
    }
}

void CANUpgrader::updateProgress()
{
    if (m_ctx.currentOffset <= static_cast<int>(m_ctx.firmwareData.size())) {
        emit progressUpdated(m_ctx.currentOffset, m_ctx.firmwareData.size());
    }
}

void CANUpgrader::log(const QString &msg)
{
    if (m_verbose) {
        emit logMessage(msg);
    }
}

bool CANUpgrader::verifyWriteAddress(uint32_t address)
{
    // 验证地址是否在有效范围内
    uint32_t startAddr = 0x08004000;
    uint32_t endAddr = 0x08128000;
    
    if (address < startAddr || address >= endAddr) {
        log(QString("地址越界: 0x%1").arg(address, 8, 16, QChar('0')));
        return false;
    }
    
    // 检查地址对齐（双字对齐）
    if ((address & 0x7) != 0) {
        log(QString("地址未对齐: 0x%1").arg(address, 8, 16, QChar('0')));
        return false;
    }
    
    return true;
}