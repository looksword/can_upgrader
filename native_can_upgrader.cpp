// native_can_upgrader.cpp
#include "native_can_upgrader.h"

NativeCANUpgrader::NativeCANUpgrader(QObject *parent) : QObject(parent)
{
    m_readTimer = new QTimer(this);
    m_readTimer->setInterval(1);
    connect(m_readTimer, &QTimer::timeout, this, &NativeCANUpgrader::onReadData);
    
    m_timeoutTimer = new QTimer(this);
    m_timeoutTimer->setSingleShot(true);
    connect(m_timeoutTimer, &QTimer::timeout, this, &NativeCANUpgrader::onTimeout);
}

NativeCANUpgrader::~NativeCANUpgrader()
{
    if (m_canSocket >= 0) {
        close(m_canSocket);
    }
}

bool NativeCANUpgrader::configureCANInterface(const QString &interfaceName, int bitrate)
{
    struct ifreq ifr;
    struct sockaddr_can addr;
    
    // 创建socket
    m_canSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (m_canSocket < 0) {
        log("创建CAN socket失败: " + QString(strerror(errno)));
        return false;
    }
    
    // 设置接口
    strcpy(ifr.ifr_name, interfaceName.toStdString().c_str());
    if (ioctl(m_canSocket, SIOCGIFINDEX, &ifr) < 0) {
        log("获取接口索引失败: " + QString(strerror(errno)));
        close(m_canSocket);
        m_canSocket = -1;
        return false;
    }
    
    // 绑定socket
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(m_canSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        log("绑定CAN socket失败: " + QString(strerror(errno)));
        close(m_canSocket);
        m_canSocket = -1;
        return false;
    }
    
    // 设置非阻塞模式
    int flags = fcntl(m_canSocket, F_GETFL, 0);
    fcntl(m_canSocket, F_SETFL, flags | O_NONBLOCK);
    
    log(QString("CAN接口初始化成功: %1").arg(interfaceName));
    return true;
}

bool NativeCANUpgrader::initCAN(const QString &interfaceName, int bitrate)
{
    return configureCANInterface(interfaceName, bitrate);
}

bool NativeCANUpgrader::loadFirmware(const QString &filePath)
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
    
    uint32_t maxSize = 0x08128000 - 0x08004000;
    if (m_ctx.firmwareData.size() > maxSize) {
        log(QString("固件文件过大: %1 字节，最大支持 %2 字节")
            .arg(m_ctx.firmwareData.size())
            .arg(maxSize));
        return false;
    }

    log(QString("固件加载成功，大小: %1 字节").arg(m_ctx.firmwareData.size()));
    return true;
}

bool NativeCANUpgrader::startUpgrade()
{
    if (m_canSocket < 0) {
        log("CAN设备未初始化");
        return false;
    }

    if (m_ctx.firmwareData.empty()) {
        log("未加载固件文件");
        return false;
    }

    m_ctx.currentOffset = 0;
    m_ctx.retryCount = 0;
    
    // 启动读取定时器
    m_readTimer->start();
    
    // 发送擦除命令
    sendEraseCommand();
    return true;
}

bool NativeCANUpgrader::sendCanFrame(uint32_t id, const uint8_t *data, int len)
{
    if (len > 8) len = 8;
    
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = len;
    memset(frame.data, 0, 8);
    memcpy(frame.data, data, len);
    
    int n = write(m_canSocket, &frame, sizeof(frame));
    if (n != sizeof(frame)) {
        log("发送CAN帧失败: " + QString(strerror(errno)));
        return false;
    }
    
    if (m_verbose) {
        QString hexData;
        for (int i = 0; i < len; i++) {
            hexData += QString("%1 ").arg(data[i], 2, 16, QChar('0'));
        }
        log(QString("发送: ID=0x%1, Data=[%2]").arg(id, 3, 16, QChar('0')).arg(hexData));
    }
    
    return true;
}

bool NativeCANUpgrader::readCanFrame(struct can_frame &frame)
{
    int n = read(m_canSocket, &frame, sizeof(frame));
    if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            log("读取CAN帧失败: " + QString(strerror(errno)));
        }
        return false;
    }
    return n == sizeof(frame);
}

void NativeCANUpgrader::sendEraseCommand()
{
    uint8_t data[8] = {CMD_ERASE_APP, 0, 0, 0, 0, 0, 0, 0};
    if (sendCanFrame(CAN_ID_COMMAND, data, 8)) {
        log("发送擦除APP区域命令...");
        m_timeoutTimer->start(TIMEOUT_MS);
    } else {
        log("发送擦除命令失败");
        emit upgradeFinished(false);
    }
}

void NativeCANUpgrader::sendDataFrame(const uint8_t *data, int len)
{
    if (len > MAX_DATA_PER_FRAME) len = MAX_DATA_PER_FRAME;
    
    uint8_t payload[8] = {0};
    for (int i = 0; i < len; i++) {
        payload[i] = data[i];
    }
    
    if (sendCanFrame(CAN_ID_DATA, payload, len)) {
        m_timeoutTimer->start(TIMEOUT_MS);
    } else {
        m_ctx.retryCount++;
        if (m_ctx.retryCount >= MAX_RETRY_COUNT) {
            log("重试次数超限，升级失败");
            emit upgradeFinished(false);
        } else {
            m_timeoutTimer->start(TIMEOUT_MS);
        }
    }
}

void NativeCANUpgrader::sendJumpCommand()
{
    uint8_t data[8] = {CMD_JUMP_TO_APP, 0, 0, 0, 0, 0, 0, 0};
    if (sendCanFrame(CAN_ID_COMMAND, data, 8)) {
        log("发送跳转到APP命令...");
        m_timeoutTimer->start(TIMEOUT_MS);
    } else {
        log("发送跳转命令失败");
        emit upgradeFinished(false);
    }
}

void NativeCANUpgrader::onReadData()
{
    struct can_frame frame;
    while (readCanFrame(frame)) {
        processResponse(frame);
    }
}

void NativeCANUpgrader::processResponse(const struct can_frame &frame)
{
    uint32_t id = frame.can_id;
    uint8_t len = frame.can_dlc;
    const uint8_t *data = frame.data;
    
    if (len < 1) return;
    
    if (m_verbose) {
        QString hexData;
        for (int i = 0; i < len; i++) {
            hexData += QString("%1 ").arg(data[i], 2, 16, QChar('0'));
        }
        log(QString("收到: ID=0x%1, Data=[%2]").arg(id, 3, 16, QChar('0')).arg(hexData));
    }
    
    m_timeoutTimer->stop();
    m_ctx.retryCount = 0;
    
    if (id == CAN_ID_COMMAND) {
        uint8_t cmd = data[0];
        
        if (cmd == CMD_ERASE_APP) {
            log("APP区域擦除成功");
            if (!m_ctx.firmwareData.empty()) {
                int sendLen = std::min(MAX_DATA_PER_FRAME, 
                                      static_cast<int>(m_ctx.firmwareData.size() - m_ctx.currentOffset));
                sendDataFrame(&m_ctx.firmwareData[m_ctx.currentOffset], sendLen);
                m_ctx.currentOffset += sendLen;
            }
        } else if (cmd == CMD_JUMP_TO_APP) {
            log("跳转到APP成功，升级完成");
            m_readTimer->stop();
            emit upgradeFinished(true);
        } else {
            // // 错误码
            // uint8_t error = data[0];
            // switch (error) {
            // case ERR_FLASH_ERROR:
            //     log("Flash错误");
            //     break;
            // case ERR_WRP_ERROR:
            //     log("写保护错误");
            //     break;
            // default:
            //     log(QString("未知错误: 0x%1").arg(error, 2, 16, QChar('0')));
            //     break;
            // }
            // emit upgradeFinished(false);
            log(QString("ID:500 收到命令响应: 0x%1").arg(cmd, 2, 16, QChar('0')));
        }
    }
    else if (id == CAN_ID_DATA) {
        // if (data[0] == ERR_NONE) {
        //     if (m_ctx.currentOffset < static_cast<int>(m_ctx.firmwareData.size())) {
        //         int sendLen = std::min(MAX_DATA_PER_FRAME,
        //                               static_cast<int>(m_ctx.firmwareData.size() - m_ctx.currentOffset));
        //         sendDataFrame(&m_ctx.firmwareData[m_ctx.currentOffset], sendLen);
        //         m_ctx.currentOffset += sendLen;
        //         updateProgress();
        //     } else {
        //         log("所有数据发送完成，准备跳转...");
        //         sendJumpCommand();
        //     }
        // } else {
        //     log(QString("写入错误: 0x%1").arg(data[0], 2, 16, QChar('0')));
        //     emit upgradeFinished(false);
        // }
        if (m_verbose && m_ctx.currentOffset >= MAX_DATA_PER_FRAME) {
            int lastOffset = m_ctx.currentOffset - MAX_DATA_PER_FRAME;
            if (lastOffset >= 0 && lastOffset < static_cast<int>(m_ctx.firmwareData.size())) {
                bool match = true;
                for (int i = 0; i < len && i < MAX_DATA_PER_FRAME; i++) {
                    if (i < 8 && data[i] != m_ctx.firmwareData[lastOffset + i]) {
                        match = false;
                        break;
                    }
                }
                if (!match) {
                    log("警告：返回数据与发送数据不匹配");
                }
            }
        }
        
        // 继续发送下一帧
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
    }
}

void NativeCANUpgrader::onTimeout()
{
    m_ctx.retryCount++;
    
    if (m_ctx.retryCount >= MAX_RETRY_COUNT) {
        log("超时重试次数超限，升级失败");
        emit upgradeFinished(false);
    } else {
        log(QString("等待响应超时，重试 %1/%2...")
            .arg(m_ctx.retryCount)
            .arg(MAX_RETRY_COUNT));
        
        if (m_ctx.currentOffset == 0) {
            sendEraseCommand();
        } else if (m_ctx.currentOffset < static_cast<int>(m_ctx.firmwareData.size())) {
            int retryOffset = m_ctx.currentOffset - MAX_DATA_PER_FRAME;
            if (retryOffset < 0) retryOffset = 0;
            int sendLen = std::min(MAX_DATA_PER_FRAME,
                                  static_cast<int>(m_ctx.firmwareData.size() - retryOffset));
            sendDataFrame(&m_ctx.firmwareData[retryOffset], sendLen);
            m_ctx.currentOffset = retryOffset + sendLen;
        } else {
            sendJumpCommand();
        }
    }
}

void NativeCANUpgrader::updateProgress()
{
    if (m_ctx.currentOffset <= static_cast<int>(m_ctx.firmwareData.size())) {
        emit progressUpdated(m_ctx.currentOffset, m_ctx.firmwareData.size());
    }
}

void NativeCANUpgrader::log(const QString &msg)
{
    if (m_verbose) {
        emit logMessage(msg);
    }
}