// can_upgrader.h
#ifndef CAN_UPGRADER_H
#define CAN_UPGRADER_H

#include <QObject>
#include <QCanBus>
#include <QCanBusDevice>
#include <QFile>
#include <QTimer>
#include <QDebug>
#include <QThread>
#include <algorithm>
#include <vector>
#include <cstdint>

class CANUpgrader : public QObject
{
    Q_OBJECT

public:
    explicit CANUpgrader(QObject *parent = nullptr);
    ~CANUpgrader();

    bool initCAN(const QString &interfaceName, int bitrate = 500000);
    bool loadFirmware(const QString &filePath);
    bool startUpgrade();
    void setVerbose(bool verbose) { m_verbose = verbose; }

signals:
    void progressUpdated(int current, int total);
    void logMessage(const QString &msg);
    void upgradeFinished(bool success);

private slots:
    void onFramesReceived();
    void onErrorOccurred(QCanBusDevice::CanBusError error);
    void onTimeout();

private:
    // CAN ID定义（与下位机对应）
    static const uint32_t CAN_ID_COMMAND;      // 命令通道（擦除/复位/跳转）
    static const uint32_t CAN_ID_DATA;         // 数据通道（固件写入）
    static const uint32_t CAN_ID_TEST;         // 测试通道
    
    // 命令定义（与下位机对应）
    enum Command : uint8_t {
        CMD_ERASE_APP = 0x00,           // 擦除APP区域
        CMD_JUMP_TO_APP = 0x01,         // 跳转到APP
        CMD_DATA_WRITE = 0x02,          // 数据写入（实际由ID区分）
    };
    
    // 错误码定义
    enum ErrorCode : uint8_t {
        ERR_NONE = 0x00,
        ERR_ADDRESS_OVERFLOW = 0xAC,    // 地址溢出
        ERR_WRITE_ERROR = 0xEE,         // 写入错误
        ERR_VERIFY_FAILED = 0xED,       // 校验失败
        ERR_FLASH_ERROR = 0x08,         // Flash错误
        ERR_WRP_ERROR = 0x09,           // 写保护错误
    };
    
    struct UpgradeContext {
        std::vector<uint8_t> firmwareData;
        uint32_t writeAddress = 0x08004000;  // APP起始地址
        uint32_t currentOffset = 0;           // 当前偏移量
        int expectedSeq = 0;                  // 期望的序号
        bool waitingAck = false;              // 等待确认
        uint8_t retryCount = 0;               // 重试次数
    };
    
    static const int MAX_DATA_PER_FRAME;   // CAN帧最大8字节
    static const int MAX_RETRY_COUNT;       // 最大重试次数
    static const int TIMEOUT_MS;            // 超时时间
    
    UpgradeContext m_ctx;
    QCanBusDevice *m_canDevice = nullptr;
    QTimer *m_timeoutTimer = nullptr;
    bool m_verbose = false;
    
    // 解码表（下位机使用decode_canBasic异或解码）
    static const uint8_t DECODE_XOR_TABLE[8];
    
    void sendEraseCommand();
    void sendDataFrame(const uint8_t *data, int len);
    void sendJumpCommand();
    void processResponse(const QCanBusFrame &frame);
    void encodeData(uint8_t *output, const uint8_t *input, int len);
    void log(const QString &msg);
    void updateProgress();
    bool verifyWriteAddress(uint32_t address);
};

#endif // CAN_UPGRADER_H