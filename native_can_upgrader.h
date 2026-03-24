// native_can_upgrader.h
#ifndef NATIVE_CAN_UPGRADER_H
#define NATIVE_CAN_UPGRADER_H

#include <QObject>
#include <QFile>
#include <QTimer>
#include <QThread>
#include <QTextStream>
#include <vector>
#include <cstdint>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <algorithm>
#include <cstring>
#include <errno.h>
#include <fcntl.h>

class NativeCANUpgrader : public QObject
{
    Q_OBJECT

public:
    explicit NativeCANUpgrader(QObject *parent = nullptr);
    ~NativeCANUpgrader();

    bool initCAN(const QString &interfaceName, int bitrate = 500000);
    bool loadFirmware(const QString &filePath);
    bool startUpgrade();
    void setVerbose(bool verbose) { m_verbose = verbose; }

signals:
    void progressUpdated(int current, int total);
    void logMessage(const QString &msg);
    void upgradeFinished(bool success);

private slots:
    void onReadData();
    void onTimeout();

private:
    // CAN ID定义
    static const uint32_t CAN_ID_COMMAND = 0x500;
    static const uint32_t CAN_ID_DATA = 0x501;
    static const uint32_t CAN_ID_TEST = 0x502;
    
    enum Command : uint8_t {
        CMD_ERASE_APP = 0x00,
        CMD_JUMP_TO_APP = 0x01,
    };
    
    enum ErrorCode : uint8_t {
        ERR_NONE = 0x00,
        ERR_ADDRESS_OVERFLOW = 0xAC,
        ERR_WRITE_ERROR = 0xEE,
        ERR_VERIFY_FAILED = 0xED,
        ERR_FLASH_ERROR = 0x08,
        ERR_WRP_ERROR = 0x09,
    };
    
    struct UpgradeContext {
        std::vector<uint8_t> firmwareData;
        uint32_t writeAddress = 0x08004000;
        uint32_t currentOffset = 0;
        int retryCount = 0;
    };
    
    static const int MAX_DATA_PER_FRAME = 8;
    static const int MAX_RETRY_COUNT = 3;
    static const int TIMEOUT_MS = 500;
    
    UpgradeContext m_ctx;
    int m_canSocket = -1;
    QTimer *m_readTimer = nullptr;
    QTimer *m_timeoutTimer = nullptr;
    bool m_verbose = false;
    
    bool configureCANInterface(const QString &interfaceName, int bitrate);
    bool sendCanFrame(uint32_t id, const uint8_t *data, int len);
    bool readCanFrame(struct can_frame &frame);
    void sendEraseCommand();
    void sendDataFrame(const uint8_t *data, int len);
    void sendJumpCommand();
    void processResponse(const struct can_frame &frame);
    void log(const QString &msg);
    void updateProgress();
};

#endif // NATIVE_CAN_UPGRADER_H