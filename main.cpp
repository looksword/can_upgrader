// main.cpp
#include <QCoreApplication>
#include <QCommandLineParser>
#include <QTextStream>
#include <signal.h>
#include "native_can_upgrader.h"

static volatile bool g_running = true;

void signalHandler(int)
{
    g_running = false;
}

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName("can_upgrader");
    // QCoreApplication::setApplicationVersion("1.0");
    
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    QCommandLineParser parser;
    parser.setApplicationDescription("CAN总线固件升级工具（原生SocketCAN）");
    parser.addHelpOption();
    parser.addVersionOption();
    
    parser.addPositionalArgument("file", "固件文件路径 (.bin)");
    
    QCommandLineOption canInterfaceOption(QStringList() << "i" << "interface",
                                          "CAN接口名称 (如 can0)",
                                          "interface");
    parser.addOption(canInterfaceOption);
    
    QCommandLineOption bitrateOption(QStringList() << "b" << "bitrate",
                                     "CAN总线波特率 (默认: 500000)",
                                     "bitrate");
    parser.addOption(bitrateOption);
    
    QCommandLineOption verboseOption(QStringList() << "s" << "showdetails",
                                     "显示详细信息");
    parser.addOption(verboseOption);
    
    parser.process(app);
    
    const QStringList args = parser.positionalArguments();
    if (args.isEmpty()) {
        parser.showHelp(1);
    }
    
    QString firmwareFile = args.first();
    QString canInterface = parser.value(canInterfaceOption);
    if (canInterface.isEmpty()) {
        canInterface = "can0";
    }
    
    int bitrate = parser.value(bitrateOption).toInt();
    if (bitrate == 0) {
        bitrate = 500000;
    }
    
    bool verbose = parser.isSet(verboseOption);
    
    NativeCANUpgrader upgrader;
    
    QObject::connect(&upgrader, &NativeCANUpgrader::logMessage,
                     [](const QString &msg) {
        QTextStream out(stdout);
        out << msg << endl;
    });
    
    QObject::connect(&upgrader, &NativeCANUpgrader::progressUpdated,
                     [](int current, int total) {
        QTextStream out(stdout);
        int percent = (current * 100) / total;
        out << "\r[" << QString(percent / 2, '#') 
            << QString(50 - percent / 2, ' ') << "] "
            << percent << "% (" << current << "/" << total << ")" 
            << flush;
    });
    
    QObject::connect(&upgrader, &NativeCANUpgrader::upgradeFinished,
                     [&app](bool success) {
        QTextStream out(stdout);
        out << endl;
        if (success) {
            out << "success." << endl;
        } else {
            out << "failed." << endl;
        }
        g_running = false;
        app.quit();
    });
    
    upgrader.setVerbose(verbose);
    
    if (!upgrader.initCAN(canInterface, bitrate)) {
        QTextStream(stderr) << "init can failed" << endl;
        return 1;
    }
    
    if (!upgrader.loadFirmware(firmwareFile)) {
        QTextStream(stderr) << "load bin file failed" << endl;
        return 1;
    }
    
    if (!upgrader.startUpgrade()) {
        QTextStream(stderr) << "failed to go upgrade" << endl;
        return 1;
    }
    
    while (g_running) {
        app.processEvents();
        QThread::msleep(10);
    }
    
    return 0;
}