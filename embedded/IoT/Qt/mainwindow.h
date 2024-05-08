#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <QMainWindow>
#include <thread>
#include <QMediaPlayer>
#include <QVideoWidget>
#include <QHBoxLayout>
#include <QMediaPlaylist>
#include <QTcpSocket>
#include <QHostAddress>
#include <vector>
#include <QThread>
#include <QWebEngineView>
#include <mqtt/async_client.h>
#include <QPointer> // 추가
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

const std::string TOPIC = "ros_test";
const std::string SERVER_ADDRESS = "k10a706.p.ssafy.io:1883";
const std::string CLIENT_ID = "IoT_Display";

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();
    Ui::MainWindow* ui;
    QString videoFilePath;
    QMediaPlayer* player;
    QVideoWidget* videoWidget;
    QMediaPlaylist* playlist;
    std::thread thread_socket;
    std::vector<QString> videoList;
    QString nowStatus;
    QString appDir;
    QWebEngineView* webView;
    mqtt::async_client mqttClient;

    void connectionLost(const std::string& cause);
    void messageArrived(mqtt::const_message_ptr msg);
    void deliveryComplete(mqtt::delivery_token_ptr token);


private:

    class MqttCallback : public mqtt::callback {
    public:
        explicit MqttCallback(MainWindow* mainWindow) : mainWindow(mainWindow) {}

        void connection_lost(const std::string& cause) override {
            if (mainWindow)
                mainWindow->connectionLost(cause);
        }

        void message_arrived(mqtt::const_message_ptr msg) override {
            if (mainWindow)
                mainWindow->messageArrived(msg);
        }

        void delivery_complete(mqtt::delivery_token_ptr token) override {
            if (mainWindow)
                mainWindow->deliveryComplete(token);
        }

    private:
        MainWindow* mainWindow;
    };

MqttCallback* mqttCallback;
    void changeVideo(const QString& videoPath);
    void socketThread();
    void setWebView();
    void changeVolume(int volume);
    void mqttConnect();
    //void mqttThread();



signals:
    void requestVideoChange(const QString& videoPath);
    void requestVolumeChange(int volume);
    void requestWebView();

};
#endif // MAINWINDOW_H
