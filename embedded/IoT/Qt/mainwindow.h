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

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

const std::string TOPIC = "your/topic";
const std::string SERVER_ADDRESS = "mqtt://localhost:1883";
const std::string CLIENT_ID = "paho_cpp_async_subcribe";

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget* parent = nullptr);
	~MainWindow();

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
	//std::thread mqttThread;


private:
	Ui::MainWindow* ui;

	void changeVideo(const QString& videoPath);
	void socketThread();
	void setWebView();
	void changeVolume(int volume);
	void mqttConnect();
	void mqttThread();
	mqtt::callback mqttCallback(const std::string& topic, const mqtt::message_ptr& msg);

signals:
	void requestVideoChange(const QString& videoPath);
	void requestVolumeChange(int volume);
	void requestWebView();

};
#endif // MAINWINDOW_H
