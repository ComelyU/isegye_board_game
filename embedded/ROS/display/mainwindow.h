#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QHBoxLayout>
#include <QPushButton>
#include <QThread>
#include <QString>
#include <QMovie>
#include <QDebug>
#include <QVector>
#include <QFont>
#include <QFontDatabase>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget* parent = nullptr);
	~MainWindow();

private:
	Ui::MainWindow* ui;

	QVector<QString>emoji_list;
	QLabel* label;
	rclcpp::Node::SharedPtr node;
	rclcpp::Subscription <std_msgs::msg::String>::SharedPtr subscription;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;

	void setEmoji(QString status);
	void setButton();
	void msgPub();
	void msgReceived(const std_msgs::msg::String::SharedPtr msg);
	void runRos();
signals:
	void emojiReceivedSignal(const QString &data);
	void buttonReceivedSignal();

};

#endif // MAINWINDOW_H
