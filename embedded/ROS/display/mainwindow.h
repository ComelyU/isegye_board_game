#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QHBoxLayout>
#include <QPushButton>
#include <QThread>
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
	QLabel* label;
	rclcpp::Node::SharedPtr node;
	rclcpp::Subscription <std_msgs::msg::String>::SharedPtr subscription;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;

	int cnt;
	void pub();
	void messageReceived(const std_msgs::msg::String::SharedPtr msg);
	void run_ros();
};

#endif // MAINWINDOW_H
