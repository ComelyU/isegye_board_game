#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent)
	: QMainWindow(parent)
	, ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	cnt = 0;

	label = new QLabel("wait", this);
	label->setAlignment(Qt::AlignCenter);
	setCentralWidget(label);
	QPushButton* pubBtn = new QPushButton("Publish", this);
	pubBtn->setGeometry(20, 40, 100, 30);
	connect(pubBtn, &QPushButton::clicked, this, &MainWindow::pub);

	run_ros();

}

MainWindow::~MainWindow()
{
	node.reset();
	rclcpp::shutdown();
	delete ui;
}

void MainWindow::run_ros(){
  rclcpp::init(0, nullptr);
  node = rclcpp::Node::make_shared("my_package");

  subscription = node->create_subscription<std_msgs::msg::String>(
          "topic", 10, std::bind(&MainWindow::messageReceived, this, std::placeholders::_1));

  publisher = node->create_publisher<std_msgs::msg::String>("topic", 10);
  // ROS 스레드 시작
  std::thread spin_thread([this]() {
      rclcpp::spin(node);
  });
  spin_thread.detach(); // 스레드 분리

}
void MainWindow::messageReceived(const std_msgs::msg::String::SharedPtr msg)
{
	std::cout << "subs complete\n";
	label->setText(QString::fromStdString(msg->data));

}

void MainWindow::pub() {
	auto msg = std::make_shared<std_msgs::msg::String>();
	msg->data = "Message " + std::to_string(cnt);
	publisher->publish(*msg);

	RCLCPP_INFO(rclcpp::get_logger("this"), "Publish Button: %d", cnt);

	cnt++;
}


