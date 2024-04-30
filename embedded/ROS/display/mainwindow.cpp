#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent)
	: QMainWindow(parent)
	, ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	emoji_list= {"sleep.gif","wake.gif","heart.gif","no.gif"};
	label = new QLabel("wait", this);
	label->setAlignment(Qt::AlignCenter);
	setCentralWidget(label);

	connect(this, &MainWindow::emojiReceivedSignal, this, &MainWindow::setEmoji);

	setEmoji("sleep");
	runRos();

}

MainWindow::~MainWindow()
{
	node.reset();
	rclcpp::shutdown();

	delete ui;
}

void MainWindow::runRos() {
	rclcpp::init(0, nullptr);
	node = rclcpp::Node::make_shared("my_package");

	emoji_subscription = node->create_subscription<std_msgs::msg::String>(
		"emoji", 10, std::bind(&MainWindow::emojiReceived, this, std::placeholders::_1));

	//location_subscription = node->create_subscription<std_msgs::msg::String>("tutle_location", 10, std::bind(&MainWindow::locationReceived, this, std::placeholders::_1));

	publisher = node->create_publisher<std_msgs::msg::String>("turtle_location", 10);
	// ROS 스레드 시작
	std::thread spin_thread([this]() {
		rclcpp::spin(node);
		});
	spin_thread.detach(); // 스레드 분리

}
void MainWindow::emojiReceived(const std_msgs::msg::String::SharedPtr msg)
{
	std::cout << "emoji subs complete\n";

	emit emojiReceivedSignal(QString::fromStdString(msg->data));



}
void MainWindow::locationReceived(const std_msgs::msg::String::SharedPtr msg)
{
	std::cout << "loaction subs complete\n";
	//label->setText(QString::fromStdString(msg->data));
	setButton();
}
void MainWindow::pub() {
	auto msg = std::make_shared<std_msgs::msg::String>();
	msg->data ="next";
	publisher->publish(*msg);

	//RCLCPP_INFO(rclcpp::get_logger("this"), "Publish Button: %d", cnt);

	/*
	   delete button code
	*/
	setEmoji("heart2");

}
void MainWindow::setEmoji(QString status) {
	QString app_path = QCoreApplication::applicationDirPath();
	QMovie* movie = new QMovie(app_path + "/../src/"+status+".gif");
	movie->setScaledSize(QSize(1024, 600));

	label->setMovie(movie);
	movie->start();

	//label->setStyleSheet("background-color:grey");
	//label->setFixedSize(QSize(1024,600));
	label->show();
	std::cout<<"set emoji complete\n";
}

void MainWindow::setButton() {
	QPushButton* pubBtn = new QPushButton("Publish", this);
	pubBtn->setGeometry(20, 40, 100, 30);
	connect(pubBtn, &QPushButton::clicked, this, &MainWindow::pub);

}
