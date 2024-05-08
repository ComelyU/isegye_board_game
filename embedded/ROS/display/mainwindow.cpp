#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent)
	: QMainWindow(parent)
	, ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	emoji_list= {"sleep.gif","wake.gif","heart.gif","no.gif"};



	connect(this, &MainWindow::emojiReceivedSignal, this, &MainWindow::setEmoji);
	connect(this, &MainWindow::buttonReceivedSignal, this, &MainWindow::setButton);
	setButton();
	//setEmoji("sleep");
	runRos();

	resize(1024,600);
	//showFullScreen();
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
		"emoji", 10, std::bind(&MainWindow::msgReceived, this, std::placeholders::_1));

	publisher = node->create_publisher<std_msgs::msg::String>("topic", 10);
	// ROS 스레드 시작
	std::thread spin_thread([this]() {
		rclcpp::spin(node);
		});
	spin_thread.detach(); // 스레드 분리

}
void MainWindow::msgReceived(const std_msgs::msg::String::SharedPtr msg)
{
	std::cout << msg->data<<": subs complete\n";
	if((msg->data)=="button")
	  emit buttonReceivedSignal();
	else
	  emit emojiReceivedSignal(QString::fromStdString(msg->data));

}

void MainWindow::msgPub() {
	auto msg = std::make_shared<std_msgs::msg::String>();
	msg->data ="next";
	publisher->publish(*msg);

	std::cout<<"pub complete\n";
	setEmoji("smile");

}
void MainWindow::setEmoji(QString status) {
  if (centralWidget() != nullptr) {
           delete centralWidget();
       }
  label = new QLabel("wait", this);
  label->setAlignment(Qt::AlignCenter);
	QString app_path = QCoreApplication::applicationDirPath();
	QMovie* movie = new QMovie(app_path + "/../src/"+status+".gif");
	movie->setScaledSize(QSize(1024, 600));
	setCentralWidget(label);
	label->setMovie(movie);
	movie->start();

	//label->setStyleSheet("background-color:grey");
	//label->setFixedSize(QSize(1024,600));
	label->show();
	std::cout<<"set emoji complete\n";
}

void MainWindow::setButton() {
	QPushButton* pubBtn = new QPushButton("준비를 마치고\n 화면을 눌러주세요", this);
	//pubBtn->setStyleSheet("color: #5E412F; background-color: rgba(255,224,140,240); border: 11px solid #FF8220; border-radius: 100px;");
	pubBtn->setStyleSheet("color: #5E412F; background-color: rgba(255,224,140,240); border: 11px solid #5E412F; border-radius: 100px;");
	pubBtn->setFixedSize(800,500);

	QString fontName = "Jua";
	  // 폰트 데이터베이스에서 폰트 경로 확인
	  QString fontPath = QFontDatabase::applicationFontFamilies(QFontDatabase::addApplicationFont("../src/Jua.ttf")).at(0);
	  // 폰트 로드
	  QFont font(fontName);
	  // 폰트 크기 설정
	  font.setPointSize(70);
	 pubBtn->setFont(font);

	QVBoxLayout* lay = new QVBoxLayout();
	lay->addStretch();
	lay->addWidget(pubBtn,0,Qt::AlignCenter);
	lay->addStretch();
	//lay->setContentsMargins(0, 0, 0, 0);

	QWidget* widget = new QWidget(this);
	widget->setLayout(lay);

	setCentralWidget(widget);

	connect(pubBtn, &QPushButton::clicked, this, &MainWindow::msgPub);

	std::cout<<"set button complete\n";
}
