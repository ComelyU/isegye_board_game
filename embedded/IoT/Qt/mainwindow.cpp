#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow),
	mqttClient(SERVER_ADDRESS, CLIENT_ID),
	mqttCallback(new MqttCallback(this))
{
	ui->setupUi(this);
	appDir = QCoreApplication::applicationDirPath();
	videoList = { "default","west","space","ocean","jungle","middle","horror","puzzle" };	//비디오리스트
	nowStatus = "default";


	player = new QMediaPlayer(this);
	videoWidget = new QVideoWidget();
	playlist = new QMediaPlaylist(player);
	player->setVideoOutput(videoWidget);
	changeVolume(50);

	changeVideo(appDir + "/../src/default.mp4");

	setWebView();
	mqttConnect();
	connect(this, &MainWindow::requestVideoChange, this, &MainWindow::changeVideo);
	connect(this, &MainWindow::requestWebView, this, &MainWindow::setWebView);


	//showFullScreen();

}

MainWindow::~MainWindow()
{
	// 스레드 종료
	mqttClient.disconnect();

	delete ui;
}

void MainWindow::setWebView() {
  if (centralWidget() != nullptr) {
         delete centralWidget();
     }
  webView = new QWebEngineView(this); // 새로운 객체 생성
  webView->setUrl(QUrl("http://192.168.212.219:8000/index.html"));
  setCentralWidget(webView);
}

void MainWindow::changeVideo(const QString& videoPath) {	//영상 바꾸기
  if (centralWidget() != nullptr) {
         delete centralWidget();
     }
    videoWidget = new QVideoWidget();
        setCentralWidget(videoWidget);
        player->setVideoOutput(videoWidget);
        playlist->clear();
        playlist->addMedia(QUrl::fromLocalFile(videoPath));
        playlist->setCurrentIndex(0);
        playlist->setPlaybackMode(QMediaPlaylist::Loop);

	player->setPlaylist(playlist);
	player->play();
}
void MainWindow::changeVolume(int volume) {
	player->setVolume(volume);
}

void MainWindow::connectionLost(const std::string& cause)
{
	std::cerr << "Connection lost: " << cause << std::endl;
}

void MainWindow::messageArrived(mqtt::const_message_ptr msg)
{
  QString requestData = QString::fromStdString(msg->to_string());

	std::cout << "Message arrived: " << msg->to_string() << std::endl;
	std::cout << "\ttopic: '" << msg->get_topic() << "'" << std::endl;
	if (requestData == nowStatus) {		//이미 같은 테마
		std::cout << "Already Playing" << std::endl;
	      return ;
	}
	else if (requestData == "webview") {
		emit requestWebView();
		std::cout << "Change webview!" << std::endl;
		return ;
	}
	else if (std::find(videoList.begin(), videoList.end(), requestData) != videoList.end()) {	// 요청에 따라 영상 변경
		nowStatus = requestData;
		emit requestVideoChange(appDir + "/../src/" + requestData + ".mp4");
		std::cout << "Change Complete!" << std::endl;
		return ;
	}
	else if (requestData.toInt() >= 0 and requestData.toInt() <= 100) {
		changeVolume(requestData.toInt());
		std::cout << "Change Volume!" << std::endl;
		return ;
	}

	else
		std::cerr << "Invalid request: " << requestData.toStdString() << std::endl;

}

void MainWindow::deliveryComplete(mqtt::delivery_token_ptr token)
{
	std::cout << "Delivery complete" << std::endl;
}

void MainWindow::mqttConnect()
{
	try {
		mqtt::connect_options connOpts;
		connOpts.set_keep_alive_interval(20);
		connOpts.set_clean_session(true);
		connOpts.set_user_name("accio");
		connOpts.set_password("accio706");

		std::cout << "Connecting to MQTT server..." << std::endl;
		mqttClient.connect(connOpts)->wait();
		std::cout << "Connected" << std::endl;

		std::cout << "Subscribing to topic " << TOPIC << "..." << std::endl;
		mqttClient.subscribe(TOPIC, 1)->wait();
		std::cout << "Subscribed" << std::endl;


		mqttClient.set_callback(*mqttCallback);
		std::cout << "Set Callback" << std::endl;

		//mqtt::message_ptr msg = mqtt::make_message(TOPIC, "west");
		//mqttClient.publish(msg)->wait();
		//publish test
	}
	catch (const mqtt::exception& exc) {

		std::cerr << "Error: " << exc.what() << std::endl;

	}
}



//void MainWindow::socketThread() {
//	QTcpSocket socket;
//	while (true) {
//		if (socket.state() != QAbstractSocket::ConnectedState) {
//			socket.connectToHost("k10a706.p.ssafy.io", 1883); // 서버에 연결
//			if (socket.waitForConnected()) {
//				std::cout << "Connected to server." << std::endl;
//				QString m = "pi";
//				socket.write(m.toUtf8());
//				socket.flush();
//			}
//			else {
//				std::cerr << "Failed to connect to server. Retrying ..." << std::endl;
//				QThread::msleep(1000);
//				continue;
//			}
//		}

//		if (socket.waitForReadyRead()) {
//			QString requestData = socket.readAll();
//			std::cout << "Received request: " << requestData.toStdString() << std::endl;

//			if (requestData == nowStatus) {		//이미 같은 테마
//				std::cout << "Already Playing" << std::endl;
//				continue;
//			}
//			else if (std::find(videoList.begin(), videoList.end(), requestData) != videoList.end()) {	// 요청에 따라 영상 변경
//				nowStatus = requestData;
//				emit requestVideoChange(appDir + "/../src/" + requestData + ".mp4");
//				std::cout << "Change Complete!" << std::endl;
//			}
//			else if(requestData.toInt()>=0 and requestData.toInt()<=100){
//			    changeVolume(requestData.toInt());
//			  }
//			else if(requestData=="WebView"){
//			    setWebView();
//			  }
//			else
//				std::cerr << "Invalid request: " << requestData.toStdString() << std::endl;
//		}

//		if (socket.state() == QAbstractSocket::UnconnectedState) {
//			std::cerr << "Connection to server lost. Retrying second..." << std::endl;
//			QThread::msleep(1000);
//		}
//	}
//}
