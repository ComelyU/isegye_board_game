#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow),
	mqttClient(SERVER_ADDRESS,CLIENT_ID)
{
	ui->setupUi(this);

	player = new QMediaPlayer(this);
	videoWidget = new QVideoWidget();
	playlist = new QMediaPlaylist(player);
	setCentralWidget(videoWidget);
	player->setVideoOutput(videoWidget);
	changeVolume(50);
	appDir = QCoreApplication::applicationDirPath();
	videoList = { "default","west","space","ocean","jungle","middle","horror","puzzle" };	//비디오리스트
	nowStatus = "default";
	changeVideo(appDir + "/../src/default.mp4");

	setWebView();
	mqttConnect();
	connect(this, &MainWindow::requestVideoChange, this, &MainWindow::changeVideo);		// 스레드에서 소켓 통신 수행
//	//thread_socket = std::thread(&MainWindow::socketThread, this);
//	thread_socket= std::thread(&MainWindow::mqttThread,this);


	showFullScreen();

}

MainWindow::~MainWindow()
{
	// 스레드 종료
	//mqttClient.disconnect();
	thread_socket.join();
	delete ui;
}
void connection_lost(const std::string& cause) {
    std::cout << "Connection lost" << std::endl;
    if (!cause.empty())
        std::cout << "\tcause: " << cause << std::endl;
}

// MQTT 메시지 수신 콜백 함수
void message_arrived(mqtt::const_message_ptr msg) {
    std::cout << "Message arrived" << std::endl;
    std::cout << "\ttopic: '" << msg->get_topic() << "', "
              << "payload: '" << msg->to_string() << "'" << std::endl;
}

// MQTT 연결 완료 콜백 함수
void delivery_complete(mqtt::delivery_token_ptr token) {}

void MainWindow::setWebView(){
  setCentralWidget(webView);
  webView=new QWebEngineView(this);
  webView->setUrl(QUrl("http://192.168.201.219:8000/index.html"));
}

void MainWindow::changeVideo(const QString& videoPath) {	//영상 바꾸기
	setCentralWidget(videoWidget);
	playlist->clear();
	playlist->addMedia(QUrl::fromLocalFile(videoPath));
	playlist->setCurrentIndex(0);
	playlist->setPlaybackMode(QMediaPlaylist::Loop);

	player->setPlaylist(playlist);
	player->play();
}
void MainWindow::changeVolume(int volume){
	player->setVolume(volume);
}

//void MainWindow::mqttCallback(const std::string& topic, const mqtt::message_ptr& msg) {
//    // 수신된 메시지 처리
//    QString requestData(msg);

////    if (requestData == nowStatus) {
////        std::cout << "Already Playing" << std::endl;
////        return;
////    }
////    else if (std::find(videoList.begin(), videoList.end(), requestData) != videoList.end()) {
////        nowStatus = requestData;
////        emit requestVideoChange(appDir + "/../src/" + requestData + ".mp4");
////        std::cout << "Change Complete!" << std::endl;
////    }
////    else if (requestData.toInt() >= 0 && requestData.toInt() <= 100) {
////        changeVolume(requestData.toInt());
////    }
////    else if (requestData == "WebView") {
////        setWebView();
////    }
////    else {
////        std::cerr << "Invalid request: " << requestData.toStdString() << std::endl;
////    }
//}

void MainWindow::mqttConnect() {
    try {
        // MQTT 브로커에 연결
        mqtt::connect_options connOpts;
        connOpts.set_keep_alive_interval(20); // 옵션 설정 가능
        connOpts.set_clean_session(true);
        connOpts.set_user_name("accio");
        connOpts.set_password("accio706");
        std::cout << "Connecting to MQTT server..." << std::endl;
        mqttClient.connect(connOpts)->wait();
        std::cout << "Connected" << std::endl;

        // 연결이 완료되면 해당 토픽을 구독
        std::cout << "Subscribing to topic " << TOPIC << "..." << std::endl;
        mqttClient.subscribe(TOPIC, 1)->wait();
        std::cout << "Subscribed" << std::endl;

        // MQTT 콜백 함수를 설정
        mqtt::callback CB(connection_lost,message_arrived,delivery_complete);
        mqttClient.set_callback(CB);
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
