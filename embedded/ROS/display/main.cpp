#include "mainwindow.h"
#include <QApplication>
#include <csignal>
#include <iostream>

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received. Exiting...\n";
    QCoreApplication::exit(0);
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // Ctrl+C 시그널 핸들러 설정
    signal(SIGINT, signalHandler);

    MainWindow w;
    w.show();
    return a.exec();
}
