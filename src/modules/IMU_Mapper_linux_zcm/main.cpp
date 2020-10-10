#include "mainwindow.h"
#include <QApplication>

#include "zcm/zcm-cpp.hpp"

#include <pthread.h>
#include <stdio.h>
#include <thread>
//#include <QTextCodec>//set for UTF-8

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    //QTextCodec::setCodecForTr(QTextCodec::codecForLocale());//set for UTF-8

    MainWindow w;
    w.show();

    return a.exec();
}
