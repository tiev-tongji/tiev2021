#-------------------------------------------------
#
# Project created by QtCreator 2018-01-20T17:06:31
#
#-------------------------------------------------

QT       += core gui
QMAKE_CXXFLAGS += -std=c++11
CONFIG += c++11

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = IMU_Mapper_linux_zcm
TEMPLATE = app

LIBS += /usr/local/lib/libzcm.so

INCLUDEPATH += ./../../ \
                ./../../ \
                ./../../common/ \
                ./../../msg/include/ \
                ./../../common/coordinate_converter/ \
                ./../../common/nature

SOURCES += main.cpp\
        mainwindow.cpp\
        ./../../common/coordinate_converter/coordinate_converter.cpp\
        ./../../common/nature/angle.cpp\
        ./../../common/coordinate_converter/basic_coordinate_converter.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui
