QT += widgets opengl
requires(qtConfig(listwidget))
qtHaveModule(printsupport): QT += printsupport

TARGET = tvis
TEMPLATE = app

HEADERS         = src/qt/twindow.h
SOURCES         = main.cpp \
                  src/qt/twindow.cpp
RESOURCES       = src/qt/dockwidgets.qrc