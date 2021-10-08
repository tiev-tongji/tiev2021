#include <QApplication>
#include <QtOpenGL>

#include "src/qt/twindow.h"

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    Q_INIT_RESOURCE(dockwidgets);
    TWindow tievWin;
    tievWin.show();
    return app.exec();
}