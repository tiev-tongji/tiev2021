#include <QGuiApplication>
#include <QOpenGLContext>
#include <QSurfaceFormat>
#include <thread>

#include "message_manager.h"
#include "tviswindow.h"

int main(int argc, char* argv[]) {
    QGuiApplication app(argc, argv);

    QSurfaceFormat fmt;
    fmt.setDepthBufferSize(24);

    // Request OpenGL 3.3 core or OpenGL ES 3.0.
    if(QOpenGLContext::openGLModuleType() == QOpenGLContext::LibGL) {
        qDebug("Requesting 3.3 core context");
        fmt.setVersion(3, 3);
        fmt.setProfile(QSurfaceFormat::CoreProfile);
    }
    else {
        qDebug("Requesting 3.0 context");
        fmt.setVersion(3, 0);
    }

    QSurfaceFormat::setDefaultFormat(fmt);
    // load semantic map
    const QString map_file = "/home/autolab/carla_path/Town03_original.txt";
    SemanticMap   semantic_map(map_file);
    // initilize the camera and coordinate converter
    Camera camera(QVector3D(0, 0, 1));
    camera.setPosition(semantic_map.getOrigin() + QVector3D(0, 0, 500));
    // camera.setPosition(QVector3D(-8, 0, 10));
    // camera.setYaw(0);
    // camera.setPitch(-M_PI_4);
    CoodinateConverter coordinate_converter(semantic_map.getOrigin());
    TVisWindow         tvis_window(&semantic_map, camera, coordinate_converter);
    tvis_window.setMinimumSize(QSize(800, 600));
    // tvis_window.showFullScreen();
    tvis_window.show();

    // message manager
    MessageManager& msg_manager = MessageManager::getInstance();
    std::thread     msg_receiver_ipc =
        std::thread(&MessageManager::msgReceiveIpc, &msg_manager);
    std::thread msg_receiver_udp =
        std::thread(&MessageManager::msgReceiveUdp, &msg_manager);

    return app.exec();
}