#include "tviswindow.h"
#include <QImage>
#include <QOpenGLBuffer>
#include <QOpenGLContext>
#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QOpenGLVertexArrayObject>
#include <QPauseAnimation>
#include <QPropertyAnimation>
#include <QSequentialAnimationGroup>
#include <QTimer>

#include <iostream>

TVisWindow::TVisWindow(SemanticMap* semantic_map_, const Camera& camera_,
                       const CoodinateConverter& coordinate_converter_)
    : m_program(0), m_vbo(0), m_vao(0) {
    semantic_map         = semantic_map_;
    camera               = camera_;
    coordinate_converter = coordinate_converter_;
    QTimer* timer        = new QTimer(this);
    QObject::connect(timer, &QTimer::timeout, this,
                     [&]() { listeningTiEVMessage(); });
    timer->start(10);
}

TVisWindow::~TVisWindow() {
    makeCurrent();
    delete m_program;
    delete m_vbo;
    delete m_vao;
}

static const char* vertexShaderSource =
    "layout(location = 0) in vec4 vertex;\n"
    "uniform mat4 project;\n"
    "uniform mat4 view;\n"
    "uniform mat4 model;\n"
    "void main() {\n"
    "   gl_Position = project* view* model* vertex;\n"
    "}\n";

static const char* fragmentShaderSource = "out highp vec4 fragColor;\n"
                                          "uniform highp vec4 color;\n"
                                          "void main() {\n"
                                          "   fragColor = color;\n"
                                          "}\n";

QByteArray versionedShaderCode(const char* src) {
    QByteArray versionedSrc;

    if(QOpenGLContext::currentContext()->isOpenGLES())
        versionedSrc.append(QByteArrayLiteral("#version 300 es\n"));
    else
        versionedSrc.append(QByteArrayLiteral("#version 330\n"));

    versionedSrc.append(src);
    return versionedSrc;
}

void TVisWindow::initializeGL() {
    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();

    if(m_program) {
        delete m_program;
        m_program = 0;
    }
    m_program = new QOpenGLShaderProgram;
    m_program->addShaderFromSourceCode(QOpenGLShader::Vertex,
                                       versionedShaderCode(vertexShaderSource));
    m_program->addShaderFromSourceCode(
        QOpenGLShader::Fragment, versionedShaderCode(fragmentShaderSource));
    m_program->link();
    m_program->bind();

    semantic_map->initGL();
    tiev_car.initGL();
    // QObject::connect(&tiev_car, &TiEVCar::poseChanged, [&]() { update(); });
    QObject::connect(&tiev_car, &TiEVCar::poseChanged, [&]() {
        const auto& pose = tiev_car.getPose();
        QVector3D   car_direction(std::cos(pose.z()), std::sin(pose.z()), 0);
        camera.setPosition(QVector3D(pose.x(), pose.y(), 80)
                           - 20 * car_direction);
        update();
    });

    f->glEnable(GL_DEPTH_TEST);
    f->glEnable(GL_CULL_FACE);
}

void TVisWindow::resizeGL(int w, int h) {
    QOpenGLExtraFunctions* f =
        QOpenGLContext::currentContext()->extraFunctions();
    f->glViewport(0, 0, w, h);
}

void TVisWindow::paintGL() {
    // Now use QOpenGLExtraFunctions instead of QOpenGLFunctions as we want to
    // do more than what GL(ES) 2.0 offers.
    QOpenGLExtraFunctions* f =
        QOpenGLContext::currentContext()->extraFunctions();

    f->glClearColor(0, 0, 0, 1);
    f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    QMatrix4x4 m_proj;
    m_proj.perspective(camera.getZoom(), GLfloat(width()) / height(), 0.01f,
                       1200.0f);
    m_program->setUniformValue("project", m_proj);
    m_program->setUniformValue("view", camera.getViewMaxtrix());
    qDebug() << "car:" << tiev_car.getPose();

    // paint tiev elements
    semantic_map->paint(m_program);
    // tiev_car.paint(m_program);
}

void TVisWindow::mouseMoveEvent(QMouseEvent* event) {
    if(mouse.mid_press && camera.lock2d()) {
        const double x_offset = event->x() - mouse.last_x;
        const double y_offset = event->y() - mouse.last_y;
        camera.moveCamera(x_offset, y_offset);
        update();
    }
    if(mouse.left_press) {
        const double x_offset = event->x() - mouse.last_x;
        const double y_offset = event->y() - mouse.last_y;
        camera.rotateCamera(x_offset, y_offset);
        update();
    }
}

void TVisWindow::mousePressEvent(QMouseEvent* event) {
    mouse.last_x = event->x();
    mouse.last_y = event->y();
    if(event->button() == Qt::MidButton) {
        mouse.mid_press = true;
        camera.recordOriginPosition();
    }
    else if(event->button() == Qt::LeftButton) {
        mouse.left_press = true;
    }
    else if(event->button() == Qt::RightButton) {
        mouse.right_press = true;
    }
}

void TVisWindow::mouseReleaseEvent(QMouseEvent* event) {
    if(event->button() == Qt::MidButton) {
        mouse.mid_press = false;
    }
    else if(event->button() == Qt::LeftButton) {
        mouse.left_press = false;
    }
    else if(event->button() == Qt::RightButton) {
        mouse.right_press = false;
    }
}

void TVisWindow::wheelEvent(QWheelEvent* event) {
    const auto offset = event->angleDelta();
    camera.processMouseScroll(offset.y());
    update();
}

void TVisWindow::keyPressEvent(QKeyEvent* event) {
    camera.processKeyPress(event);
    update();
}

void TVisWindow::listeningTiEVMessage() {
    auto&   message_manager = MessageManager::getInstance();
    NavInfo nav_info;
    message_manager.getNavInfo(nav_info);
    if(nav_info.detected) {
        const auto& car_pose = nav_info.car_pose.utm_position;
        tiev_car.setPose(car_pose.utm_x, car_pose.utm_y, car_pose.heading);
    }
}