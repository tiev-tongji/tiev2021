#include "tiev_elements.h"
#include <QDebug>
#include <QFile>
#include <QString>
#include <vector>

SemanticMap::SemanticMap(const QString& file_name) {
    // load semantic map
    QFile file(file_name);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "semantic map file: " << file_name << " not exist!";
    }
    auto header = file.readLine().split(' ');
    // check number of fields
    if(header.size() != 14) {
        qDebug("the format maybe wrong");
    }
    // get the original point
    auto        first_line = file.readLine().split(' ');
    const float x_origin   = first_line[3].toDouble();
    const float y_origin   = first_line[4].toDouble();
    origin.setX(x_origin);
    origin.setY(y_origin);
    origin.setZ(0.0);
    data.push_back(x_origin);
    data.push_back(y_origin);
    data.push_back(0.0);
    while(!file.atEnd()) {
        auto        line = file.readLine().split(' ');
        const float x    = line[3].toFloat();
        const float y    = line[4].toFloat();
        data.push_back(x);
        data.push_back(y);
        data.push_back(0.0);
    }
    qDebug("Map file already loaded!Enjoy!");
}

void SemanticMap::initGL() {
    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();
    vao                 = new QOpenGLVertexArrayObject;
    if(vao->create()) vao->bind();

    vbo = new QOpenGLBuffer;
    vbo->create();
    vbo->bind();
    vbo->setUsagePattern(QOpenGLBuffer::StaticDraw);
    vbo->allocate(data.constData(), count() * sizeof(GLfloat));
    f->glEnableVertexAttribArray(0);
    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
    vbo->release();
    vao->release();
}

void SemanticMap::paint(QOpenGLShaderProgram* program) {
    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();
    vao->bind();
    QMatrix4x4 model;
    program->setUniformValue("model", model);
    program->setUniformValue("color", QVector4D(0, 255, 0, 0.7));

    // Now call a function introduced in OpenGL 3.1 / OpenGL ES 3.0. We
    // requested a 3.3 or ES 3.0 context, so we know this will work.
    f->glDrawArrays(GL_LINE_STRIP, 0, vertexCount());
    vao->release();
}

SemanticMap::~SemanticMap() {
    if(vao) delete vao;
    if(vbo) delete vbo;
};

TiEVCar::TiEVCar(const double length, const double width, const double height) {
    // a car is a box with 8 vertex
    /**
     *                   ^y
     *         1 ________|________2
     *          |        |        |
     *----------|--------|--------|----->x
     *         0|________|________|3
     *                   |
     **/
    const auto add_point = [&](float x, float y, float z) {
        data.push_back(x);
        data.push_back(y);
        data.push_back(z);
    };
    add_point(-length / 2, -width / 2, 0);
    add_point(-length / 2, width / 2, 0);
    add_point(length / 2, width / 2, 0);
    add_point(length / 2, -width / 2, 0);

    add_point(-length / 2, -width / 2, height);
    add_point(-length / 2, width / 2, height);
    add_point(length / 2, width / 2, height);
    add_point(length / 2, -width / 2, height);
    index_data = QVector<GLuint>({ 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6,
                                   6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 });
}

void TiEVCar::initGL() {
    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();
    vao                 = new QOpenGLVertexArrayObject;
    if(vao->create()) vao->bind();

    vbo = new QOpenGLBuffer;
    vbo->create();
    vbo->bind();
    vbo->setUsagePattern(QOpenGLBuffer::StaticDraw);
    vbo->allocate(data.constData(), count() * sizeof(GLfloat));
    ebo = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
    ebo->create();
    ebo->bind();
    ebo->allocate(index_data.constData(), index_data.size() * sizeof(GLuint));
    f->glEnableVertexAttribArray(0);
    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
    vbo->release();
    vao->release();
    ebo->release();
}

void TiEVCar::paint(QOpenGLShaderProgram* program) {
    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();
    vao->bind();
    QMatrix4x4 model;
    model.translate(pose.x(), pose.y());                      // car position
    model.rotate(pose.z() / M_PI * 180, QVector3D(0, 0, 1));  // car heading;
    program->setUniformValue("model", model);
    program->setUniformValue("color", QVector4D(155, 1555, 0, 1.0));

    // Now call a function introduced in OpenGL 3.1 / OpenGL ES 3.0. We
    // requested a 3.3 or ES 3.0 context, so we know this will work.
    f->glDrawElements(GL_LINES, index_data.size(), GL_UNSIGNED_INT, 0);
    vao->release();
}

void TiEVCar::setPose(const double x, const double y, const double heading) {
    pose.setX(x);
    pose.setY(y);
    pose.setZ(heading);
    emit poseChanged();
}

TiEVCar::~TiEVCar() {
    if(ebo) delete ebo;
    if(vbo) delete vbo;
    if(vao) delete vao;
}