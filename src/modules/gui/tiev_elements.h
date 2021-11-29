#ifndef TIEV_ELEMENTS_H
#define TIEV_ELEMENTS_H

#include "message_manager.h"
#include <QOpenGLBuffer>
#include <QOpenGLContext>
#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QVector3D>
#include <QVector>
#include <qopengl.h>

class TiEVObject : public QObject {
public:
    const GLfloat* constData() const {
        return data.constData();
    }
    int count() const {
        return data.size();
    }

protected:
    QOpenGLVertexArrayObject* vao = nullptr;
    QOpenGLBuffer*            vbo = nullptr;
    QVector<GLfloat>          data;
};

class SemanticMap : public TiEVObject {
public:
    void initGL();
    void paint(QOpenGLShaderProgram* program);

public:
    SemanticMap(){};
    ~SemanticMap();
    SemanticMap(const QString& file_name);
    int vertexCount() const {
        return data.size() / 3;
    }

    const QVector3D& getOrigin() const {
        return origin;
    }

private:
    QVector3D origin;
};

class TiEVCar : public TiEVObject {
    Q_OBJECT

public:
    void initGL();
    void paint(QOpenGLShaderProgram* program);

public:
    TiEVCar(const double length = 5, const double width = 2,
            const double height = 1.8);
    ~TiEVCar();
    int vertexCount() const {
        return data.size() / 3;
    }
    void setPose(const double x, const double y, const double heading);
    const QVector3D& getPose() const {
        return pose;
    }

signals:
    void poseChanged();

private:
    QOpenGLBuffer*  ebo = nullptr;
    QVector<GLuint> index_data;
    QVector3D       pose;
};

#endif
