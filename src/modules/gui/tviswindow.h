#ifndef GLWIDGET_H
#define GLWIDGET_H

#include "camera.h"
#include "coordinate_converter.h"
#include "tiev_elements.h"
#include <QMatrix4x4>
#include <QOpenGLWindow>
#include <QVector3D>

class QOpenGLTexture;
class QOpenGLShaderProgram;
class QOpenGLBuffer;
class QOpenGLVertexArrayObject;

class TVisWindow : public QOpenGLWindow {
    Q_OBJECT

public:
    TVisWindow() = delete;
    TVisWindow(SemanticMap* semantic_map_, const Camera& camera_,
               const CoodinateConverter& coordinate_converter);
    ~TVisWindow();

    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
public slots:
    void listeningTiEVMessage();

protected:
    // mouse and keyboard event
    void mouseMoveEvent(QMouseEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

private:
    // tiev elements
    SemanticMap* semantic_map;
    TiEVCar      tiev_car;

private:
    QOpenGLShaderProgram*     m_program;
    QOpenGLBuffer*            m_vbo;
    QOpenGLVertexArrayObject* m_vao;

    int m_projMatrixLoc;
    int m_camMatrixLoc;
    int m_worldMatrixLoc;
    int m_color;

    QVector3D          worldOriginPoint;
    Camera             camera;
    CoodinateConverter coordinate_converter;
    struct Mouse {
        double last_x      = -1;
        double last_y      = -1;
        bool   left_press  = false;
        bool   mid_press   = false;
        bool   right_press = false;
    } mouse;
};

#endif