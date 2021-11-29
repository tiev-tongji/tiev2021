#ifndef CAMERA_H
#define CAMERA_H

#include <QKeyEvent>
#include <QMatrix4x4>
#include <QVector3D>
#include <math.h>

class Camera {
public:
    Camera(){};
    Camera(const QVector3D& world_up);
    ~Camera(){};

    void processMouseMovement(const double x_offset, const double y_offset);
    void rotateCamera(const double x_offset, const double y_offset);
    void moveCamera(const double x_offset, const double y_offset);
    void processMouseScroll(const double offset);
    void processKeyPress(QKeyEvent* event);
    void recordOriginPosition();
    void recordOriginYawPitch();

    void setPosition(const QVector3D& position_);
    void setTarget(const QVector3D& target_);
    void setYaw(const double yaw_);
    void setPitch(const double pitch_);

    QMatrix4x4       getViewMaxtrix() const;
    double           getZoom() const;
    const QVector3D& getPosition() const {
        return position;
    };
    bool lock2d() const {
        return _lock2d;
    };

private:
    void update();

private:
    QVector3D worldUp;
    QVector3D position;
    QVector3D target;
    QVector3D direction;
    QVector3D up;
    QVector3D right;
    QVector3D drag_origin_position;
    double    rotate_origin_yaw;
    double    rotate_origin_pitch;

    double yaw   = 0.0;
    double pitch = -M_PI_4;

    double zoom    = 45.0;
    bool   _lock2d = false;
};

#endif
