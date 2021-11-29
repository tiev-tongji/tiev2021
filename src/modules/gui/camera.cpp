#include "camera.h"
#include "math_util.h"
#include <QDebug>
#include <math.h>

Camera::Camera(const QVector3D& world_up) {
    worldUp  = world_up;
    position = QVector3D(0, 0, 500);
    update();
}

void Camera::processMouseMovement(const double x_offset,
                                  const double y_offset) {
    yaw += x_offset;
    pitch += y_offset;
    update();
}

void Camera::rotateCamera(const double x_offset, const double y_offset) {
    yaw = math_util::normalizeAngleRad(rotate_origin_pitch - 0.001 * x_offset);
    pitch = std::clamp((rotate_origin_pitch - 0.001 * y_offset), -M_PI_2 + 0.01,
                       M_PI_2 - 0.01);
    update();
}

void Camera::moveCamera(const double x_offset, const double y_offset) {
    auto x_direction = QVector3D(right.x(), right.y(), 0.0).normalized();
    auto y_direction = QVector3D(up.x(), up.y(), 0.0).normalized();
    position         = drag_origin_position
               + 0.00002 * drag_origin_position.z() * zoom
                     * (-x_offset * x_direction + y_offset * y_direction);
}

void Camera::processMouseScroll(const double offset) {
    // offset step = 120
    zoom -= offset / 1200.0 * zoom;
    if(zoom < 0.1) zoom = 0.1;
    if(zoom > 45.0) zoom = 45.0;
}

void Camera::processKeyPress(QKeyEvent* event) {
    if(event->key() == Qt::Key_Tab) {
        _lock2d = !_lock2d;
    }
    update();
}

void Camera::recordOriginPosition() {
    drag_origin_position = position;
}

void Camera::recordOriginYawPitch() {
    rotate_origin_yaw   = yaw;
    rotate_origin_pitch = pitch;
}

QMatrix4x4 Camera::getViewMaxtrix() const {
    QMatrix4x4 view;
    view.lookAt(position, target, up);
    // view.lookAt(QVector3D(0, 0, 500), QVector3D(0, 0, -1), QVector3D(0, 1,
    // 0));
    qDebug() << "direction:" << direction << " right:" << right << " up:" << up
             << " position:" << position;
    return view;
}

double Camera::getZoom() const {
    return zoom;
}

void Camera::setPosition(const QVector3D& position_) {
    position = position_;
}

void Camera::setTarget(const QVector3D& target_) {
    target    = target_;
    direction = (target - position).normalized();
    right     = QVector3D::crossProduct(direction, worldUp).normalized();
    up        = QVector3D::crossProduct(right, direction).normalized();
}

void Camera::setYaw(const double yaw_) {
    yaw = yaw_;
    update();
}

void Camera::setPitch(const double pitch_) {
    pitch = pitch_;
    update();
}

void Camera::update() {
    if(_lock2d) {
        direction.setX(0.0);
        direction.setY(0.0);
        direction.setZ(-1.0);
        right.setZ(0.0);
        right.normalize();
        position.setZ(100);
    }
    else {
        direction.setX(std::cos(yaw) * std::cos(pitch));
        direction.setY(std::sin(yaw) * std::cos(pitch));
        direction.setZ(std::sin(pitch));
        direction.normalize();
        right = QVector3D::crossProduct(direction, worldUp).normalized();
        // position.setZ(10);
    }
    up     = QVector3D::crossProduct(right, direction).normalized();
    target = position + direction;
}