#ifndef COORDINATE_CONVERTER_H
#define COORDINATE_CONVERTER_H
#include <QMatrix4x4>
#include <QVector3D>

class CoodinateConverter {
public:
    CoodinateConverter(){};
    CoodinateConverter(const QVector3D& world_origin_);
    ~CoodinateConverter(){};

    void             setWolrdOrigin(const QVector3D& world_origin_);
    const QVector3D& getWolrdOrigin() const;
    QMatrix4x4       getWorldModel(const QVector3D& point, const double yaw);

private:
    QVector3D world_origin;
};

#endif
