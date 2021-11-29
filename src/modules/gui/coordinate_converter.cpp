#include "coordinate_converter.h"

CoodinateConverter::CoodinateConverter(const QVector3D &world_origin_)
{
    world_origin = world_origin_;
}

void CoodinateConverter::setWolrdOrigin(const QVector3D &world_origin_)
{
    world_origin = world_origin_;
}

const QVector3D &CoodinateConverter::getWolrdOrigin() const
{
    return world_origin;
}

QMatrix4x4 CoodinateConverter::getWorldModel(const QVector3D &point, const double yaw)
{
    //TODO
    QMatrix4x4 world_model;
    world_model.setToIdentity();
    return world_model;
}