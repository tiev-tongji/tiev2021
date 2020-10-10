#pragma

#include "../../message_manager/message_manager.h"
#include "../path_time_graph/st_boundary.h"
#include "../math/linear_interpolation.h"

namespace TiEV {
/**
*@file
*@brief class Obstacle definition for temporary use in speed planning
**/

class Obstacle : public DynamicObj {
private:
    STBoundary st_boundary_;

public:
    Obstacle() = default;

    virtual ~Obstacle() = default;

    explicit Obstacle(const DynamicObj& obj) :
        DynamicObj(obj),
        st_boundary_() {
        for (auto& point : path) {
            point.angle.setByRad(NormalizeAngle(Angle::PI - point.angle.getRad()));
        }
    }

    void set_st_boundary(const STBoundary& st_boundary) { st_boundary_ = st_boundary; }

    const STBoundary& st_boundary() const { return st_boundary_; }
};
}
