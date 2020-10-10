#pragma once

namespace TiEV {

/**
 * @class STPoint
 * @brief point on the S-T graph
 */
class STPoint {
private:
    double s_;
    double t_;

public:
    STPoint();

    STPoint(double s, double t);

    STPoint& operator=(const STPoint& st_point) = default;

    void set_s(double s);

    void set_t(double t);

    double s() const;

    double t() const;
};
}
