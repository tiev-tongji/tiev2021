#ifndef _NATURE_ANGLE_
#define _NATURE_ANGLE_

namespace TiEV{

class Angle{
protected:
    double val;

public:
    enum Dirction{
        CLOCKWISE,
        ANTICLOCKWISE
    };

    const static double PI;

    void setByDegree(double angle, const double offsetOfeast = 0, const Dirction D = ANTICLOCKWISE);
    void setByRad(double angle, const double offsetOfeast = 0, const Dirction D = ANTICLOCKWISE);
    double getDegree() const;
    double getRad() const;
    double cos() const;
    double sin() const;
    double tan() const;
};

}

#endif // _ANTURE_ANGLE_
