#include "line_segment.h"
#include <algorithm>

namespace TiEV {

bool LineSegIntersec(const LineSegment& lineSeg1, const LineSegment& lineSeg2) {
    if (!(std::min(lineSeg1.start_.t(), lineSeg1.end_.t()) <=
        std::max(lineSeg2.start_.t(), lineSeg2.end_.t()) &&
        std::min(lineSeg2.start_.t(), lineSeg2.end_.t()) <=
        std::max(lineSeg1.start_.t(), lineSeg1.end_.t()) &&
        std::min(lineSeg1.start_.s(), lineSeg1.end_.s()) <=
        std::max(lineSeg2.start_.s(), lineSeg2.end_.s()) &&
        std::min(lineSeg2.start_.s(), lineSeg2.end_.s()) <=
        std::max(lineSeg1.start_.s(), lineSeg1.end_.s()))) {
        return false;
    }

    double u, v, w, z;
    u = (lineSeg2.start_.t() - lineSeg1.start_.t()) *
        (lineSeg1.end_.s() - lineSeg1.start_.s()) -
        (lineSeg1.end_.t() - lineSeg1.start_.t()) *
        (lineSeg2.start_.s() - lineSeg1.start_.s());

    v = (lineSeg2.end_.t() - lineSeg1.start_.t()) *
        (lineSeg1.end_.s() - lineSeg1.start_.s()) -
        (lineSeg1.end_.t() - lineSeg1.start_.t()) *
        (lineSeg2.end_.s() - lineSeg1.start_.s());

    w = (lineSeg1.start_.t() - lineSeg2.start_.t()) *
        (lineSeg2.end_.s() - lineSeg2.start_.s()) -
        (lineSeg2.end_.t() - lineSeg2.start_.t()) *
        (lineSeg1.start_.s() - lineSeg2.start_.s());

    z = (lineSeg1.end_.t() - lineSeg2.start_.t()) *
        (lineSeg2.end_.s() - lineSeg2.start_.s()) -
        (lineSeg2.end_.t() - lineSeg2.start_.t()) *
        (lineSeg1.end_.s() - lineSeg2.start_.s());

    return u * v <= 1e-9 && w * z <= 1e-9;
}
}