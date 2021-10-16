#pragma once

#include "st_point.h"

namespace TiEV {
/**
 * @class LineSegment
 * @brief Linesegment on S-T graph
 */
class LineSegment {
private:
    STPoint start_;
    STPoint end_;

public:
    LineSegment() = default;

    LineSegment(const STPoint& start, const STPoint& end) {
        start_ = start;
        end_ = end;
    }

    const STPoint& start() const { return start_; }

    const STPoint& end() const { return end_; }

    /**
     * @brief Check if two line segments intersect
     * @param lineSeg1 The first line segment
     * @param lineSeg2 The second line segment
     * @return True or false
     */
    friend bool LineSegIntersec(const LineSegment& lineSeg1, const LineSegment& lineSeg2);
};
}