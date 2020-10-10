#ifndef CLUSTER_H
#define CLUSTER_H

#include "common.h"


namespace cluster {

    template <typename T>
    using Cluster_ = std::vector<cv::Point_<T>>;

    typedef Cluster_<int> Cluster;

    void cluster_lanes(
            cv::InputArray lane_mask, // [in]
            std::vector<Cluster> & clusters // [out]
    );

    template <typename T>
    bool less_than(const Cluster_<T>& c1, const Cluster_<T>& c2){
        return c1[0].x > c2[0].x;
    }

    inline cv::Vec3b random_color() {
        static uchar_t hue = 0;
        hue = (hue + 41) % 180;
        cv::Mat hsv(1,1, CV_8UC3, cv::Vec3b(hue, 150, 150)), bgr;
        cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
        return cv::Vec3b(bgr.data[0], bgr.data[1], bgr.data[2]);
    }

    template <typename T>
    inline cv::Point toIntPoint(cv::Point_<T> point){
        return cv::Point(
                    static_cast<int>(point.x + .5),
                    static_cast<int>(point.y + .5));
    }

}

#endif
