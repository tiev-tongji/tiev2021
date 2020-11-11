#pragma once

#include "dp_st_config.h"
#include "path_time_graph/path_time_graph.h"
#include "st_graph_point.h"
#include <vector>

namespace TiEV {

class DpStCost {
private:
    // dp st configuration
    DpStConfig            dp_st_config_;
    std::vector<Obstacle> obstacles_;
    double                unit_t_;

public:
    DpStCost() = delete;

    DpStCost(const DpStConfig& dp_st_config, const std::vector<Obstacle>& obstacles, double unit_t) : dp_st_config_(dp_st_config), obstacles_(obstacles), unit_t_(unit_t) {}

    /**
     * @brief Calculate the obstacle cost of a given cell
     * @param point The given gridded s-t graph cell
     * @return The obstacle cost
     */
    double GetObstacleCost(const StGraphPoint& st_graph_point);

    /**
     * @brief Calculate the speed cost by two given cell
     * @param first The first s-t graph cell
     * @param second The second s-t graph cell
     * @param speed_limit Speed limit
     * @return The speed cost
     */
    double GetSpeedCost(const STPoint& first, const STPoint& second, const double speed_limit);

    /**
     * @brief Calculate acceleration cost by two given cells
     * @param pre_speed The previous speed
     * @param first The first s-t graph cell
     * @param second The second s-t graph cell
     * @return The acceleration cost
     */
    double GetAccelCostByTwoPoints(const double pre_speed, const STPoint& first, const STPoint& second);

    /**
     * @brief Calculate acceleration cost by three given cells
     * @param first The first s-t graph cell
     * @param second The second s-t graph cell
     * @param third The third s-t graph cell
     * @return The acceleration cost
     */
    double GetAccelCostByThreePoints(const STPoint& first, const STPoint& second, const STPoint& third);

    /**
     * @brief Calculate jerk cost by two given cells
     * @param pre_speed The previous speed
     * @param pre_acc The previous acceleration
     * @param pre_point The first point
     * @param curr_point The second point
     * @return The jerk cost
     */
    double GetJerkCostByTwoPoints(const double pre_speed, const double pre_acc, const STPoint& pre_point, const STPoint& curr_point);

    /**
     * @brief Calculate jerk cost by three given cells
     * @param first_speed The speed of the first point
     * @param first_point The first point
     * @param second_point The second point
     * @param third_point The third point
     * @return The jerk cost
     */
    double GetJerkCostByThreePoints(const double first_speed, const STPoint& first_point, const STPoint& second_point, const STPoint& third_point);

    /**
     * @brief Calculate jerk cost by four given cells
     * @param first The first point
     * @param second The second point
     * @param third The third point
     * @param fourth The fourth point
     * @return The jerk cost
     */
    double GetJerkCostByFourPoints(const STPoint& first, const STPoint& second, const STPoint& third, const STPoint& fourth);

    /**
     * @brief Calculate the cost according to acceleration
     * @param accel Acceleration
     * @return Acceleration cost
     */
    double GetAccelCost(const double accel);

    /**
     * @brief Calculate the cost according to jerk
     * @param jerk Jerk
     * @return Jerk cost
     */
    double GetJerkCost(const double jerk);
};
}
