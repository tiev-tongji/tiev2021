#pragma once

#include "../path_time_graph/path_time_graph.h"
#include "../speed/speed_data.h"
#include "../speed/speed_limit.h"
#include "st_graph_point.h"
#include "dp_st_cost.h"
#include "dp_st_config.h"
#include <vector>

namespace TiEV {

/**
 * @class
 * @brief Gridded path-time graph for DP speed optimization.
 *        With STBoundaries calculated in PathTimeGraph and some other infos,
 *        Costs of every cell can be calculated through dynamic programming.
 */
class GriddedPathTimeGraph {
private:
    PathTimeGraph& st_data_;
    const std::vector<Obstacle>& obstacles_;

    // Initial status
    Point init_point_;

    // cost_table_[t][s]
    std::vector<std::vector<StGraphPoint> > cost_table_;

    // configuration for dp search
    DpStConfig dp_st_config_;

    double unit_s_ = 0.0;
    double unit_t_ = 0.0;

    // dp cost calculation
    DpStCost dp_st_cost_;

    // speed limit on path
    SpeedLimit speed_limit_;

public:
    GriddedPathTimeGraph(PathTimeGraph& st_data,
                         const std::vector<Obstacle>& obstacles,
                         const Point& init_point,
                         const SpeedLimit& speed_limit)
                         : st_data_(st_data),
                           obstacles_(obstacles),
                           init_point_(init_point),
                           dp_st_config_(),
                           unit_s_(st_data_.path_length() / (dp_st_config_.matrix_dimension_s() - 1)),
                           unit_t_(st_data_.total_time() / (dp_st_config_.matrix_dimension_t() - 1)),
                           dp_st_cost_(dp_st_config_, obstacles, unit_t_),
                           speed_limit_(speed_limit) {}

    /**
     * @brief Dynamic programming search
     * @param speed_data Retrieved speed profile
     * @return Whether success
     */
    bool Search(SpeedData* speed_data);

    /**
     * @brief Getter
     */
    const std::vector<std::vector<StGraphPoint> >& cost_table() {
        return cost_table_;
    }

private:

    /**
     * @brief Init the cost table
     * @return Whether success
     */
    bool InitCostTable();

    /**
     * @brief Calculate the total cost for every possible speed profile
     * @return Whether success
     */
    bool CalculateTotalCost();

    /**
     * @brief Retrieve the best speed profile based on the calculated costs
     * @param speed_data Pointer to the favorable speed profile
     * @return Whether success
     */
    bool RetrieveSpeedProfile(SpeedData* speed_data);


    /**
     * @brief Calculate the cost of one certain cell
     * @param c Column - time
     * @param r Row - Path
     */
    void CalculateCostAt(size_t c, size_t r);

    /**
     * @brief Calculate the row-range for
     *        search in the next column
     * @param point
     * @param next_higheet_row
     * @param next_lowest_row
     */
    void GetRowRange(const StGraphPoint& point,
                     size_t* next_highest_row,
                     size_t* next_lowest_row);

    /**
     * @brief Calculate the cost of edge in speed profile
     *        based on speed cost, acceleration cost and
     *        jerk cost
     * @param first The first point
     * @param second
     * @param third
     * @param fourth
     * @param speed_limit
     * @return Edge cost
     */
    double CalculateEdgeCost(const STPoint& first,
                             const STPoint& second,
                             const STPoint& third,
                             const STPoint& fourth,
                             const double speed_limit);

    /**
     * @brief Edge cost calculation used for cells in the second column
     * @param row
     * @param speed_limit
     * @return Edge cost
     */
    double CalculateEdgeCostForSecondCol(const size_t row, const double speed_limit);

    /**
     * @brief Edge cost calculation used for cells in the third column
     * @param curr_row
     * @param pre_row
     * @param speed_limit
     * @return Edge cost
     */
    double CalculateEdgeCostForThirdCol(const size_t curr_row,
                                        const size_t pre_row,
                                        const double speed_limit);
};
}
