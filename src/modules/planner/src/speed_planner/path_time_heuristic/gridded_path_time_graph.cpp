#include "path_time_heuristic/gridded_path_time_graph.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace TiEV {

bool GriddedPathTimeGraph::Search(SpeedData* speed_data) {
    const double kBoundaryEpsilon = 1e-2;

    /**
     * In the situation below, the vehicle can't move at the very beginning
     */
    for(const auto& boundary : st_data_.st_boundaries()) {
        if(boundary.IsPointInBoundary({ 0.0, 0.0 }) || (std::fabs(boundary.min_t()) < kBoundaryEpsilon && std::fabs(boundary.min_s()) < kBoundaryEpsilon)) {
            std::vector<SpeedPoint> speed_profile;
            double                  t = 0.0;
            for(int i = 0; i < dp_st_config_.matrix_dimension_t(); i++, t += unit_t_) {
                SpeedPoint speed_point;
                speed_point.set_s(0.0);
                speed_point.set_t(t);
                speed_point.set_v(0.0);
                speed_point.set_a(0.0);
                speed_profile.emplace_back(speed_point);
            }
            *speed_data = SpeedData(speed_profile);
            return true;
        }
    }

    if(!InitCostTable()) {
        std::cout << "Initialize cost table failed." << std::endl;
        return false;
    }

    if(!CalculateTotalCost()) {
        std::cout << "Calculate total cost failed." << std::endl;
        return false;
    }

    if(!RetrieveSpeedProfile(speed_data)) {
        std::cout << "Retrieve best speed profile failed." << std::endl;
        std::vector<SpeedPoint> speed_profile;
        double                  t = 0.0;
        for(int i = 0; i < dp_st_config_.matrix_dimension_t(); i++, t += unit_t_) {
            SpeedPoint speed_point;
            /*
            if (i == 0) {
                speed_point.set_s(0.0);
            } else {
                speed_point.set_s(std::max(speed_profile[i - 1].s(), init_point_.v * t + 0.5 * dp_st_config_.max_deceleration() * t * t));
            }
            */

            /////
            speed_point.set_s(0.0);
            speed_point.set_v(0.0);
            speed_point.set_t(t);

            // speed_point.set_v(max(0.0, init_point_.v + dp_st_config_.max_deceleration() * t));
            speed_point.set_a(0.0);
            speed_profile.emplace_back(speed_point);
        }
        *speed_data = SpeedData(speed_profile);
    }

    // Classify st boundaries based on speed profile
    for(auto& boundary : st_data_.st_boundaries()) {
        double t  = boundary.bottom_left_point().t();
        double bs = boundary.bottom_left_point().s();
        double us = boundary.upper_left_point().s();
        // std::cout << "ST boundary starts at " << t << ' ';

        if(t > speed_data->TotalTime()) {
            boundary.set_boundary_type(STBoundary::BoundaryType::UNKNOWN);
            continue;
        }
        if(bs > speed_data->TotalLength()) {
            boundary.set_boundary_type(STBoundary::BoundaryType::YIELD);

            // std::cout << "Boundary type: YIELD" << std::endl;
            continue;
        }
        double s_on_speed_data = speed_data->GetSByTime(t);

        if(s_on_speed_data > us) {  // st boundary beneath the speed profile
            boundary.set_boundary_type(STBoundary::BoundaryType::OVERTAKE);
            // std::cout << "Boundary type: OVERTAKE" << std::endl;
        }
        else if(s_on_speed_data < bs) {  // st boundary above the speed profile
            boundary.set_boundary_type(STBoundary::BoundaryType::YIELD);
            // std::cout << "Boundary type: YIELD" << std::endl;
        }
        else {
            boundary.set_boundary_type(STBoundary::BoundaryType::CAR_COLLISION);
            // std::cout << "Boundary type: COLLISION" << std::endl;
        }
    }
    return true;
}

bool GriddedPathTimeGraph::InitCostTable() {
    uint32_t dim_s = dp_st_config_.matrix_dimension_s();
    uint32_t dim_t = dp_st_config_.matrix_dimension_t();

    if(dim_s < 2 || dim_t < 2) return false;

    cost_table_ = std::vector<std::vector<StGraphPoint>>(dim_t, std::vector<StGraphPoint>(dim_s, StGraphPoint()));

    double curr_t = 0.0;
    for(uint32_t i = 0; i < cost_table_.size(); i++) {
        auto&  cost_table_i = cost_table_[i];
        double curr_s       = 0.0;
        for(uint32_t j = 0; j < cost_table_i.size(); j++) {
            cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
            curr_s += unit_s_;
        }
        curr_t += unit_t_;
    }
    return true;
}

bool GriddedPathTimeGraph::CalculateTotalCost() {
    size_t next_highest_row = 0;
    size_t next_lowest_row  = 0;

    for(size_t c = 0; c < cost_table_.size(); c++) {
        size_t highest_row = 0;
        size_t lowest_row  = cost_table_.back().size() - 1;

        int count = static_cast<int>(next_highest_row) - static_cast<int>(next_lowest_row) + 1;
        if(count > 0) {
            for(size_t r = next_lowest_row; r <= next_highest_row; r++) {
                CalculateCostAt(c, r);
            }
        }

        for(size_t r = next_lowest_row; r <= next_highest_row; r++) {
            const auto& cost_cr = cost_table_[c][r];
            if(cost_cr.total_cost() < std::numeric_limits<double>::infinity()) {
                size_t h_r = 0;
                size_t l_r = 0;

                GetRowRange(cost_cr, &h_r, &l_r);
                highest_row = std::max(highest_row, h_r);
                lowest_row  = std::min(lowest_row, l_r);
            }
        }
        next_highest_row = highest_row;
        next_lowest_row  = lowest_row;
    }
    return true;
}

void GriddedPathTimeGraph::GetRowRange(const StGraphPoint& point, size_t* next_highest_row, size_t* next_lowest_row) {
    double v0 = 0.0;
    if(!point.pre_point()) {
        v0 = init_point_.v;
    }
    else {
        v0 = (point.index_s() - point.pre_point()->index_s()) * unit_s_ / unit_t_;
    }

    const auto max_s_size = cost_table_.back().size() - 1;

    const double speed_coeff = unit_t_ * unit_t_;

    const double delta_s_upper_bound = v0 * unit_t_ + 0.5 * dp_st_config_.max_acceleration() * speed_coeff;

    // Debug(Charles Peng):
    // std::cout << "Next upper bound(m): " << delta_s_upper_bound << ' ';

    *next_highest_row = point.index_s() + static_cast<int>(delta_s_upper_bound / unit_s_);
    if(*next_highest_row >= max_s_size) {
        *next_highest_row = max_s_size;
    }

    const double delta_s_lower_bound = v0 * unit_t_ + 0.5 * dp_st_config_.max_deceleration() * speed_coeff;

    // Debug(Charles Peng):
    // std::cout << "Next lower bound(m): " << delta_s_lower_bound << ' ' << std::endl;

    int nlr = static_cast<int>(point.index_s()) + static_cast<int>(delta_s_lower_bound / unit_s_);
    if(nlr > static_cast<int>(max_s_size)) {
        *next_lowest_row = max_s_size;
    }
    else if(nlr < 0) {
        *next_lowest_row = 0;
    }
    else {
        *next_lowest_row = nlr;
    }
}

double GriddedPathTimeGraph::CalculateEdgeCost(const STPoint& first, const STPoint& second, const STPoint& third, const STPoint& fourth, const double speed_limit) {
    double speed_cost = dp_st_cost_.GetSpeedCost(third, fourth, speed_limit);
    double accel_cost = dp_st_cost_.GetAccelCostByThreePoints(second, third, fourth);
    double jerk_cost  = dp_st_cost_.GetJerkCostByFourPoints(first, second, third, fourth);
    return speed_cost + accel_cost + jerk_cost;
}

double GriddedPathTimeGraph::CalculateEdgeCostForSecondCol(const size_t row, const double speed_limit) {
    double         init_speed = init_point_.v;
    double         init_acc   = init_point_.a;
    const STPoint& pre_point  = cost_table_[0][0].st_point();
    const STPoint& curr_point = cost_table_[1][row].st_point();

    double speed_cost = dp_st_cost_.GetSpeedCost(pre_point, curr_point, speed_limit);
    double accel_cost = dp_st_cost_.GetAccelCostByTwoPoints(init_speed, pre_point, curr_point);
    double jerk_cost  = dp_st_cost_.GetJerkCostByTwoPoints(init_speed, init_acc, pre_point, curr_point);
    return speed_cost + accel_cost + jerk_cost;
}

double GriddedPathTimeGraph::CalculateEdgeCostForThirdCol(const size_t curr_row, const size_t pre_row, const double speed_limit) {
    double         init_speed = init_point_.v;
    const STPoint& first      = cost_table_[0][0].st_point();
    const STPoint& second     = cost_table_[1][pre_row].st_point();
    const STPoint& third      = cost_table_[2][curr_row].st_point();

    double speed_cost = dp_st_cost_.GetSpeedCost(second, third, speed_limit);
    double accel_cost = dp_st_cost_.GetAccelCostByThreePoints(first, second, third);
    double jerk_cost  = dp_st_cost_.GetJerkCostByThreePoints(init_speed, first, second, third);
    return speed_cost + accel_cost + jerk_cost;
}

void GriddedPathTimeGraph::CalculateCostAt(size_t c, size_t r) {
    auto& cost_cr = cost_table_[c][r];
    cost_cr.SetObstacleCost(dp_st_cost_.GetObstacleCost(cost_cr));
    if(cost_cr.obstacle_cost() == std::numeric_limits<double>::infinity()) {
        return;
    }

    const auto& cost_init = cost_table_[0][0];

    // The first initial cell has a cost of zero
    if(c == 0) {
        cost_cr.SetTotalCost(0.0);
        return;
    }

    // TODO(all): Set speed limit on the path
    const double speed_limit = speed_limit_.GetSpeedLimit(unit_s_ * r);
    // const double speed_limit = 16; // Temporarily set

    // The cell is at the first column
    if(c == 1) {
        const double acc = (r * unit_s_ / unit_t_ - init_point_.v) / unit_t_;
        if(acc < dp_st_config_.max_deceleration() || acc > dp_st_config_.max_acceleration()) {
            return;
        }

        for(const auto& st_boundary : st_data_.st_boundaries()) {
            if(st_boundary.HasOverlap(cost_cr.st_point(), cost_init.st_point())) {
                return;
            }
        }

        double cost = cost_cr.obstacle_cost() + cost_init.total_cost() + CalculateEdgeCostForSecondCol(r, speed_limit);
        cost_cr.SetTotalCost(cost);
        cost_cr.SetPrePoint(cost_init);
        return;
    }

    const double SpeedRangeBuffer = 0.20;
    const auto   max_s_diff       = static_cast<size_t>(dp_st_config_.vehicle_upper_speed_limit() * (1 + SpeedRangeBuffer) * unit_t_ / unit_s_);
    const size_t r_low            = (max_s_diff < r ? r - max_s_diff : 0);

    const auto& pre_col = cost_table_[c - 1];

    double curr_speed_limit = speed_limit;
    if(c == 2) {
        for(size_t r_pre = r_low; r_pre <= r; ++r_pre) {
            // TODO(all): Set speed limit
            curr_speed_limit = std::fmin(curr_speed_limit, speed_limit_.GetSpeedLimit(unit_s_ * r_pre));
            const double acc = (r * unit_s_ - 2 * r_pre * unit_s_) / (unit_t_ * unit_t_);
            if(acc < dp_st_config_.max_deceleration() || acc > dp_st_config_.max_acceleration()) {
                continue;
            }

            bool hasOverLap = false;
            for(const auto& st_boundary : st_data_.st_boundaries()) {
                if(st_boundary.HasOverlap(cost_cr.st_point(), pre_col[r_pre].st_point())) {
                    hasOverLap = true;
                    break;
                }
            }
            if(hasOverLap) continue;

            double cost = cost_cr.obstacle_cost() + pre_col[r_pre].total_cost() + CalculateEdgeCostForThirdCol(r, r_pre, curr_speed_limit);

            if(cost < cost_cr.total_cost()) {
                cost_cr.SetTotalCost(cost);
                cost_cr.SetPrePoint(pre_col[r_pre]);
            }
        }
        return;
    }

    for(size_t r_pre = r_low; r_pre <= r; ++r_pre) {
        if(pre_col[r_pre].total_cost() == std::numeric_limits<double>::infinity() || pre_col[r_pre].pre_point() == nullptr) {
            continue;
        }

        // TODO(all): Set speed limit
        curr_speed_limit    = std::fmin(curr_speed_limit, speed_limit_.GetSpeedLimit(unit_s_ * r_pre));
        const double curr_a = (cost_cr.index_s() * unit_s_ + pre_col[r_pre].pre_point()->index_s() * unit_s_ - 2 * pre_col[r_pre].index_s() * unit_s_) / (unit_t_ * unit_t_);
        if(curr_a > dp_st_config_.max_acceleration() || curr_a < dp_st_config_.max_deceleration()) {
            continue;
        }

        bool hasOverLap = false;
        for(const auto& st_boundary : st_data_.st_boundaries()) {
            if(st_boundary.HasOverlap(cost_cr.st_point(), pre_col[r_pre].st_point())) {
                hasOverLap = true;
                break;
            }
        }
        if(hasOverLap) continue;

        size_t              r_prepre           = pre_col[r_pre].pre_point()->index_s();
        const StGraphPoint& prepre_graph_point = cost_table_[c - 2][r_prepre];
        if(prepre_graph_point.total_cost() == std::numeric_limits<double>::infinity()) {
            continue;
        }

        if(prepre_graph_point.pre_point() == nullptr) {
            continue;
        }
        const STPoint& triple_pre_point = prepre_graph_point.pre_point()->st_point();
        const STPoint& prepre_point     = prepre_graph_point.st_point();
        const STPoint& pre_point        = pre_col[r_pre].st_point();
        const STPoint& curr_point       = cost_cr.st_point();

        double cost = cost_cr.obstacle_cost() + pre_col[r_pre].total_cost() + CalculateEdgeCost(triple_pre_point, prepre_point, pre_point, curr_point, curr_speed_limit);

        if(cost < cost_cr.total_cost()) {
            cost_cr.SetTotalCost(cost);
            cost_cr.SetPrePoint(pre_col[r_pre]);
        }
    }
}

bool GriddedPathTimeGraph::RetrieveSpeedProfile(SpeedData* speed_data) {
    double              min_cost       = std::numeric_limits<double>::infinity();
    const StGraphPoint* best_end_point = nullptr;

    for(const StGraphPoint& cur_point : cost_table_.back()) {
        if(cur_point.total_cost() != std::numeric_limits<double>::infinity() && cur_point.total_cost() < min_cost) {
            best_end_point = &cur_point;
            min_cost       = cur_point.total_cost();
        }
    }

    for(const auto& row : cost_table_) {
        const StGraphPoint& cur_point = row.back();
        /*
        if (cur_point.total_cost() != std::numeric_limits<double>::infinity()) {
            std::cout << cur_point.total_cost() << ' ';
        } else {
            std::cout << "INF" << ' ';
        }
        std::cout << std::endl;
        */
        if(cur_point.total_cost() != std::numeric_limits<double>::infinity() && cur_point.total_cost() < min_cost) {
            best_end_point = &cur_point;
            min_cost       = cur_point.total_cost();
        }
    }

    if(best_end_point == nullptr) {
        // std::cout << "RetrieveSpeedProfile: No best_end_point found!" << std::endl;
        return false;
    }

    // std::cout << "Best end point: " << std::endl;
    // std::cout << best_end_point->total_cost() << ' ' << best_end_point->st_point().s() << best_end_point->st_point().t() << std::endl;

    std::vector<SpeedPoint> speed_profile;
    const StGraphPoint*     cur_point = best_end_point;
    while(cur_point != nullptr) {
        SpeedPoint speed_point;
        speed_point.set_s(cur_point->st_point().s());
        speed_point.set_t(cur_point->st_point().t());
        speed_profile.emplace_back(speed_point);
        cur_point = cur_point->pre_point();
    }
    std::reverse(speed_profile.begin(), speed_profile.end());

    // The first s-t point is not at the initial coordinate
    const double Eps = std::numeric_limits<double>::epsilon();
    if(speed_profile.front().t() > Eps || speed_profile.front().s() > Eps) {
        // std::cout << "RetrieveSpeedProfile: The first s-t point is not at the initial coordinate!" << std::endl;
        return false;
    }

    for(size_t i = 0; i + 1 < speed_profile.size(); ++i) {
        const double v = (speed_profile[i + 1].s() - speed_profile[i].s()) / (speed_profile[i + 1].t() - speed_profile[i].t());
        speed_profile[i].set_v(v);
    }
    /*
    size_t last_idx = speed_profile.size() - 1;
    double delta_t = speed_profile[last_idx].t() - speed_profile[last_idx - 1].t();
    double last_speed = speed_profile[last_idx - 1].v() + dp_st_config_.max_deceleration() * delta_t;
    speed_profile.back().set_v(last_speed);
    */
    speed_profile.back().set_v(0);
    // Debug
    /*
    std::cout << "DP retrieved speed profile: " << std::endl;
    for (const auto& point : speed_profile) {
       std::cout << "s = " << point.s() << " t = " << point.t() << " v = " << point.v() << std::endl;
    }
    */

    *speed_data = SpeedData(speed_profile);
    return true;
}
}
