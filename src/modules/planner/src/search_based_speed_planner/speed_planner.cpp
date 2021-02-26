#include "speed_planner.h"
#include <functional>
#include <iostream>
#include <unordered_set>
#include "tiev_utils.h"

namespace TiEV {

const std::vector<Step> Step::children() const {
  std::vector<Step> children;
  for (const auto &a : acceleration) {
    double t = t_ + kT_interval;
    double s = s_ + v_ * kT_interval + a * kT_interval * kT_interval / 2;
    double v = v_ + a * kT_interval;
    double aa = a;
    if (v < 0 || t > 5) continue;
    children.emplace_back(s, t, v, aa);
    children.back().set_father(*this);
  }
  return children;
}

const std::vector<Step> Step::samples_from_father(
    const double t_interval) const {
  std::vector<Step> samples;
  if (!father_) return samples;
  double t = 0;
  while (t < kT_interval) {
    double s = father_->v() * t + a_ * t * t / 2;
    double v = father_->v() + a_ * t;
    Step step(father_->s() + s, father_->t() + t, v, a_);
    samples.push_back(step);
    samples.back().set_father(*this);
    t += t_interval;
  }
  return samples;
}

void SpeedPlanner::construct_st_box(
    const std::vector<DynamicObj> &dynamic_obj_list,
    const std::vector<Pose> *trajectory) {
  st_box_list.clear();
  for (const auto &obj : dynamic_obj_list) {
    std::vector<std::pair<double, double>> collision_st;
    for (int t = 0; t < obj.path.size(); ++t) {
      const auto &p = obj.path[t];
      if (point2LineDis(p, *trajectory) < 3.5) {
        double s = (*trajectory)[shortestPointIndex(p, *trajectory)].s;
        collision_st.emplace_back(t, s);
      }
    }
    if (!collision_st.empty()) {
      double s_start = collision_st.front().second;
      double s_end = collision_st.back().second;
      double t_start = collision_st.front().first;
      double t_end = collision_st.back().first;
      STBox box;
      box.ld = Point2d(t_start, s_start - obj.length / 2 - buffer);
      box.lu = Point2d(t_start, s_start + obj.length / 2);
      box.rd = Point2d(t_end, s_end - obj.length / 2 - buffer);
      box.ru = Point2d(t_end, s_end + obj.length / 2);
      st_box_list.push_back(box);
    }
  }
}

bool SpeedPlanner::SpeedPlanning(
    const std::vector<DynamicObj> &dynamic_obj_list, const double &car_speed,
    std::vector<Pose> *trajectory) {
  while (!step_pq.empty()) step_pq.pop();
  if (!trajectory || trajectory->empty()) {
    std::cout << "WARNING: No trajectory to speed planning!" << endl;
    return false;
  }
  // construct STBox
  construct_st_box(dynamic_obj_list, trajectory);
  // depth first search
  const auto &valid = [&](const Step &step) {
    for (const auto &pose : *trajectory) {
      if (step.s() - pose.s < 0.5 && step.s() - pose.s > -0.5) {
        if (step.v() > pose.v && step.a() > -3.6) return false;
        if (step.a() > pose.a) return false;
        break;
      }
    }
    for (const auto &st_box : st_box_list) {
      if (step.t() < st_box.ld.x || step.t() > st_box.rd.x) continue;
      double sin_theta =
          Segment(st_box.ld, st_box.rd)
              .cross(Segment(st_box.ld, Point2d(step.t(), step.s())));
      if (sin_theta >= 0) {
        return false;
      }
    }
    return true;
  };
  std::unordered_set<Step, StepHash> visited_steps;
  std::vector<Step> speed_result;
  std::function<bool(const Step &)> dfs = [&](const Step &step) {
    // std::cout << "search now:" << step << endl;
    if (step.t() >= 5) {
      Step back_step = step;
      while (back_step.has_father()) {
        // std::cout << "back step now:" << back_step << endl;
        const auto &steps = back_step.samples_from_father(0.02);
        speed_result.insert(speed_result.begin(), steps.begin(), steps.end());
        back_step = *back_step.father();
      }
      return true;
    }
    for (const auto &child : step.children()) {
      if (!valid(child) || visited_steps.find(child) != visited_steps.end())
        continue;
      visited_steps.insert(child);
      if (dfs(child)) return true;
      visited_steps.erase(child);
    }
    return false;
  };
  const auto start_pose = trajectory->front();
  Step initial_step(start_pose.s, start_pose.t, car_speed, start_pose.a);
  step_pq.push(initial_step);
  if (!dfs(initial_step)) return false;
  // std::cout << "speed_result size:" << speed_result.size() << endl;
  // for (const auto &step : speed_result) {
  //   std::cout << step << endl;
  // }
  // std::queue<Step> candidates;
  // candidates.push(initial_step);
  // while (!candidates.empty()) {
  //   const auto step_now = candidates.front();
  //   candidates.pop();
  //   for (const auto &child : initial_step.children()) {
  //     if (valid(child)) {
  //       step_pq.push(child);
  //       candidates.push(child);
  //     }
  //   }
  //   std::cout << "speed planning completed!" << std::endl;
  // }
  // const auto best_end_step = step_pq.top();
  Pose last_pose;
  for (auto &pose : *trajectory) {
    bool speed_seted = false;
    for (const auto &step : speed_result) {
      if (step.s() >= pose.s) {
        pose.t = step.t();
        pose.v = step.v();
        pose.a = step.a();
        speed_seted = true;
        break;
      }
    }
    if (!speed_seted) {
      if (last_pose.v < 0.25) {
        pose.v = 0;
        pose.t = 10000;
      } else {
        pose.t = last_pose.t + (pose.s - last_pose.s) / (last_pose.v + 1e-8);
        pose.v = last_pose.v;
      }
      pose.a = 0;
    }
    last_pose = pose;
  }
  return true;
}
}  // namespace TiEV