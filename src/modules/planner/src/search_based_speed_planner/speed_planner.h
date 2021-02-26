#ifndef SPEED_PLANNER_H
#define SPEED_PLANNER_H

#include <memory>
#include <queue>
#include <vector>
#include "point2d.h"
#include "tiev_class.h"

namespace TiEV {

const std::vector<double> acceleration = {4,  3.5,  3,  2.5,  2,  1.5,
                                          1,  0.5,  0,  -0.5, -1, -1.5,
                                          -2, -2.5, -3, -3.5, -4};  // m/s^2
constexpr double kT_interval = 0.5;                                 // second

struct STBox {
  Point2d ld, lu, rd, ru;
  friend std::ostream &operator<<(std::ostream &out, const STBox &box) {
    out << "STBox:{left down=" << box.ld << " right down=" << box.rd
        << " left up=" << box.lu << " right up=" << box.ru << "}";
    return out;
  }
};

class Step {
 public:
  Step() : father_(nullptr) {}
  Step(const double &s, const double &t, const double &v, const double &a)
      : s_(s), t_(t), v_(v), a_(a), father_(nullptr) {
    value_ = s_ + 10 * t_;
  };
  Step(const Step &step) : Step(step.s(), step.t(), step.v(), step.a()) {
    if (step.has_father()) father_ = step.father();
  };

  const std::vector<Step> children() const;
  const std::vector<Step> samples_from_father(const double t_interval) const;

  inline const double s() const { return s_; }
  inline const double t() const { return t_; }
  inline const double v() const { return v_; }
  inline const double a() const { return a_; }
  const std::shared_ptr<Step> father() const { return father_; }
  bool has_father() const { return father_ != nullptr; }
  void set_father(const Step &father) {
    father_ = std::make_shared<Step>(father);
  }
  inline const double value() const { return value_; }

  void operator=(const Step &other) {
    s_ = other.s();
    t_ = other.t();
    v_ = other.v();
    a_ = other.a();
    father_ = other.father();
  }
  bool operator<(const Step &other) const { return value_ < other.value(); }
  bool operator==(const Step &other) const {
    return (s_ == other.s() && t_ == other.t() && v_ == other.v() &&
            a_ == other.a());
  }

  friend std::ostream &operator<<(std::ostream &out, const Step &step) {
    out << "Step:{s=" << step.s() << " \tt=" << step.t() << " \tv=" << step.v()
        << " \ta=" << step.a() << "}";
    return out;
  }

 private:
  double s_;
  double t_;
  double v_;
  double a_;
  std::shared_ptr<Step> father_ = nullptr;
  double value_;
};

class StepHash {
 public:
  size_t operator()(const Step &step) const {
    return step.s() * step.t() + step.v() * step.a() * step.a() + step.s() +
           step.t() + step.v() + step.a() * step.a();
  }
};

class SpeedPlanner {
 public:
  SpeedPlanner() = default;
  ~SpeedPlanner() = default;

  bool SpeedPlanning(const std::vector<DynamicObj> &dynamic_obj_list,
                     const double &car_speed, std::vector<Pose> *trajectory);

 private:
  void Init();
  void construct_st_box(const std::vector<DynamicObj> &dynamic_obj_list,
                        const std::vector<Pose> *trajectory);

  std::priority_queue<Step> step_pq;
  std::vector<STBox> st_box_list;

  const double buffer = 2;
};
}  // namespace TiEV

#endif