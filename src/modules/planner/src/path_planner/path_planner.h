#ifndef __PATH_PLANNER__H__
#define __PATH_PLANNER__H__

#include <cmath>
#include <functional>
#include <map>
#include <queue>
#include <thread>
#include <tuple>
#include <vector>

#include "config.h"
#include "const.h"
#include "distance_table.h"
#include "message_manager.h"
#include "pose.h"
#include "speed_optimizer.h"
#include "steering_functions/dubins_state_space/dubins_state_space.hpp"
#include "steering_functions/hc_cc_state_space/cc_dubins_state_space.hpp"
#include "steering_functions/hc_cc_state_space/hc_reeds_shepp_state_space.hpp"
#include "steering_functions/reeds_shepp_state_space/reeds_shepp_state_space.hpp"

using namespace std;

// #define NO_TIME_LIMIT
// #define DEBUG_EXPANSION_CALLBACK
// #define DEBUG_ANALYTIC_EXPANSION_CALLBACK
#define USE_HC_PATH_ANALYTIC_EXPANSION
// #define USE_DUBINS_ANALYTIC_EXPANSION
#define USE_CLOTHOID_PRIMITIVES

namespace TiEV {

constexpr int MAX_TARGET_NUM = 5;

static mutex path_planner_mtx;

// the length of primitive
const static double primi_l = 1.0;  // m

const static std::vector<double> k_list      = {-CAR_MAX_K,
                                           -3.0 / 4.0 * CAR_MAX_K,
                                           -1.0 / 2.0 * CAR_MAX_K,
                                           -1.0 / 4.0 * CAR_MAX_K,
                                           0.0,
                                           1.0 / 4.0 * CAR_MAX_K,
                                           1.0 / 2.0 * CAR_MAX_K,
                                           3.0 / 4.0 * CAR_MAX_K,
                                           CAR_MAX_K};
const static std::vector<double> k_step_list = {
    0.0, -1.0 / 4.0 * CAR_MAX_K, 1.0 / 4.0 * CAR_MAX_K, 1.0 / 2.0 * CAR_MAX_K,
    -1.0 / 2.0 * CAR_MAX_K};

class PathPlanner {
 public:
  static PathPlanner* getInstance() {
    static PathPlanner inner_instance;
    return &inner_instance;
  }

  void runPathPlanner(const NavInfo&                 nav_info,
                      const std::vector<HDMapPoint>& ref_path,
                      const DynamicObjList&          dynamic_objs,
                      const double max_speed, const bool reverse,
                      const double        abs_safe_map[MAX_ROW][MAX_COL],
                      const double        lane_safe_map[MAX_ROW][MAX_COL],
                      const vector<Pose>& start_maintained_path,
                      const Pose& target, std::vector<Pose>* result_path);

  void setReferencePath(const std::vector<HDMapPoint>& ref_path);

  void setAbsSafeMap(const double map[MAX_ROW][MAX_COL]);

  void setLaneSafeMap(const double map[MAX_ROW][MAX_COL]);

  void setStartMaintainedPath(const vector<Pose>& start_maintained_path);

  void setTarget(const Pose& target);

  void setBackwardEnabled(bool enabled);

  void setNavInfo(const NavInfo& nav_info);

  void setDynamicObjList(const DynamicObjList& dynamic_obj_list);

  void setVelocityLimit(const double speed);

  void plan(std::vector<Pose>* result);

 private:
  PathPlanner();
  ~PathPlanner(){};

  // lookup distance table for rs and dubins curve
  const DistanceTable* distance_table_rs;
  const DistanceTable* distance_table_dubins;
  const Config*        config;
  MessageManager*      view_controller;

  // is_planning
  bool is_planning = false;

  // search timer
  time_t plan_start_time;

  // planner settings
  Pose    start_pose;
  Pose    target_pose;
  bool    backward_enabled;
  double  lane_safe_map[MAX_ROW][MAX_COL];
  double  abs_safe_map[MAX_ROW][MAX_COL];
  double  velocity_limit = 20;
  NavInfo nav_info;

  DynamicObjList          dynamic_obj_list;
  std::vector<Pose>       start_maintained_path;
  std::vector<HDMapPoint> ref_path;

  // results
  std::vector<Pose> trajectory;

  template <class T, int block_size>
  class block_mem_pool {
   public:
    template <class... arg_types>
    T* make(const arg_types&... args) {
      if (tail_offset >= block_size) {
        void* alloced = ::operator new(sizeof(T) * block_size);
        mems.emplace_back(static_cast<T*>(alloced));
        tail_offset = 0;
      }
      return new (mems.back() + (tail_offset++)) T(args...);
    }
    void clear() {
      while (!mems.empty()) {
        for (int i = 0; i < tail_offset; ++i) (mems.back() + i)->~T();
        ::operator delete(static_cast<void*>(mems.back()));
        mems.pop_back();
        tail_offset = block_size;
      }
    }
    int size() {
      return (mems.size() * block_size) - (block_size - tail_offset);
    }
    ~block_mem_pool() { clear(); }

   private:
    vector<T*> mems;
    int        tail_offset = block_size;
  };
  //-----------------------------------------------------------
  // Local path planning definition
  //-----------------------------------------------------------
  class primitive;
  typedef primitive* primitive_ptr;

  // astate is the inner state used by hybrid astar planner
  class astate {
   public:
    astate() : x(0), y(0), a(0), s(0), curvature(0), is_backward(false){};
    astate(double x_, double y_, double a_, double s_, double k_,
           bool is_backward_)
        : x(x_),
          y(y_),
          a(a_),
          s(s_),
          curvature(k_),
          is_backward(is_backward_){};
    double               x;
    double               y;
    double               a;
    double               s;
    double               curvature;
    bool                 is_backward;
    friend std::ostream& operator<<(std::ostream& out, const astate& state) {
      out << "astate:{x=" << state.x << " y=" << state.y << " a=" << state.a
          << " s=" << state.s << " k=" << state.curvature
          << " b=" << state.is_backward << "}";
      return out;
    }
  };

  // node of the hybrid astar search contains properties of a node like
  // heuristic and cost, and a pointer to the primitive generated
  // in the last expansion
  class node {
   public:
    // score = heuristic + cost
    double score;
    // the minimum possible speed at this position
    double minimum_speed;
    double cost;
    // the distance covered since last reversing
    // (backward -> forward or forward -> backward)
    double dis_after_reverse;
    // points to the last expanded primitive
    primitive_ptr ptr;

    inline bool operator<(const node& n) const { return score > n.score; }
  };

  // base_primitive contains pre-computed properties and
  // pre-sampled states of hybrid astar primitive.
  // base_primitive always starts from state (0.0, 0.0, 0.0)
  // and will be transformed to perform an expansion at
  // searching time.
  class base_primitive {
   public:
    base_primitive(const vector<astate>& sampled_states);
    const vector<astate>& get_states() const { return sampled_states; }
    double                get_length() const { return length; }
    double get_maximum_curvature() const { return max_curvature; }
    double get_average_curvature() const { return average_curvature; }
    bool   get_is_backward() const { return is_backward; }
    static constexpr double PRIMITIVE_SAMPLING_STEP = GRID_RESOLUTION;

   protected:
    static vector<astate> generate_line(double length, bool is_backward);
    static vector<astate> generate_arc(double curvature, double length,
                                       bool is_backward);
    static vector<astate> generate_clothoid(double begin_curvature,
                                            double k_step, bool is_backward);

   private:
    vector<astate> sampled_states;
    double         length;
    double         max_curvature;
    double         average_curvature;
    bool           is_backward;
  };

  class arc_base_primitive : public base_primitive {
   public:
    arc_base_primitive(double curvature, double length, bool is_backward);
    double get_curvature() const { return curvature; }

   private:
    double curvature;
  };

  class line_base_primitive : public base_primitive {
   public:
    line_base_primitive(double length, bool is_backward);
  };

  class clothoid_base_primitive : public base_primitive {
   public:
    clothoid_base_primitive(double begin_curvature, double k_step,
                            double length, bool is_backward);
    double get_begin_curvature() const { return begin_curvature; }
    double get_end_curvature() const { return end_curvature; }

   private:
    double begin_curvature;
    double end_curvature;
  };

  // base_primitive_set is a set of primitives which initializes all
  // appropriate base primitives and the linkage relationship among them
  // (linkage relationship means base_primitive_set calculates all
  //  possible base primitive successors for each base primitive)
  class base_primitive_set {
   public:
    virtual const vector<base_primitive> get_nexts(
        const astate& state, const double current_speed) const = 0;
    virtual const vector<base_primitive> get_nexts(
        const primitive& primitive) const = 0;
    virtual void prepare(bool backward_enabled){};
  };

  class clothoid_base_primitive_set : public base_primitive_set {
   public:
    clothoid_base_primitive_set();
    virtual const vector<base_primitive> get_nexts(
        const astate& state, const double current_speed) const;
    virtual const vector<base_primitive> get_nexts(
        const primitive& primitive) const;
    virtual void prepare(bool backward_enabled);

   private:
    struct primitives_for_k {
      double                          current_k;
      bool                            backward_enabled;
      vector<clothoid_base_primitive> primitives;
    };

    vector<primitives_for_k>        all_k_sets;
    vector<const primitives_for_k*> current_subset;

    static void generate_clothoid_base_primitive_set(
        const double begin_k, bool backward_enabled,
        vector<clothoid_base_primitive>& out_primitives);
  };

  clothoid_base_primitive_set clothoid_base_primitives;

  // primitive contains properties of a transformed base primitive
  // each expansion generates a primitive object at search time.
  class primitive {
   public:
    // construct a primitive transformed from base, started from start_state
    // and pointed to its parent.
    // for saving computation and memory consumption, not all of the sampled
    // states of the base primitive is transformed at constructure time,
    // only start and end states are transformed at the beginning.
    // other states are automatically transformed at the first time
    // get_states() is called. only call get_states() when u need
    // exactly each internal state sampled and transformed.
    primitive(const base_primitive& base, const primitive_ptr parent);
    const vector<astate>& get_states();
    astate                get_start_state() const;
    astate                get_end_state() const;
    double                get_length() const;
    const base_primitive& get_base() const;
    const primitive_ptr   get_parent() const;

   private:
    const base_primitive base;
    const primitive_ptr  parent;
  };

  // analytic_expansion_provider provides analytic_expansion
  class analytic_expansion_provider {
   public:
    virtual double get_length() const = 0;
    // get_is_map_exceeded() should be a fast verification
    // of whether the generated curve exceeds the local map.
    // it is not necessary for this to exactly detect each
    // exceeding curves, but it has to be as quick as possible.
    virtual bool get_is_map_exceeded() const = 0;
    // traverse() samples the generated curve and calls callback
    // for each sampled state, the callback should return whether
    // the traversing is supposed to continue.
    // this method returns whether the traversing is completed.
    virtual bool traverse(
        const function<bool(const astate&)>& callback) const = 0;
    virtual ~analytic_expansion_provider()                   = default;
    static constexpr double ANALYTIC_EXPANSION_SAMPLING_STEP =
        0.2 / GRID_RESOLUTION;
  };

  class dubins_provider : public analytic_expansion_provider {
   public:
    dubins_provider(const astate& _start_state, const astate& _end_state,
                    double _max_curvature);
    virtual double get_length() const;
    virtual bool   get_is_map_exceeded() const;
    virtual bool traverse(const function<bool(const astate&)>& callback) const;
    virtual ~dubins_provider() override = default;

   private:
    steer::State       start_state;
    vector<Control>    controls;
    Dubins_State_Space dubins_space;
  };

  class reeds_shepp_provider : public analytic_expansion_provider {
   public:
    reeds_shepp_provider(const astate& _start_state, const astate& _end_state,
                         double _max_curvature);
    virtual double get_length() const;
    virtual bool   get_is_map_exceeded() const;
    virtual bool traverse(const function<bool(const astate&)>& callback) const;
    virtual ~reeds_shepp_provider() override = default;

   private:
    steer::State            start_state;
    vector<Control>         controls;
    Reeds_Shepp_State_Space rs_space;
  };

  class cc_dubins_path_provider : public analytic_expansion_provider {
   public:
    cc_dubins_path_provider(const astate& _start_state,
                            const astate& _end_state, double _max_curvature,
                            double _max_sigma);
    virtual double get_length() const;
    virtual bool   get_is_map_exceeded() const;
    virtual bool traverse(const function<bool(const astate&)>& callback) const;
    virtual ~cc_dubins_path_provider() override = default;

   private:
    steer::State          start_state;
    vector<Control>       controls;
    CC_Dubins_State_Space cc_dubins_space;
  };

  class hc_reeds_shepp_path_provider : public analytic_expansion_provider {
   public:
    hc_reeds_shepp_path_provider(const astate& _start_state,
                                 const astate& _end_state,
                                 double _max_curvature, double _max_sigma);
    virtual double get_length() const;
    virtual bool   get_is_map_exceeded() const;
    virtual bool traverse(const function<bool(const astate&)>& callback) const;
    virtual ~hc_reeds_shepp_path_provider() override = default;

   private:
    steer::State               start_state;
    vector<Control>            controls;
    HC_Reeds_Shepp_State_Space hc_rs_space;
  };

  class local_planning_map {
   public:
    void prepare(const astate& target, double (*abs_safe_map)[MAX_COL],
                 double (*lane_safe_map)[MAX_COL], bool is_backward_enabled);
    void prepare(const DynamicObjList&          dynamic_obj_list,
                 const std::vector<HDMapPoint>& ref_path,
                 double (*abs_safe_map)[MAX_COL],
                 double (*lane_safe_map)[MAX_COL], bool is_backward_enabled);

    bool   is_abs_crashed(int x, int y) const;
    bool   is_abs_crashed(const astate& state) const;
    bool   is_abs_crashed(primitive& primitive) const;
    bool   is_lane_crashed(int x, int y) const;
    bool   is_lane_crashed(const astate& state) const;
    bool   is_lane_crashed(primitive& primitive) const;
    double get_maximum_safe_distance(int row_idx, int col_idx) const;
    double get_minimum_distance_from_map_boundaries(const astate& state) const;

    double get_heuristic(const astate& state, bool can_reverse) const;
    bool   is_target(const astate& state) const;
    int    try_get_target_index(primitive& primitive) const;
    bool   is_in_map(const astate& state) const;
    bool   is_in_map(int row_idx, int col_idx) const;

   private:
    std::vector<HDMapPoint> ref_path;

    Pose   left_bound_p, right_bound_p;
    astate target;

    double (*abs_safe_map)[MAX_COL];
    double (*lane_safe_map)[MAX_COL];
    double astar_2d_distance_map[MAX_ROW][MAX_COL];

    bool backward_enabled;
    bool is_planning_to_target;

    static constexpr double BACKWARD_PUNISHMENT_FACTOR = 2.0;
    static constexpr double TURNING_PUNISHMENT_FACTOR  = 1.0;

    void calculate_2d_distance_map();
  };

  //--------------------------------------------
  //      TiEV Path Planning Algrithom
  //--------------------------------------------
  class TiEVPlanner {
   public:
    const std::vector<astate>& plan(
        const DynamicObjList&          dynamic_obj_list,
        const std::vector<HDMapPoint>& ref_path, const astate& start_state,
        double start_speed_m_s, bool is_backward_enabled,
        double (*abs_safe_map)[MAX_COL], double (*lane_safe_map)[MAX_COL],
        time_t max_duration, const base_primitive_set* base_primitives);

   private:
    void   visit(const astate& state);
    bool   is_visited(const astate& state);
    bool   is_time_out();
    double get_cost_factor(const astate& prev_state,
                           const astate& now_state) const;

    time_t start_time;
    time_t dead_line;
    long   iterations;

    astate start_state;
    double start_speed_m_s;
    bool   is_backward_enabled;

    bool node_visited_map[2 * MAX_ROW][2 * MAX_COL][ANGLE_NUM];

    static constexpr double BACKWARD_COST_FACTOR = 5.0;

    local_planning_map                 planning_map;
    const base_primitive_set*          base_primitives;
    block_mem_pool<primitive, 32768>   primitive_pool;
    priority_queue<node, vector<node>> node_pool;
    vector<astate>                     analytic_expansion_result;

    vector<astate> result;
  } tiev_planner;

  //---------------------------------------
  // Hybrid Astar Path Planning Algrithom
  //---------------------------------------
  class AstarPlanner {
   public:
    const std::vector<astate>& plan(
        const astate& start_state, const astate& target_state,
        double start_speed_m_s, bool is_backward_enabled,
        double (*abs_safe_map)[MAX_COL], double (*lane_safe_map)[MAX_COL],
        time_t max_duration, const base_primitive_set* base_primitives);

   private:
    void   visit(const astate& state);
    bool   is_visited(const astate& state);
    bool   is_time_out();
    bool   try_analytic_expansion(const astate& state, bool backward_allowed,
                                  double max_curvature, double max_sigma);
    double get_cost_factor(const astate& prev_state,
                           const astate& now_state) const;

    time_t start_time;
    time_t dead_line;
    long   iterations;

    astate start_state;
    astate target_state;
    double start_speed_m_s;
    bool   is_backward_enabled;

    bool node_visited_map[2 * MAX_ROW][2 * MAX_COL][ANGLE_NUM];

    static constexpr double BACKWARD_COST_FACTOR = 2.0;

    local_planning_map                 planning_map;
    const base_primitive_set*          base_primitives;
    block_mem_pool<primitive, 32768>   primitive_pool;
    priority_queue<node, vector<node>> node_pool;
    vector<astate>                     analytic_expansion_result;

    vector<astate> result;
  } astar_planner;

  static constexpr double SPEED_DESCENT_FACTOR = 1.0;  // m/s^2

  static inline double wrap_angle_0_2_PI(double a) {
    a = fmod(a, 2 * M_PI);
    return a < 0 ? a + 2 * M_PI : a;
  }

  static inline double wrap_angle_in_PI(double a) {
    a = wrap_angle_0_2_PI(a);
    if (a > M_PI) a -= 2 * M_PI;
    if (a <= -M_PI) a += 2 * M_PI;
    return a;
  }

 public:
  /* It is what it seems to be */
  static inline double sqrDistance(const double& x_0, const double& y_0,
                                   const double& x_1, const double& y_1) {
    const double dx = x_0 - x_1;
    const double dy = y_0 - y_1;
    return dx * dx + dy * dy;
  }
  static inline double euclideanDistance(const double& x_0, const double& y_0,
                                         const double& x_1, const double& y_1) {
    const double dx = x_0 - x_1;
    const double dy = y_0 - y_1;
    return sqrt(dx * dx + dy * dy);
  }
};
}  // namespace TiEV

#endif  //!__PATH_PLANNER__H__