#ifndef __PATH_PLANNER__H__
#define __PATH_PLANNER__H__

#include "config.h"
#include "const.h"
#include "steering_functions/hc_cc_state_space/cc_dubins_state_space.hpp"
#include "steering_functions/hc_cc_state_space/hc_reeds_shepp_state_space.hpp"
#include "steering_functions/reeds_shepp_state_space/reeds_shepp_state_space.hpp"
#include "steering_functions/dubins_state_space/dubins_state_space.hpp"
#include "message_manager.h"
#include "pose.h"
#include "speed_optimizer.h"
#include <cmath>
#include <map>
#include <queue>
#include <thread>
#include <vector>
#include <tuple>
#include <functional>

using namespace std;

#define NO_TIME_LIMIT
// #define DEBUG_EXPANSION_CALLBACK
// #define DEBUG_ANALYTIC_EXPANSION_CALLBACK
#define USE_HC_PATH_ANALYTIC_EXPANSION
// #define USE_DUBINS_ANALYTIC_EXPANSION
#define USE_CLOTHOID_PRIMITIVES
// #define USE_ARC_PRIMITIVES

namespace TiEV {

constexpr int MAX_TARGET_NUM = 5;

static mutex path_planner_mtx;

class PathPlanner {
public:
    static PathPlanner* getInstance() {
        path_planner_mtx.lock();
        static PathPlanner inner_instance;
        path_planner_mtx.unlock();
        return &inner_instance;
    }

    void runPlanner(
        const DynamicObjList& dynamic_objs,
        double max_speed, bool reverse,
        double abs_safe_map[MAX_ROW][MAX_COL],
        double lane_safe_map[MAX_ROW][MAX_COL],
        const vector<Pose>& start_maintained_path,
        const vector<Pose>& targets,
        double current_speed,
        vector<SpeedPath>& results);

    void setAbsSafeMap(double map[MAX_ROW][MAX_COL]);

    void setLaneSafeMap(double map[MAX_ROW][MAX_COL]);

    void setStartMaintainedPath(const vector<Pose>& start_maintained_path);

    void setTargets(const vector<Pose>& targets);

    void setBackwardEnabled(bool enabled);

    void setCurrentSpeed(double speed);

    void setDynamicObjList(const DynamicObjList& dynamic_obj_list);

    void setVelocityLimit(double speed);

    void plan();

    bool getResults(vector<SpeedPath>& results);

    void getDistanceMaps(pair<double, double> (*xya_dis_map)[MAX_COL]) const;

#ifdef DEBUG_EXPANSION_CALLBACK
    typedef void(*node_expansion_callback)(
        double x, double y, double a, double k,
        bool is_backward, double heuristic, double cost);
#endif
#ifdef DEBUG_ANALYTIC_EXPANSION_CALLBACK
    typedef void(*analytic_expansion_callback)(
        const vector<pair<double, double>>& xys);
#endif

private:
    PathPlanner();
    ~PathPlanner(){};

    const Config*   config;
    MessageManager* view_controller;

    // is_planning
    bool is_planning = false;

    // search timer
    time_t plan_start_time;

    // planner settings
    bool         backward_enabled = false;
    Pose         start_point;
    vector<Pose> targets;

    // messages
    double         safe_map[MAX_ROW][MAX_COL];
    double         lane_safe_map[MAX_ROW][MAX_COL];
    double         abs_safe_map[MAX_ROW][MAX_COL];
    DynamicObjList dynamic_obj_list;
    double         current_speed  = 0;
    double         velocity_limit = 20;

    // visualization
    bool used_map[MAX_ROW][MAX_COL];

    // results
    SpeedPath    speed_paths[MAX_TARGET_NUM];
    vector<Pose> start_maintained_path;
    vector<pair<double, double>> speed_limits[MAX_TARGET_NUM];
    bool have_result[MAX_TARGET_NUM];

    //-----------------------------------------------------------
    // Threads
    //-----------------------------------------------------------
    void planner_thread(int target_index);

    //-----------------------------------------------------------
    // SpeedPlanner
    //-----------------------------------------------------------
    void planSpeed(int target_index);

    //-----------------------------------------------------------
    // AStar
    //-----------------------------------------------------------
    class primitive;
    /*
    class primitive_ptr {
        public:
            primitive_ptr() : vec(NULL), index(-1) {}
            primitive_ptr(vector<primitive>* vec, int index)
                : vec(vec), index(index) {}
            primitive* operator -> () const { return &(vec->operator[](index)); }
            primitive& operator * () const { return vec->operator[](index); }
            bool is_null() const { return vec == NULL; }
            primitive* base() const { return &(vec->operator[](index)); }

        private:
            vector<primitive>* vec;
            int index;
    };
    */
    typedef primitive* primitive_ptr;

    // astate is the inner state used by hybrid astar planner
    class astate {
    public:
        double x, y, a;
        double s;
        double curvature;
        bool is_backward;
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

        inline bool operator < (const node& n) const {
            return score > n.score;
        }
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
            double get_length() const { return length; }
            double get_maximum_curvature() const { return max_curvature; }
            double get_average_curvature() const { return average_curvature; }
            bool get_is_backward() const { return is_backward; }
            static constexpr double PRIMITIVE_SAMPLING_STEP = 0.2 / GRID_RESOLUTION;

        protected:
            static vector<astate> generate_line(
                double length,
                bool is_backward);
            static vector<astate> generate_arc(
                double curvature,
                double length,
                bool is_backward);
            static vector<astate> generate_clothoid(
                double begin_curvature,
                double end_curvature,
                double length,
                bool is_backward);

        private:
            vector<astate> sampled_states;
            double length;
            double max_curvature;
            double average_curvature;
            bool is_backward;
    };

    class arc_base_primitive : public base_primitive {
        public:
            arc_base_primitive(
                double curvature,
                double length,
                bool is_backward);
            double get_curvature() const { return curvature; }

        private:
            double curvature;
    };

    class line_base_primitive : public base_primitive {
        public:
            line_base_primitive(
                double length,
                bool is_backward);
    };

    class clothoid_base_primitive : public base_primitive {
        public:
            clothoid_base_primitive(
                double begin_curvature,
                double end_curvature,
                double length,
                bool is_backward);
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
            virtual const vector<const base_primitive*>& get_nexts(const astate& state) const = 0;
            virtual const vector<const base_primitive*>& get_nexts(const primitive& primitive) const = 0;
    };

    class arc_base_primitive_set : public base_primitive_set {
        public:
            arc_base_primitive_set();
            virtual const vector<const base_primitive*>& get_nexts(const astate& state) const;
            virtual const vector<const base_primitive*>& get_nexts(const primitive& primitive) const;
        private:
            vector<arc_base_primitive> primitives;
            vector<vector<const base_primitive*>> nexts;
    };

    class clothoid_base_primitive_set : public base_primitive_set {
        public:
            clothoid_base_primitive_set();
            void prepare(double current_speed_m_s, bool backward_enabled);
            virtual const vector<const base_primitive*>& get_nexts(const astate& state) const;
            virtual const vector<const base_primitive*>& get_nexts(const primitive& primitive) const;
        private:
            struct subset_for_speed {
                double current_speed_m_s;
                bool backward_enabled;
                vector<clothoid_base_primitive> primitives;
                vector<vector<const base_primitive*>> nexts;
            };

            vector<subset_for_speed> subsets;
            const subset_for_speed* current_subset = NULL;

            static void generate_clothoid_base_primitive_set(
                double max_curvature, double max_sigma,
                double clothoid_length, bool backward_enabled,
                vector<clothoid_base_primitive>& out_primitives,
                vector<vector<const base_primitive*>>& out_nexts);
    };

#if defined(USE_ARC_PRIMITIVES)
    arc_base_primitive_set arc_base_primitives;
#elif defined(USE_CLOTHOID_PRIMITIVES)
    clothoid_base_primitive_set clothoid_base_primitives;
#else
    #error define at least one flag to set which type of primitives to use
#endif

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
        primitive(const base_primitive* base,
            const primitive_ptr parent,
            const astate& start_state,
            double sin_start_state_a,
            double cos_start_state_a);
        const vector<astate>& get_states();
        astate                get_start_state() const;
        astate                get_end_state() const;
        double                get_length() const;
        const base_primitive* get_base() const;
        const primitive_ptr   get_parent() const;
        const bool            is_samped() const;

    private:
        const base_primitive* base;
        const primitive_ptr   parent;

        astate         start_state, end_state;
        vector<astate> sampled_states;
        bool           sampled;

        // translations
        double trans_sin;
        double trans_cos;

        void trans(astate& src) const;
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
        virtual bool traverse(const function<bool (const astate&)>& callback) const = 0;
        virtual ~analytic_expansion_provider() = default;
        static constexpr double ANALYTIC_EXPANSION_SAMPLING_STEP = 0.2 / GRID_RESOLUTION;
    };

    class dubins_provider : public analytic_expansion_provider {
    public:
        dubins_provider(
            const astate& _start_state,
            const astate& _end_state,
            double _max_curvature);
        virtual double get_length() const;
        virtual bool get_is_map_exceeded() const;
        virtual bool traverse(const function<bool (const astate&)>& callback) const;
        virtual ~dubins_provider() override = default;
    private:
        steer::State start_state;
        vector<Control> controls;
        Dubins_State_Space dubins_space;
    };

    class reeds_shepp_provider : public analytic_expansion_provider {
    public:
        reeds_shepp_provider(
            const astate& _start_state,
            const astate& _end_state,
            double _max_curvature);
        virtual double get_length() const;
        virtual bool get_is_map_exceeded() const;
        virtual bool traverse(const function<bool (const astate&)>& callback) const;
        virtual ~reeds_shepp_provider() override = default;
    private:
        steer::State start_state;
        vector<Control> controls;
        Reeds_Shepp_State_Space rs_space;
    };

    class cc_dubins_path_provider : public analytic_expansion_provider {
    public:
        cc_dubins_path_provider(
            const astate& _start_state,
            const astate& _end_state,
            double _max_curvature,
            double _max_sigma);
        virtual double get_length() const;
        virtual bool get_is_map_exceeded() const;
        virtual bool traverse(const function<bool (const astate&)>& callback) const;
        virtual ~cc_dubins_path_provider() override = default;
    private:
        steer::State start_state;
        vector<Control> controls;
        CC_Dubins_State_Space cc_dubins_space;
    };

    class hc_reeds_shepp_path_provider : public analytic_expansion_provider {
    public:
        hc_reeds_shepp_path_provider(
            const astate& _start_state,
            const astate& _end_state,
            double _max_curvature,
            double _max_sigma);
        virtual double get_length() const;
        virtual bool get_is_map_exceeded() const;
        virtual bool traverse(const function<bool (const astate&)>& callback) const;
        virtual ~hc_reeds_shepp_path_provider() override = default;
    private:
        steer::State start_state;
        vector<Control> controls;
        HC_Reeds_Shepp_State_Space hc_rs_space;
    };

    class local_planning_map {
        public:
            void prepare(const astate& target,
                double (*safe_map)[MAX_COL],
                bool is_backward_enabled);

            bool is_crashed(int x, int y) const;
            bool is_crashed(const astate& state) const;
            bool is_crashed(primitive& primitive) const;
            double get_maximum_safe_distance(int row_idx, int col_idx) const;
            double get_minimum_distance_from_map_boundaries(const astate& state) const;

            double get_heuristic(const astate& state, bool can_reverse) const;
            bool is_target(const astate& state) const;
            int try_get_target_index(primitive& primitive) const;
            bool is_in_map(const astate& state) const;
            bool is_in_map(int row_idx, int col_idx) const;

            void merge_xya_distance_map(pair<double, double> (*output_map)[MAX_COL]) const;

        private:
            astate target;
            double (*safe_map)[MAX_COL];
            bool backward_enabled;

            static constexpr int XYA_MAP_SHIFT_FACTOR = 2;
            // the xya maps are one cell larger than our local map,
            // so that we can mark the edge of the map unsafe,
            // then we don't need to worry about cells out of the
            // boundary, since we only do 1-step expansion when
            // calculating xya_distance_map.
            static constexpr int XYA_MAP_ROWS = ((MAX_ROW - 1) >> XYA_MAP_SHIFT_FACTOR) + 2;
            static constexpr int XYA_MAP_COLS = ((MAX_COL - 1) >> XYA_MAP_SHIFT_FACTOR) + 2;
            static constexpr int XYA_MAP_DEPTH = 32;
            static constexpr double XYA_MAP_DELTA_A = (M_PI * 2) / (XYA_MAP_DEPTH);
            static constexpr double BACKWARD_PUNISHMENT_FACTOR = 2.0;
            static constexpr double TURNING_PUNISHMENT_FACTOR = 1.0;
            double xya_distance_map[XYA_MAP_ROWS][XYA_MAP_COLS][XYA_MAP_DEPTH];
            bool xya_safe_map[XYA_MAP_ROWS][XYA_MAP_COLS];

            template<class T, int buffer_size> class ring_buffer {
                public:
                #define add(a) ((a + 1) % buffer_size)
                #define dec(a) ((a + (buffer_size - 1)) % buffer_size)
                    ring_buffer() : buffer(buffer_size) { }
                    void clear() { begin_idx = end_idx = 0; }
                    bool empty() const { return begin_idx == end_idx; }
                    int length() const { return (end_idx - begin_idx + buffer_size) % buffer_size; }
                    void push_front(const T& t) { buffer[begin_idx = dec(begin_idx)] = t; }
                    void push_back(const T& t) { buffer[end_idx] = t; end_idx = add(end_idx); }
                    void pop_front() { begin_idx = add(begin_idx); }
                    void pop_back() { end_idx = dec(end_idx); }
                    const T& front() const { return buffer[begin_idx]; }
                    const T& back() const { return buffer[dec(end_idx)]; }
                #undef add
                #undef dec
                private:
                    vector<T> buffer;
                    int begin_idx;
                    int end_idx;
            };

            static constexpr int XYA_BUFFER_SIZE = (int)pow(2, (int)ceil(
                std::log2(XYA_MAP_ROWS * XYA_MAP_COLS * XYA_MAP_DEPTH)));

            ring_buffer<int, XYA_BUFFER_SIZE> ring_buffer_xya;
            bool is_in_buffer_xya[XYA_MAP_ROWS * XYA_MAP_COLS * XYA_MAP_DEPTH];

            static int get_angle_index(double ang);
            void calculate_xya_distance_map();
    };

    class hybrid_astar_planner {
        public:
            void plan(const astate& start_state, const astate& target_state,
                double start_speed_m_s, bool is_backward_enabled,
                double (*safe_map)[MAX_COL], time_t max_duration,
                const base_primitive_set* base_primitives);
            bool get_have_result() const;
            const vector<astate>& get_result() const;

            void merge_xya_distance_map(pair<double, double> (*output_map)[MAX_COL]) const;

        private:
            int& history(const astate& state);
            bool is_time_out();
            bool try_analytic_expansion(
                const astate& state,
                bool backward_allowed,
                double max_curvature,
                double max_sigma);
            double get_cost_factor(
                const astate& prev_state,
                const astate& now_state,
                int history_visits) const;

            time_t start_time;
            time_t dead_line;
            long iterations;

            astate start_state, target_state;
            double start_speed_m_s;
            bool is_backward_enabled;

            static constexpr double BACKWARD_COST_FACTOR = 3.0;
            static constexpr double MIN_DISTANCE_BETWEEN_REVERSING = 3.0 / GRID_RESOLUTION;
            static constexpr double NODE_REVISIT_PUNISHMENT = 0.2;
            static constexpr double CURVATURE_PUNISHMENT_FACTOR = 25.0;
            static constexpr double CURVATURE_CHANGE_PUNISHMENT_FACTOR = 12.5;

            template<class T, int block_size> class block_mem_pool {
                public:
                    template<class... arg_types> T* make(const arg_types& ...args) {
                        if (tail_offset >= block_size) {
                            void* alloced = ::operator new(sizeof(T) * block_size);
                            mems.emplace_back(static_cast<T*>(alloced));
                            tail_offset = 0;
                        }
                        return new (mems.back() + (tail_offset ++)) T(args...);
                    }
                    void clear() {
                        while (!mems.empty()) {
                            for (int i = 0; i < tail_offset; ++i)
                                (mems.back() + i)->~T();
                            ::operator delete(static_cast<void*>(mems.back()));
                            mems.pop_back();
                            tail_offset = block_size;
                        }
                    }
                    int size() {
                        return (mems.size() * block_size) -
                            (block_size - tail_offset);
                    }
                    ~block_mem_pool() { clear(); }
                private:
                    vector<T*> mems;
                    int tail_offset = block_size;
            };

            local_planning_map planning_map;
            const base_primitive_set* base_primitives;
            block_mem_pool<primitive, 32768> primitive_pool;
            priority_queue<node, vector<node>> node_pool;
            vector<astate> analytic_expansion_result;

            static constexpr int HISTORY_MAP_SHIFT_FACTOR = 2;
            static constexpr int HISTORY_MAP_ROWS =
                ((MAX_ROW - 1) >> HISTORY_MAP_SHIFT_FACTOR) + 1;
            static constexpr int HISTORY_MAP_COLS =
                ((MAX_COL - 1) >> HISTORY_MAP_SHIFT_FACTOR) + 1;
            static constexpr int HISTORY_MAP_DEPTH = 16;
            static constexpr double HISTORY_MAP_DELTA_A =
                (M_PI * 2) / HISTORY_MAP_DEPTH;
            int node_history_map[MAX_ROW][MAX_COL][HISTORY_MAP_DEPTH];

            vector<astate> result;
            bool have_result;
    } hybrid_astar_planners[MAX_TARGET_NUM];

    // (k * curvature + b) * (v ^ 2) == 1
    // https://sm.ms/image/YUv7qXxGQKi9aH3
    static constexpr double KV_LINE_K = 0.9808;
    static constexpr double KV_LINE_B = -0.0031;
    static constexpr double CAR_MAX_CURVATURE = 0.181818182; // 1/m
    static constexpr double SPEED_DESCENT_FACTOR = 1.0; // m/s^2

    static inline double max_curvature_under_velocity(double velocity_m_s) {
        double curvature = (1.0 / (velocity_m_s * velocity_m_s) -
            KV_LINE_B) / KV_LINE_K;
        return max(min(curvature, CAR_MAX_CURVATURE), 0.0);
    }

    static inline double max_velocity_for_curvature(double curvature_1_m) {
        return sqrt(1.0 / max(1e-8, KV_LINE_K *
            max(curvature_1_m, 0.0) + KV_LINE_B));
    }

    // we assume the steering wheel can rotate PI every three seconds
    // at its maximum angular speed.
    static constexpr double MAX_STEERING_WHEEL_ROTATE_SPEED = M_PI / 3.0; // ang/s
    static constexpr double MAX_STEERING_WHEEL_ANGLE = M_PI + M_PI_2; // ang

    static inline double max_sigma_under_velocity(double velocity_m_s) {
        return CAR_MAX_CURVATURE / (max(fabs(velocity_m_s), 5.0 / 3.6) *
            (MAX_STEERING_WHEEL_ANGLE / MAX_STEERING_WHEEL_ROTATE_SPEED));
    }

    static inline double wrap_angle_0_2_PI(double a) {
        a = fmod(a, 2 * M_PI);
        return a < 0 ? a + 2 * M_PI : a;
    }

public:
    /* Safe map is an integer map
     * output[i][j] == 0 => point (i,j) is safe
     * output[i][j] > 0 => point (i,j) is too close to some obstacles
     */
    static void calculateSafeMap(u_char map[MAX_ROW][MAX_COL], int output[MAX_ROW][MAX_COL], int safe_distance, queue<pair<int, int>>& using_queue) {
        const int dx[] = { 0, 0, -1, 1 };
        const int dy[] = { -1, 1, 0, 0 };

        memset(output, 0, sizeof(int) * MAX_ROW * MAX_COL);

        if(safe_distance <= 0) return;

        while(!using_queue.empty())
            using_queue.pop();
        for(int i = 0; i < MAX_ROW; ++i)
            for(int j = 0; j < MAX_COL; ++j)
                if(map[i][j]) {
                    output[i][j] = safe_distance;
                    using_queue.push(make_pair(i, j));
                }

        while(!using_queue.empty()) {
            const int x = using_queue.front().first;
            const int y = using_queue.front().second;
            using_queue.pop();
            for(int i = 0; i < 4; ++i) {
                const int tx = x + dx[i], ty = y + dy[i];
                if(tx >= 0 && tx < MAX_ROW && ty >= 0 && ty < MAX_COL && output[tx][ty] < output[x][y] - 1) {
                    output[tx][ty] = output[x][y] - 1;
                    using_queue.push(make_pair(tx, ty));
                }
            }
        }
    }

    /* It is what it seems to be */
    static inline double euclideanDistance(const double& x_0, const double& y_0, const double& x_1, const double& y_1) {
        const double dx = x_0 - x_1;
        const double dy = y_0 - y_1;
        return sqrt(dx * dx + dy * dy);
    }

    static inline double curvatureConstraint(const double& speed_2_ms, const double& steering_ability) {
        return steering_ability * GRAVITY / speed_2_ms;
    }
};
}

#endif  //!__PATH_PLANNER__H__