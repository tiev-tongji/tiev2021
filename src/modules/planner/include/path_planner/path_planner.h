#ifndef __PATH_PLANNER__H__
#define __PATH_PLANNER__H__

#include "config.h"
#include "const.h"
#include "look_up_tables/distance_table.h"
#include "message_manager.h"
#include "pose.h"
#include "speed_optimizer.h"
#include <cmath>
#include <map>
#include <queue>
#include <thread>
#include <vector>
#include <tuple>

using namespace std;

// #define NO_ANALYTIC_EXPANSION
#define NO_TIME_LIMIT

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

    void getCostMaps(int (*output_map)[MAX_COL]) const;

    void getDistanceMaps(pair<double, double> (*xya_dis_map)[MAX_COL]) const;

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

    class astate {
    public:
        double x, y, a;
        double s;
        double curvature;
        bool is_backward;
    };

    class node {
    public:
        double score;
        double minimum_speed;
        double cost;
        double dis_after_reverse;

        primitive_ptr ptr;

        bool operator < (const node& n) const {
            return score > n.score;
        }
    };

    class base_primitive {
        public:
            base_primitive(const vector<astate>& sampled_states);
            const vector<astate>& get_states() const;
            double get_length() const;
            double get_maximum_curvature() const;

        private:
            vector<astate> sampled_states;
            double max_curvature;
    };

    class arc_base_primitive : public base_primitive {
        public:
            arc_base_primitive(
                double curvature,
                bool is_backward,
                double length,
                double sampling_step);

        private:
            bool curvature;
            bool is_backward;
            double sampling_step;

            static vector<astate> generate_arc(
                double curvature,
                bool is_backward,
                double length,
                double sampling_step);
    };

    class line_base_primitive : public base_primitive {
        public:
            line_base_primitive(
                double length,
                double sampling_step,
                bool is_backward);
        private:
            double length;
            double sampling_step;
            bool is_backward;

            static vector<astate> generate_line(
                double length,
                double sampling_step,
                bool is_backward);
    };

    class clothoid_base_primitive : public base_primitive {
        public:
            clothoid_base_primitive(
                double begin_curvature,
                double end_curvature,
                double length,
                double sampling_step,
                bool is_backward);
        private:
            double begin_curvature;
            double end_curvature;
            double sampling_step;
            bool is_backward;

            static vector<astate> generate_clothoid(
                double begin_curvature,
                double end_curvature,
                double length,
                double sampling_step,
                bool is_backward);
    };

    class base_primitive_set {
        public:
            virtual const vector<const base_primitive*>& get_nexts(const astate& state) const = 0;
            virtual const vector<const base_primitive*>& get_nexts(const primitive& primitive) const = 0;
            virtual double get_sampling_step_size() const = 0;
    };

    class arc_base_primitive_set : public base_primitive_set {
        public:
            arc_base_primitive_set();
            virtual const vector<const base_primitive*>& get_nexts(const astate& state) const;
            virtual const vector<const base_primitive*>& get_nexts(const primitive& primitive) const;
            virtual double get_sampling_step_size() const;
        private:
            vector<base_primitive> primitives;
            vector<vector<const base_primitive*>> nexts;
    }arc_base_primitives;

    class primitive {
    public:
        primitive(const base_primitive* base,
            const primitive_ptr parent,
            const astate& start_state);
        primitive(primitive&& primitive);
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

    class local_planning_map {
        public:
            void init(const astate& target,
                double (*safe_map)[MAX_COL],
                bool is_backward_enabled,
                double backward_cost_factor);

            bool is_crashed(int x, int y) const;
            bool is_crashed(const astate& state) const;
            bool is_crashed(primitive& primitive) const;

            double get_heuristic(const astate& state, bool can_reverse) const;
            bool is_target(const astate& state) const;
            int try_get_target_index(primitive& primitive) const;
            bool is_in_map(const astate& state) const;

            void merge_xya_distance_map(pair<double, double> (*output_map)[MAX_COL]) const;

        private:
            astate target;
            double (*safe_map)[MAX_COL];
            bool backward_enabled;
            double backward_cost_factor;

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
            double xya_distance_map[XYA_MAP_ROWS][XYA_MAP_COLS][XYA_MAP_DEPTH];
            bool xya_safe_map[XYA_MAP_ROWS][XYA_MAP_COLS];

            template<class T, int buffer_size>
            class ring_buffer {
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
            double get_maximum_safe_distance(const astate& state) const;
    };

    class hybrid_astar_planner {
        public:
            hybrid_astar_planner(
                const base_primitive_set*
                base_primitives
            );
            void plan(
                const astate& start_state,
                const astate& target_state,
                double start_speed_m_s,
                bool is_backward_enabled,
                double (*safe_map)[MAX_COL],
                time_t max_duration);
            bool get_have_result() const;
            const vector<astate>& get_result() const;

            void merge_history_map(int (*output_map)[MAX_COL]) const;
            void merge_xya_distance_map(pair<double, double> (*output_map)[MAX_COL]) const;

        private:
            int& history(const astate& state);
            bool is_time_out();
            bool try_analytic_expansion(const astate& from_state, double heuristic);
            static int get_angle_index(double ang);

            time_t start_time;
            time_t dead_line;
            long iterations;

            astate start_state, target_state;
            double start_speed_m_s;
            bool is_backward_enabled;

            static constexpr double SPEED_DESCENT_FACTOR = 1.0;
            static constexpr double BACKWARD_COST_FACTOR = 2.0;
            static constexpr double MIN_DISTANCE_BETWEEN_REVERSING = 5.0;

            local_planning_map planning_map;
            const base_primitive_set* base_primitives;
            vector<primitive> primitive_pool;
            priority_queue<node, vector<node>> node_pool;
            vector<astate> analytic_expansion_result;

            static constexpr int HISTORY_MAP_SHIFT_FACTOR = 2;
            static constexpr int HISTORY_MAP_ROWS =
                ((MAX_ROW - 1) >> HISTORY_MAP_SHIFT_FACTOR) + 1;
            static constexpr int HISTORY_MAP_COLS =
                ((MAX_COL - 1) >> HISTORY_MAP_SHIFT_FACTOR) + 1;
            static constexpr int HISTORY_MAP_DEPTH = 8;
            static constexpr double HISTORY_MAP_DELTA_A =
                (M_PI * 2) / HISTORY_MAP_DEPTH;
            int node_history_map[MAX_ROW][MAX_COL][HISTORY_MAP_DEPTH];

            vector<astate> result;
            bool have_result;
    // TODO
    } hybrid_astar_planners[MAX_TARGET_NUM] = {
        &arc_base_primitives,
        &arc_base_primitives,
        &arc_base_primitives,
        &arc_base_primitives,
        &arc_base_primitives
    };

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