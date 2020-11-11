#ifndef __PATH_PLANNER__H__
#define __PATH_PLANNER__H__

#include "config.h"
#include "const.h"
#include "look_up_tables/distance_table.h"
#include "message_manager.h"
#include "pose.h"
#include "speed_optimizer.h"
#include <map>
#include <queue>
#include <thread>
#include <vector>

using namespace std;

namespace TiEV {

//#define COUT_DEBUG_INFO
//#define NO_SPEED_PLANNER

const int    MAX_TARGET_NUM = 5;
static mutex path_planner_mtx;

const double speeds[] = { 5 / 3.6, 10 / 3.6, 20 / 3.6, 30 / 3.6, 40 / 3.6, 50 / 3.6, 60 / 3.6, 70 / 3.6 };

const double curvatures[] = {
    0.2, 0.13437, 0.03939, 0.01922, 0.01155, 0.00778, 0.0056, 0.00429,
};

const double arc_lengths[] = { 1.0, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0 };

const int speed_cnt = sizeof(speeds) / sizeof(double);

class PathPlanner {

public:
    static PathPlanner* getInstance() {
        path_planner_mtx.lock();
        static PathPlanner inner_instance;
        path_planner_mtx.unlock();
        return &inner_instance;
    }

    void runPlanner(const DynamicObjList& dynamic_objs, double max_speed, bool reverse, double abs_safe_map[MAX_ROW][MAX_COL], double lane_safe_map[MAX_ROW][MAX_COL],
                    const vector<Pose>& start_maintained_path, const vector<Pose>& targets, double current_speed, vector<SpeedPath>& results);

    void setAbsSafeMap(double map[MAX_ROW][MAX_COL]);

    void setLaneSafeMap(double map[MAX_ROW][MAX_COL]);

    void setSafeMap(double map[MAX_ROW][MAX_COL]);
    void setLaneMap(double map[MAX_ROW][MAX_COL]);
    void setAbsMap(double map[MAX_ROW][MAX_COL]);

    void setStartMaintainedPath(const vector<Pose>& start_maintained_path);

    void setTargets(const vector<Pose>& targets);

    void setBackwardEnabled(bool enabled);

    void setCurrentSpeed(double speed);

    void setDynamicObjList(const DynamicObjList& dynamic_obj_list);

    void setVelocityLimit(double speed);

    void plan();

    bool getResults(vector<SpeedPath>& results);

private:
    PathPlanner();
    ~PathPlanner(){};

    const Config*   config;
    MessageManager* view_controller;

    // is_planning
    bool is_planning = false;

    // search timer

    time_t plan_start_time;
    bool   timeout();

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
    double         stop_s         = 0;

    int current_speed_id = sizeof(speeds) / sizeof(double) - 1;

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
    void run(int target_index);

    //-----------------------------------------------------------
    // SpeedPlanner
    //-----------------------------------------------------------
    void planSpeed(int target_index);

    //-----------------------------------------------------------
    // AStar
    //-----------------------------------------------------------

    // inner classes of PathPlanner
    struct astate {
        astate() {}
        astate(double x, double y, double a) {
            this->x = x;
            this->y = y;
            this->a = a;
        }
        double x = 0, y = 0, a = 0;
        int    prior_index = -1;
        double cost        = 0;
        double curvature   = 0;
        bool   backward    = false;
        // trans_prim != NULL means accurate positions of each point have not been calculated
        //  when extended the primitive, and only the front and back point appears in stored_states
        // when a path is finally collected, all states with trans_prim set need more transfromations
        vector<astate>* trans_prim = NULL;
    };

    struct primitives {
        vector<vector<astate>> forward_primitives;
        vector<vector<astate>> backward_primitives;

        void addPrimitive(vector<astate>& forward_primitive, bool need_mirror = true);
    };

    void aStarPlan(int target_index);

    void aStarPretreatment();

    void aStarExtend(const astate& source, vector<vector<astate>>& destination, double current_euclidean);

    double aStarHeuristic(int target_index, const astate& current);

    bool aStarIsTarget(int target_index, const astate& current);

    int aStarAnalyticExpansionsInterval(double distance);

    bool aStarAnalyticExpansion(int target_index, const astate& state, vector<astate>& expansion_states, double radius);

    int aStarGetUsed(const astate& state);

    void aStarSetUsed(const astate& state, double value);

    void aStarTransformPrimitive(const astate& source, vector<astate>* primitive_ptr, vector<astate>& dest);

    // Update the cost of input state in discrete cost table
    // Return if the cost get smaller
    bool aStarUpdateCost(int state_index, int target_index);

    bool aStarIsLineSafe(const astate& a, const astate& b);

    /* Returns an approximation of the distance to target from (x, y)*/
    double aStarGetDistanceToTarget(int target_index, double x, double y);

    /* Extended positions is precomputed and waiting to be scaled to the target heading. */
    void aStarInitializeExtendedPositions();

    void generateLeftArc(int speed_id, vector<astate>& positions);

    void generateLine(int speed_id, vector<astate>& positions);

    //-----------------------------------------------------------
    // AStar temporary memories
    //------------------------------------------------------------

    static const int DISCRETE_ANGLE_NUM  = 72;                    // change this when look-up table changed
    const double     UNIT_DISCRETE_ANGLE = 0.087266462599716478;  // 2 * PI / 144

    template <typename T> class TemplateMap2 {
    public:
        T map[MAX_ROW][MAX_COL];
    };

    template <typename T> class TemplateMap3 {
    public:
        T map[MAX_ROW][MAX_COL][DISCRETE_ANGLE_NUM];
    };

    DistanceTable astar_dubins_distance_table;
    DistanceTable astar_rs_distance_table;

    TemplateMap2<double> astar_distance_to_multi_targets[MAX_TARGET_NUM];
    std::queue<std::pair<int, int>>             astar_queues[MAX_TARGET_NUM];
    std::priority_queue<std::pair<double, int>> astar_priority_queues[MAX_TARGET_NUM];
    vector<astate>     astar_stored_states[MAX_TARGET_NUM];
    TemplateMap3<int>  astar_lowest_cost_state[MAX_TARGET_NUM];
    TemplateMap2<bool> astar_used[MAX_TARGET_NUM];

    primitives astar_speed_primitives[sizeof(speeds) / sizeof(double)];

    //-----------------------------------------------------------
    // Tool methods
    //-----------------------------------------------------------

    void calculateDistanceBySPFA(double map[MAX_ROW][MAX_COL], double output[MAX_ROW][MAX_COL], const pair<int, int>& target, queue<pair<int, int>>& using_queue);

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