#include "nature/point.h"
#include "nature.h"
#include "nature/timestamp.h"
#include "look_up_tables/distance_table.h"
#include "config/config.h"
#include "message_manager.h"
#include "speed_optimizer.h"
#include "path_planner_view.h"
#include <queue>
#include <vector>
#include <thread>
#include <map>

using namespace std;

namespace TiEV{

//#define COUT_DEBUG_INFO
//#define NO_SPEED_PLANNER

const int MAX_TARGET_NUM = 5;
static mutex path_planner_mtx;

const double speeds[] = {
    5 / 3.6, 10 / 3.6, 20/ 3.6, 30/ 3.6, 40/ 3.6, 50/ 3.6, 60/ 3.6, 70/ 3.6
};

const double curvatures[] = {
  0.2, 0.13437, 0.03939, 0.01922, 0.01155, 0.00778, 0.0056, 0.00429,
};

const double arc_lengths[] = {
  1.0, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0
};

const int speed_cnt = sizeof(speeds) / sizeof(double);

class PathPlanner{

public:
    static PathPlanner* getInstance(){
        path_planner_mtx.lock();
        static PathPlanner inner_instance;
        path_planner_mtx.unlock();
        return &inner_instance;
    }

    void setAbsSafeMap(int map[GRID_ROW][GRID_COL]);

    void setLaneSafeMap(int map[GRID_ROW][GRID_COL]);

    void setSafeMap(int map[GRID_ROW][GRID_COL]);
    void setLaneMap(int map[GRID_ROW][GRID_COL]);
    void setAbsMap(int map[GRID_ROW][GRID_COL]);

    void setStartMaintainedPath(const vector<Point>& start_maintained_path);

    void setTargets(const vector<Point>& targets);

    void setBackwardEnabled(bool enabled);

    void setCurrentSpeed(double speed);

	void setDynamicObjList(const DynamicObjList& dynamic_obj_list);

    void setVelocityLimit(double speed);

    void plan();

	bool getResults(vector<SpeedPath> &results);

private:
    PathPlanner();
    ~PathPlanner(){};

    const Config* config;
    PathPlannerView* view_controller;

    //is_planning
    bool is_planning = false;

    //search timer

    time_t plan_start_time;
    bool timeout();

    //planner settings
    bool backward_enabled = false;
    Point start_point;
    vector<Point> targets;

    //messages
    int safe_map[GRID_ROW][GRID_COL];
    int lane_safe_map[GRID_ROW][GRID_COL];
    int abs_safe_map[GRID_ROW][GRID_COL];
    DynamicObjList dynamic_obj_list;
    double current_speed = 0;
    double velocity_limit = 20;
    double stop_s = 0;

    int current_speed_id = sizeof(speeds) / sizeof(double) - 1;

    //visualization
    bool used_map[GRID_ROW][GRID_COL];

    //results

	SpeedPath speed_paths[MAX_TARGET_NUM];
    vector<Point> start_maintained_path;
    vector<pair<double, double>> speed_limits[MAX_TARGET_NUM];
    bool have_result[MAX_TARGET_NUM];

    //-----------------------------------------------------------
    //Threads
    //-----------------------------------------------------------
    void run(int target_index);

    //-----------------------------------------------------------
    //SpeedPlanner
    //-----------------------------------------------------------
    void planSpeed(int target_index);

    //-----------------------------------------------------------
    //AStar
    //-----------------------------------------------------------

    //inner classes of PathPlanner
    struct astate{
        astate(){}
        astate(double x, double y, double a){
            this->x = x;
            this->y = y;
            this->a = a;
        }
        double x = 0, y = 0, a = 0;
        int prior_index = -1;
        double cost = 0;
        double curvature = 0;
        bool backward = false;
        //trans_prim != NULL means accurate positions of each point have not been calculated
        //  when extended the primitive, and only the front and back point appears in stored_states
        //when a path is finally collected, all states with trans_prim set need more transfromations
        vector<astate>* trans_prim = NULL;
    };

    struct primitives{
        vector<vector<astate>> forward_primitives;
        vector<vector<astate>> backward_primitives;

        void addPrimitive(vector<astate>& forward_primitive, bool need_mirror = true);
    };

    void aStarPlan(int target_index);

    void aStarPretreatment();

    void aStarExtend(const astate& source, vector<vector<astate>>& destination);

    double aStarHeuristic(int target_index, const astate& current);

    bool aStarIsTarget(int target_index, const astate& current);

    int aStarAnalyticExpansionsInterval(double distance);

    bool aStarAnalyticExpansion(int target_index, const astate& state,
        vector<astate>& expansion_states, double radius);

    int aStarGetUsed(const astate& state);

    void aStarSetUsed(const astate& state, double value);

    void aStarTransformPrimitive(const astate& source, vector<astate>* primitive_ptr, vector<astate>& dest);

    //Update the cost of input state in discrete cost table
    //Return if the cost get smaller
    bool aStarUpdateCost(int state_index, int target_index);

    bool aStarIsLineSafe(const astate& a, const astate& b);

    /* Returns an approximation of the distance to target from (x, y)*/
    double aStarGetDistanceToTarget(int target_index, double x, double y);

    /* Extended positions is precomputed and waiting to be scaled to the target heading. */
    void aStarInitializeExtendedPositions();

    void generateLeftArc(int speed_id, vector<astate>& positions);

    void generateLine(int speed_id, vector<astate>& positions);

    //-----------------------------------------------------------
    //AStar temporary memories
    //------------------------------------------------------------

    static const int DISCRETE_ANGLE_NUM = 72; //change this when look-up table changed
    const double UNIT_DISCRETE_ANGLE = 0.087266462599716478; //2 * PI / 144

    template<typename T> class TemplateMap2{
    public:
        T map[GRID_ROW][GRID_COL];
    };

    template<typename T> class TemplateMap3{
    public:
        T map[GRID_ROW][GRID_COL][DISCRETE_ANGLE_NUM];
    };

    DistanceTable astar_dubins_distance_table;
    DistanceTable astar_rs_distance_table;

    TemplateMap2<double> astar_distance_to_multi_targets[MAX_TARGET_NUM];
    std::queue<std::pair<int, int>> astar_queues[MAX_TARGET_NUM];
    std::priority_queue<std::pair<double, int>> astar_priority_queues[MAX_TARGET_NUM];
    vector<astate> astar_stored_states[MAX_TARGET_NUM];
    TemplateMap3<int> astar_lowest_cost_state[MAX_TARGET_NUM];
    TemplateMap2<bool> astar_used[MAX_TARGET_NUM];

    primitives astar_speed_primitives[sizeof(speeds) / sizeof(double)];

    //-----------------------------------------------------------
    //Tool methods
    //-----------------------------------------------------------

    void calculateDistanceBySPFA(
        int map[GRID_ROW][GRID_COL],
        double output[GRID_ROW][GRID_COL],
        const pair<int, int>& target,
        queue<pair<int, int>>& using_queue);

public:
    /* Safe map is an integer map
     * output[i][j] == 0 => point (i,j) is safe
     * output[i][j] > 0 => point (i,j) is too close to some obstacles
     */
    static void calculateSafeMap(u_char map[GRID_ROW][GRID_COL],
        int output[GRID_ROW][GRID_COL], int safe_distance,
        queue<pair<int, int>>& using_queue)
    {
        const int dx[] = {0, 0, -1, 1};
        const int dy[] = {-1, 1, 0, 0};

        memset(output, 0, sizeof(int) * GRID_ROW * GRID_COL);

        if(safe_distance <= 0) return;

        while(!using_queue.empty()) using_queue.pop();
        for(int i = 0; i < GRID_ROW; ++i)
            for(int j = 0; j < GRID_COL; ++j)
                if(map[i][j]){
                    output[i][j] = safe_distance;
                    using_queue.push(make_pair(i,j));
                }

        while(!using_queue.empty()){
            const int x = using_queue.front().first;
            const int y = using_queue.front().second;
            using_queue.pop();
            for(int i = 0; i < 4; ++i){
                const int tx = x + dx[i], ty = y + dy[i];
                if(tx >= 0 && tx < GRID_ROW &&
                    ty >= 0 && ty < GRID_COL &&
                    output[tx][ty] < output[x][y] - 1){
                    output[tx][ty] = output[x][y] - 1;
                    using_queue.push(make_pair(tx, ty));
                }
            }
        }
    }

    /* Need a safe_map generated by PathPlanner::calculateSafeMap */
    static inline bool isCarSafeHere(double x, double y, double a, int abs_safe_map[GRID_ROW][GRID_COL], int lane_safe_map[GRID_ROW][GRID_COL], double car_speed){
		double expanded_r = (car_speed * 0.06) / GRID_RESOLUTION;
		if(x < 0 || x >= GRID_ROW || y < 0 || y >= GRID_COL) return false;
		double cos_ang = cos(a);
		double sin_ang = sin(a);
		double big_circle_x = x - COLLISION_CIRCLE_BIG_L_TO_CAR_FRONT_AXLE * cos_ang / GRID_RESOLUTION;
		double big_circle_y = y + COLLISION_CIRCLE_BIG_L_TO_CAR_FRONT_AXLE * sin_ang / GRID_RESOLUTION;
		if(abs_safe_map[(int)round(big_circle_x)][(int)round(big_circle_y)] > expanded_r + ABS_COLLISION_CIRCLE_BIG_R / GRID_RESOLUTION \
		&& lane_safe_map[(int)round(big_circle_x)][(int)round(big_circle_y)] > ABS_COLLISION_CIRCLE_BIG_R / GRID_RESOLUTION) return true;
		double small_circle_x_1 = x - COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_1 * cos_ang / GRID_RESOLUTION;
		double small_circle_y_1 = y + COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_1 * sin_ang / GRID_RESOLUTION;
		if(abs_safe_map[(int)round(small_circle_x_1)][(int)round(small_circle_y_1)] <= expanded_r + ABS_COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION \
		|| lane_safe_map[(int)round(big_circle_x)][(int)round(big_circle_y)] <= ABS_COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION) return false;
		double small_circle_x_2 = x - COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_2 * cos_ang / GRID_RESOLUTION;
		double small_circle_y_2 = y + COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_2 * sin_ang / GRID_RESOLUTION;
		if(abs_safe_map[(int)round(small_circle_x_2)][(int)round(small_circle_y_2)] <= expanded_r + ABS_COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION \
		|| lane_safe_map[(int)round(big_circle_x)][(int)round(big_circle_y)] <= ABS_COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION) return false;
		double small_circle_x_3 = x - COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_3 * cos_ang / GRID_RESOLUTION;
		double small_circle_y_3 = y + COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_3 * sin_ang / GRID_RESOLUTION;
		if(abs_safe_map[(int)round(small_circle_x_3)][(int)round(small_circle_y_3)] <= expanded_r + COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION \
		|| lane_safe_map[(int)round(big_circle_x)][(int)round(big_circle_y)] <= ABS_COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION) return false;
		double small_circle_x_4 = x - COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_4 * cos_ang / GRID_RESOLUTION;
		double small_circle_y_4 = y + COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_4 * sin_ang / GRID_RESOLUTION;
		if(abs_safe_map[(int)round(small_circle_x_4)][(int)round(small_circle_y_4)] <= expanded_r + ABS_COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION \
		|| lane_safe_map[(int)round(big_circle_x)][(int)round(big_circle_y)] <= ABS_COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION) return false;
		return true;
    }

    /* It is what it seems to be */
    static inline double euclideanDistance(const double& x_0, const double& y_0,
        const double& x_1, const double& y_1){
        const double dx = x_0 - x_1;
        const double dy = y_0 - y_1;
        return sqrt(dx * dx + dy * dy);
    }

    static inline double curvatureConstraint(const double& speed_2_ms, const double& steering_ability){
        return steering_ability * GRAVITY / speed_2_ms;
    }

};

}
