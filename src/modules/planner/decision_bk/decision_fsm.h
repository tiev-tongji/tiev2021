#ifndef _DECISION_FSM_H
#define _DECISION_FSM_H

#include "decision_event.h"
#include "message_manager.h"
#include "road_manager.h"
#include "path_planner.h"
#include "speed_optimizer.h"
#include "utils/utils.h"
#include "decision_view.h"
#include "speed_view/speed_view.h"
#include "visualization.h"
#include "nature.h"
#include <queue>
#include "config/config.h"
#include "time_lock/time_lock.h"

using namespace std;
using namespace tinyfsm;

namespace TiEV{

//the fsm
class DecisionFSM : public Fsm<DecisionFSM>{
	protected:
		RoadManager *road_manager;
		PathPlanner *path_planner;
		DecisionView *dv;
		SpeedView *sv;
		Visualization *vs;

		const int ONE_SIDE_TARGET_NUM = 1;
		const int ALL_TARGET_NUM = 5;
		const double LATERAL_INTERVAL = 3.5; //(m)
		const double LONGITUDE_INTERVAL = 1; //(m)


	/*from road manager*/
		vector<Point> all_reference_path;	//reference path
		vector<RoadInfo> all_road_infoes;
		vector<Point> reference_path;	//reference path
		vector<RoadInfo> road_infoes;
		vector<Point> maintained_path;	//maintained path

	/*car road point mode*/
		double car_speed = 0;
		int road_mode = -1;				//road mode from road file
		int speed_mode = -1;			//speed mode from road file

	/*get by this event*/
		int safe_map[GRID_ROW][GRID_COL];
		int abs_safe_map[GRID_ROW][GRID_COL];
		int lane_safe_map[GRID_ROW][GRID_COL];
		bool reachable[GRID_ROW][GRID_COL];
		vector<Point> start_maintained_path;
		SpeedPath best_path;
		vector<Point> task_points;
		vector<Point> target_points;
		vector<bool> block_lane;
		Point last_target_point;
		double last_target_lane_id = 0;
		int last_target_lane_num = 0;
		bool not_in_lane = false;
		double last_target_dis_to_refer = 0;
		double offset_left = 0;
		double offset_right = 0;
		int car_current_hd_map_lane_id  = 0;
		double car_dis_to_refer = 0; 

	/*get from path planner*/
		vector<SpeedPath> planned_paths;

	/*for visualization*/
		ViewInfo view_info;

	public:
		DecisionFSM(){
			Config::getInstance();
			road_manager = RoadManager::getInstance();
			path_planner = PathPlanner::getInstance();
			dv = DecisionView::getInstance();
			sv = SpeedView::getInstance();
			vs = Visualization::getInstance();
		};
		void react(Event const & e){};

		virtual void react(PlanningEvent const & e);

		virtual void entry(){};
		virtual void exit(){};

	/*refresh the all member*/
		void refresh();

	protected:
	/*the primary functions*/
		bool getInfoByNavinfo(const NavInfo& nav_info);
		bool getMap(const LidarMap& lidar_map, const LaneList& lanes);
		void visualization(const PlanningEvent& e);
		bool plan(const DynamicObjList &dynamic_objs, double max_speed, bool reverse = false);
		void speedReplan(const DynamicObjList &dynamic_objs);
	/*for different state*/
		vector<Point> getTargetsOfParkingLots(ParkingLotList const &parking_lots);
		vector<Point> getTargetsOfLanes(LaneList const &lanes);
		vector<Point> getTargetsOfReference();
		vector<Point> getTargetsByMap();

	/*get best path
	 *if the best path has reverse point return false, else return true
	 */
		bool getBestPath();
	/*update maintained path*/
		void updateMaintainedPath(NavInfo const &nav_info, vector<Point> const &path);

        bool getReferencePath(NavInfo const &nav_info, bool opposite = false);
        double getReferenceSpeed();
		double getSpeedByMode(int _speed_mode);
		void correctReferencePathByLaneLine(LaneList const &lanes);
		void getSafeMap(LidarMap const &lidar_map, LaneList const &lanes);
		void getAbsSafeMap(LidarMap const &lidar_map);
		void getLaneSafeMap(const LaneList& lanes);
		void getStartMaintainedPath(NavInfo const &nav_info);
		void getReachableMap();
		bool safePoint(Point const &point);
		bool absSafePoint(Point const &point);
		bool completTask();
		void updateTaskPoints(const NavInfo& nav_info);
};

}//TiEV end

#endif
