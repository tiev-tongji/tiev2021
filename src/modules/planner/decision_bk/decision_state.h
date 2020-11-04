#ifndef _DECISION_STATE_H
#define _DECISION_STATE_H

#include "decision_fsm.h"

namespace TiEV{
//A state Demo:
class StateDemo: public DecisionFSM{
    public:
	    void entry() override;
	    void exit() override;
	    void react(PlanningEvent const & e) override;
};

//A Stop State:
class StopState: public DecisionFSM{
    public:
	    void entry() override;
	    void exit() override;
	    void react(PlanningEvent const & e) override;
	private:
		TimeLock stop_lock;
};

//Tracking
class Tracking: public DecisionFSM{
    public:
	    void entry() override;
	    void exit() override;
	    void react(PlanningEvent const & e) override;
};

//BcakTracking
class BackTracking: public DecisionFSM{
    public:
	    void entry() override;
	    void exit() override;
	    void react(PlanningEvent const & e) override;
};

//Global Planning
class GlobalPlanning : public DecisionFSM{
    public:
	    void entry() override;
	    void exit() override;
	    void react(PlanningEvent const & e) override;
};

//Global RePlanning
class GlobalRePlanning : public DecisionFSM{
    public:
	    void entry() override;
	    void exit() override;
	    void react(PlanningEvent const & e) override;
};

//Free Driving State
class FreeDriving : public DecisionFSM{
	public:
		void entry() override;
		void exit() override;
        bool getMap(const LidarMap& lidar_map, const LaneList& lanes);
        void react(PlanningEvent const & e) override;
	private:
		TimeLock no_solution;
};

//Normal driving
class NormalDriving : public DecisionFSM{
    public:
	    void entry() override;
	    void exit() override;
	    void react(PlanningEvent const & e) override;
	private:
		TimeLock no_solution;
};

//Intersection state
class Intersection : public DecisionFSM{
    public:
	    void entry() override;
	    void exit() override;
	    void react(PlanningEvent const & e) override;
		bool getMap(const LidarMap& lidar_map, const LaneList& lanes);
	private:
		TimeLock intersection_lock;
};

//Parking state
class Parking : public DecisionFSM{
    public:
	    void entry() override;
	    void exit() override;
	    void react(PlanningEvent const & e) override;
	private:
		structSLAMCONTROL slam_control;
};

//Lane driving
class LaneDriving : public DecisionFSM{
    public:
	    void entry() override;
	    void exit() override;
	    void react(PlanningEvent const & e) override;
};

//Map free
class MapFree: public DecisionFSM{
    public:
	    void entry() override;
	    void exit() override;
	    void react(PlanningEvent const & e) override;
};

//Foggy Driving
class FoggyDriving: public DecisionFSM{
    public:
	    void entry() override;
	    void exit() override;
	    void react(PlanningEvent const & e) override;
	private:
		bool getMap(LidarMap const& lidar_map, const LaneList& lanes);
};

//Uturn
class UturnState: public DecisionFSM{
    public:
	    void entry() override;
	    void exit() override;
	    void react(PlanningEvent const & e) override;
	private:
		bool getInfoByNavinfo(const NavInfo& nav_info);
		bool getMap(LidarMap const& lidar_map, const LaneList& lanes);
		bool getTargetOfUturn(const NavInfo& nav_info);
		TimeLock no_target;
		Point uturn_target;
};

}//TiEV end

#endif
