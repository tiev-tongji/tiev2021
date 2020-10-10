#ifndef _PATH_PLANNER_VIEW_
#define _PATH_PLANNER_VIEW_

#include "opencv2/opencv.hpp"
#include "message_manager.h"
#include "nature.h"
#include "TiEV_colors.h"
#include <mutex>
#include "TiEV_colors.h"

using namespace cv;

typedef pair<int, int> pixel;

namespace TiEV{

static mutex path_planner_view_mtx;

static const cv::Vec3b PATH_COLORS[] = {
    cv::Vec3b(0xff, 0xaa, 0xaa),
    cv::Vec3b(0xaa, 0xff, 0xaa),
    cv::Vec3b(0xaa, 0xaa, 0xff),
    cv::Vec3b(0xff, 0xff, 0xaa),
    cv::Vec3b(0xaa, 0xff, 0xff)
};

class PathPlannerView{
public:
    static PathPlannerView* getInstance(){
		path_planner_view_mtx.lock();
		static PathPlannerView instance;
		path_planner_view_mtx.unlock();
		return &instance;
	}

    void setTargets(const vector<TiEV::Point>& targets);
    void setStartPoint(const TiEV::Point& start);
    void setMap(int map[GRID_ROW][GRID_COL]);
    void setUsedMap(bool used_map[GRID_ROW][GRID_COL]);
    void setPath(const vector<TiEV::Point>& path);
    void clearPaths();

    void draw(Mat& view);
private:
    PathPlannerView();
    ~PathPlannerView(){}

    void drawStartPoint(const pixel& start, Mat& view);
    void drawTarget(const pixel& target, Mat& view);
    void drawSafeMap(int map[GRID_ROW][GRID_COL], Mat& view);
    void drawUsedMap(bool map[GRID_ROW][GRID_COL], Mat& view);
    void drawPath(const vector<pixel>& path, const Vec3b& color, Mat& view);

    pixel start_point;
    vector<pixel> targets;
    int map[GRID_ROW][GRID_COL];
    bool used_map[GRID_ROW][GRID_COL];
    vector<vector<pixel>> paths;

    TiEV::shared_mutex mtx;
    mutex paths_mtx;
};

}

#endif
