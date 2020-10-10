#include "path_planner_view.h"
#include <cstring>
using namespace cv;

namespace TiEV{

PathPlannerView::PathPlannerView(){
    memset(used_map, 0x00, sizeof(used_map));
    memset(map, 0xff, sizeof(map));
}

void PathPlannerView::draw(Mat& view){
    mtx.lock();
    view = VEC_TiEV_BLACK;
    drawSafeMap(map, view);
    drawUsedMap(used_map, view);
    for(int i = 0; i < paths.size(); ++i)
        drawPath(paths[i], PATH_COLORS[i], view);
    drawStartPoint(start_point, view);
    for(auto& target : targets)
        drawTarget(target, view);
    mtx.unlock();
}

void PathPlannerView::setStartPoint(const TiEV::Point& start){
    mtx.lock_shared();
    start_point.first = start.x;
    start_point.second = start.y;
    mtx.unlock_shared();
}

void PathPlannerView::setTargets(const vector<TiEV::Point>& targets){
    mtx.lock_shared();
    this->targets.resize(targets.size());
    for(int i = 0; i < targets.size(); ++i){
        this->targets[i].first = round(targets[i].x);
        this->targets[i].second = round(targets[i].y);
    }
    mtx.unlock_shared();
}

void PathPlannerView::setMap(int map[GRID_ROW][GRID_COL]){
    mtx.lock_shared();
    memcpy(this->map, map, sizeof(this->map));
    mtx.unlock_shared();
}

void PathPlannerView::setUsedMap(bool used_map[GRID_ROW][GRID_COL]){
    mtx.lock_shared();
    memcpy(this->used_map, used_map, sizeof(this->used_map));
    mtx.unlock_shared();
}

void PathPlannerView::setPath(const vector<TiEV::Point>& path){
    mtx.lock_shared();
    paths_mtx.lock();
	vector<pixel> tmp(path.size());
    for(int i = 0; i < path.size(); ++i){
        tmp[i].first = round(path[i].x);
        tmp[i].second = round(path[i].y);
    }
	paths.emplace_back();
    swap(tmp, paths.back());
    paths_mtx.unlock();
    mtx.unlock_shared();
}

void PathPlannerView::clearPaths(){
    mtx.lock_shared();
    paths_mtx.lock();
    this->paths.clear();
    paths_mtx.unlock();
    mtx.unlock_shared();
}

void PathPlannerView::drawSafeMap(int map[GRID_ROW][GRID_COL], Mat& view){
    for(int i = 0; i < GRID_ROW; ++i)
    	for(int j = 0; j < GRID_COL; ++j)
    	    if(!map[i][j]) view.at<Vec3b>(i, j) = VEC_OBS_COLOR;
}

void PathPlannerView::drawUsedMap(bool map[GRID_ROW][GRID_COL], Mat& view){
    for(int i = 0; i < GRID_ROW; ++i)
    	for(int j = 0; j < GRID_COL; ++j)
    	    if(map[i][j]) view.at<Vec3b>(i, j) = USED_COLOR;
}

void PathPlannerView::drawTarget(const pixel& target, Mat& view){
    cv::circle(view, cv::Point(target.second, target.first)
        , 3, TARGET_COLOR, -1);
}

void PathPlannerView::drawPath(const vector<pixel>& path, const Vec3b& color, Mat& view){
    for(auto& point : path){
        view.at<Vec3b>(point.first, point.second) = color;
    }
}

void PathPlannerView::drawStartPoint(const pixel& start, Mat& view){
    cv::circle(view, cv::Point(start.second, start.first)
        , 3, START_POINT_COLOR, -1);
}

}
