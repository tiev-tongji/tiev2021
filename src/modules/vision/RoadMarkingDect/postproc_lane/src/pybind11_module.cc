#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <Eigen/Dense>
#include <pybind11/stl.h>
#include<vector>

#include <unistd.h>
#include "zcmmsg/structRoadMarkingList.hpp"
#include "time.h"
#include "common.h"

//#undef DEPLOY_IMSHOW
//#define DEPLOY_IMSHOW(x, key)

#include "rotate.h"
#include "cluster.h"
#include "lane_model.h"
#include "polynomial.h"
#include "filter.h"
#include "visualize.h"

zcm::ZCM zcm_udp{};

namespace py = pybind11;
namespace lm = LaneModel;
using namespace Eigen;

const float LANEWIDTH = 70;
float pixel2real_ratio = 5.2 / 100;
int min_stop_count = 200; //large for highway, small for ordinary road(200), median beneath highway(400)
int min_lanemark_count = 500;

const int LASERMAPBEV_HEIGHT = 1024;
const int LASERMAPBEV_WIDTH = 400;
const int VISUALBEV_WIDTH = 256;

//coodinate convertion
cv::Point2f convert_to_lidar(const cv::Point2f& point_bev){
    cv::Point2f point_lidar;
    cv::Mat point_bev_ = (cv::Mat_<double>(3, 1)<< point_bev.x, point_bev.y, 1);
//    static cv::Mat homography = (cv::Mat_<double>(3, 3) <<
//                                4.49541030e-02,  2.78755872e-04, -5.85124919e+00,
//                                3.22243481e-03, -4.76127770e-02,  4.97163722e+01,
//                               -2.08830189e-04, -4.86239754e-05,  1.00000000e+00);
    static cv::Mat homography = (cv::Mat_<double>(3, 3) <<
                        -0.0410305872674881, -0.0103373109727542, 23.53885022741934,
                        -0.03468408208563678, 0.002099521760837547, 7.260788823703837,
                        -0.005287725315834751, 0.001003415840533877, 1.0);
    cv::Mat point_lidar_ = homography * point_bev_;
    point_lidar.x = point_lidar_.at<double>(0, 0) / point_lidar_.at<double>(2, 0);
    point_lidar.y = point_lidar_.at<double>(1, 0) / point_lidar_.at<double>(2, 0);

    return point_lidar;//unit m
}

cv::Point2i convert_to_bev(const cv::Point2f& point_lidar){
    cv::Point2f point_bev;
    const float LIDARRESOLUTION = 0.2;
    const int LIDARMAPWIDTH = 150;
    const int LIDARMAPHEIGHT = 300;
    cv::Mat point_lidar_ = (cv::Mat_<double>(3, 1)<< (point_lidar.y - LIDARMAPWIDTH/2)*LIDARRESOLUTION, (LIDARMAPHEIGHT - point_lidar.x)*LIDARRESOLUTION, 1);
    static cv::Mat homography = (cv::Mat_<double>(3, 3) <<
                                                        -0.0410305872674881, -0.0103373109727542, 23.53885022741934,
                                                        -0.03468408208563678, 0.002099521760837547, 7.260788823703837,
                                                        -0.005287725315834751, 0.001003415840533877, 1.0);
    cv::Mat point_bev_ = homography.inv() * point_lidar_;
    point_bev.x = point_bev_.at<double>(0, 0) / point_bev_.at<double>(2, 0);
    point_bev.y = point_bev_.at<double>(1, 0) / point_bev_.at<double>(2, 0);
    return cv::Point(
            static_cast<int>(point_bev.y + .5),
            static_cast<int>(point_bev.x + .5));
}

cv::Mat convert_lasermap_2bev(const cv::Mat laser_map_cropped){
    cv::Mat laser_map_bev(LASERMAPBEV_HEIGHT,LASERMAPBEV_WIDTH, CV_8UC1, cvScalar(0));
    int rows = laser_map_cropped.rows;
    int cols = laser_map_cropped.cols;
    for(int i=0; i<rows; i++){
        for(int j=0; j<cols; j++){
            if(laser_map_cropped.at<uchar>(i,j) > 0){
                cv::Point2i obstcal_point_laser(i, j);
                cv::Point2i obstcal_point_bev = convert_to_bev(obstcal_point_laser);
//                std::cout << "laser_point:    " << obstcal_point_laser << "    bev_point:   " << obstcal_point_bev <<std::endl;
                if(obstcal_point_bev.x  <0 || obstcal_point_bev.x  >= LASERMAPBEV_HEIGHT ||
                    obstcal_point_bev.y +(LASERMAPBEV_WIDTH-VISUALBEV_WIDTH)/2 <0 ||
                    obstcal_point_bev.y +(LASERMAPBEV_WIDTH-VISUALBEV_WIDTH)/2>= LASERMAPBEV_WIDTH)
                    continue;
                else
                    laser_map_bev.at<uchar>(obstcal_point_bev.x, obstcal_point_bev.y+(LASERMAPBEV_WIDTH-VISUALBEV_WIDTH)/2) = 1;
            }
        }
    }
    return laser_map_bev;

}

//function for zcm
void fetch_lanelinepoints(lm::Line<float>* line, std::vector<LinePoint>& points){
    cluster::Cluster_<float> tmp = line->points();
    for (auto item : tmp){
        LinePoint p;
        cv::Point2f point_lidar = convert_to_lidar(item);
        p.x = point_lidar.x;
        p.y = point_lidar.y;
        points.push_back(p);
    }
}

void struct_laneline(LaneLine& laneline, lm::Line<float>* line, int cols, int rows){
    laneline.line_type = line->linetype;
    laneline.distance = line->distance;
    laneline.boundary_type = line->boundary_type;
    laneline.boundary_confidence = line->boundary_confidence;
    printf("boundary type: ___%d      ", laneline.boundary_type);
    printf("boundary confidence: ___%d\n", laneline.boundary_confidence);
    laneline.num = line->points().size();
    fetch_lanelinepoints(line, laneline.points);
}

//input to Mat
template <int nchannel>
cv::Mat convertToCvMat(py::array_t<uchar_t>& in) {
    py::buffer_info buf = in.request();
    if (buf.ndim != 3)
        throw std::runtime_error("Number of dimensions must be 3");
    if (buf.shape[2] != nchannel) {
        std::stringstream ss;
        ss << "Buffer channel size incorrect, expecting " << nchannel <<
            ", got " << buf.shape[2];
        throw std::runtime_error(ss.str());
    }
    cv::Mat out(static_cast<int>(buf.shape[0]),
            static_cast<int>(buf.shape[1]), CV_8UC(nchannel), buf.ptr);
    return out;
}

cv::Mat convertLMToCvMat(py::array_t<uchar_t>& in) {
    py::buffer_info buf = in.request();
    if (buf.ndim != 2)
        throw std::runtime_error("Number of dimensions must be 2");
    cv::Mat out(static_cast<int>(buf.shape[0]),
                static_cast<int>(buf.shape[1]), CV_8UC1, buf.ptr);
    return out;
}

std::vector<cv::Mat> splitToCvMat(py::array_t<uchar_t>& in) {
    py::buffer_info buf = in.request();
    if (buf.ndim != 3)
        throw std::runtime_error("Number of dimensions must be 3");
    std::vector<cv::Mat> out;
    out.reserve(static_cast<size_t>(buf.shape[0]));
    for(uchar_t * p = static_cast<uchar_t*>(buf.ptr), i=0; i<buf.shape[0];
        ++i, p += buf.shape[1] * buf.shape[2]) {
        out.push_back(
            cv::Mat (static_cast<int>(buf.shape[1]),
            static_cast<int>(buf.shape[2]), CV_8UC1, static_cast<void*>(p))
        );
    }
    return out;
}

#define PROCESS_FAILED(retval) { return retval; }

class IntersectionMode {
public:
    typedef enum {PRE_INTERSECTION, INTERSECTION, DEFAULT} mode_t;
    static mode_t mode;
    static int stop_line_det_count;
    static int stop_line_mis_count;
    static int image_align_ok_count;
    static const int STOP_LINE_COUNT_DET_THRES = 10;
    static const int STOP_LINE_COUNT_MIS_THRES = 10;
    static const int IMAGE_ALIGN_OK_COUNT_THRES = 5;
    static void transition(bool image_align_ok, bool stopline_detected) {
        PRINT(stopline_detected)
        PRINT(stop_line_det_count);
        PRINT(stop_line_mis_count);
        PRINT(image_align_ok_count);
        if(mode == DEFAULT){
            if(stopline_detected){++stop_line_det_count;}
            if(!stopline_detected && stop_line_det_count > 0){--stop_line_det_count;}
            if(stop_line_det_count > STOP_LINE_COUNT_DET_THRES){
                mode = PRE_INTERSECTION;
                stop_line_mis_count = 0;
            }
        }else if(mode == PRE_INTERSECTION){
            if(!stopline_detected){++stop_line_mis_count;}
            if(stop_line_mis_count > STOP_LINE_COUNT_MIS_THRES){
                mode = INTERSECTION;
                image_align_ok_count = 0;
            }
        }else if(mode == INTERSECTION){
            if(image_align_ok){ ++image_align_ok_count; }
            if(image_align_ok_count > IMAGE_ALIGN_OK_COUNT_THRES) {
                mode = DEFAULT;
                stop_line_det_count = 0;
            }
        }else {
            assert(0);
        }
    }
    friend std::ostream& operator<<(std::ostream& os, const IntersectionMode::mode_t& mode){
        if(mode == DEFAULT) os << "DEFAULT";
        if(mode == INTERSECTION) os << "INTERSECTION";
        if(mode == PRE_INTERSECTION) os << "PRE_INTERSECTION";
        return os;
    }
};
IntersectionMode::mode_t IntersectionMode::mode = IntersectionMode::DEFAULT;
int IntersectionMode::stop_line_det_count = 0;
int IntersectionMode::stop_line_mis_count = 0;
int IntersectionMode::image_align_ok_count = 0;

std::vector<int> process_tensor(py::array_t<uchar_t> _image, py::array_t<uchar_t> _predict, py::array_t<uchar_t> _laser_map, std::vector<int> out_list) {
    // init inputs
    cv::Mat image = convertToCvMat<3>(_image);
    std::vector<cv::Mat> predict = splitToCvMat(_predict);
    cv::Mat laser_map = convertLMToCvMat(_laser_map);
    cv::Mat laser_map_cropped = laser_map(cv::Rect(0,0,151,301));//get visual area from lasermap, car(lader) locat at botoom center. Real area 60m * 14m
//    cv::Point2i test_point(300,75);
//    std::cout << "_________" << convert_to_bev(test_point) << std::endl;
    cv::imshow("windoe0", laser_map*255);
    cv::Mat laser_map_bev;
    laser_map_bev = convert_lasermap_2bev(laser_map_cropped);//convert laser map to bev todo:expand the area beyond the bev
    cv::imshow("windoe", laser_map_bev*255);
//    cv::Mat manual_curb = cv::imread("/home/xzr/tiev-plus/modules/src/vision/RoadMarkingDect/tools/manual_curb.png", CV_LOAD_IMAGE_UNCHANGED);
//    predict.push_back(manual_curb);
    PRINT("ok");
    // static status
    // MOT Kalman Filter for lines
    const float THRES_MATCH_DISTANCE_PIX = 15;
    const float THRES_DETECTED_NUM = 8;
    const float THRES_MISSED_NUM = 3;
    static FilterGroup<Polynomial> filters_poly(
        THRES_MATCH_DISTANCE_PIX, THRES_DETECTED_NUM, THRES_MISSED_NUM);
    static FilterGroup<Polynomial> filters_curb(
            2, 2, 2);
    //const std::vector<cluster::Cluster_<float> > empty_clusters;
    // Low Pass Filter for yaw rotating angle
    static double yaw_angle = 0.0;
    static float y_bias = static_cast<float>(image.rows);
    const double YAW_LOWPASS_RATE = 0.7;
    static float min_y_filtered = 0.0f; //-y_bias;
    const float MIN_Y_LOWPASS_RATE = 0.8f;

    const cv::Point2f CENTER(1.25e+02, 1.05254885e+03f);
    const float VEHICLE_HALF_WIDTH = 0.65f / pixel2real_ratio;

	/*
	static size_t frame_cnt;
    ++frame_cnt;
    PRINT("=======================================");
    PRINT(frame_cnt);
	*/

    const std::vector<size_t> SOLID_IDS = {0, 1, 2, 3, 4, 5};
    const std::vector<size_t> YELLOW_IDS = {1, 3, 5, 7, 9};
    const std::vector<size_t> WHITE_IDS = {0, 2, 4, 6, 8};
    const std::vector<size_t> DASHED_IDS = {6, 7, 8, 9};
    const std::vector<size_t> ZEBRA_IDS = {13};
    const std::vector<size_t> STOPLINE_IDS = {10, 11, 12};

    const std::vector<size_t> ARROW_THRU_IDS = {15, 23, 18, 24};
    const std::vector<size_t> ARROW_LEFT_IDS = {19, 21, 22, 18};
    const std::vector<size_t> ARROW_RIGHT_IDS = {20, 21, 18};
    const std::vector<size_t> ARROW_UTURN_IDS = {14, 22, 23};
    const std::vector<size_t> ARROW_THRU_LEFT_IDS = {16};
    const std::vector<size_t> ARROW_THRU_RIGHT_IDS = {17};

    cv::Mat solid_mask, dashed_mask, yellow_mask, white_mask, zebra_mask, line_mask, line_zebra_mask, arrow_thru_mask, arrow_left_mask, arrow_right_mask, arrow_uturn_mask, arrow_thru_left_mask, arrow_thru_right_mask, stopline_mask, arrow_mask;

    {// init masks
        auto sum_mask = [&predict](cv::Mat& mask, const std::vector<size_t>& ids) {
            for(size_t line_id : ids){
                mask = (mask.empty() ? predict[line_id] : mask + predict[line_id] );
            }
        };
        sum_mask(solid_mask, SOLID_IDS);
        sum_mask(dashed_mask, DASHED_IDS);
        sum_mask(yellow_mask, YELLOW_IDS);
        sum_mask(white_mask, WHITE_IDS);
        sum_mask(zebra_mask, ZEBRA_IDS);
        sum_mask(stopline_mask, STOPLINE_IDS);
        sum_mask(arrow_thru_mask, ARROW_THRU_IDS);
        sum_mask(arrow_left_mask, ARROW_LEFT_IDS);
        sum_mask(arrow_right_mask, ARROW_RIGHT_IDS);
        sum_mask(arrow_uturn_mask, ARROW_UTURN_IDS);
        sum_mask(arrow_thru_left_mask, ARROW_THRU_LEFT_IDS);
        sum_mask(arrow_thru_right_mask, ARROW_THRU_RIGHT_IDS);

        line_mask = solid_mask + dashed_mask;
        line_zebra_mask = line_mask + zebra_mask;
        arrow_mask = arrow_thru_mask + arrow_left_mask + arrow_right_mask + arrow_uturn_mask + arrow_thru_left_mask + arrow_thru_right_mask;
    }

    // hough transform detecting line direction (yaw angle)
    const int SINGLE_SIDE_WIDEN_WIDTH = 128;
    ImageAlign image_align(line_mask, SINGLE_SIDE_WIDEN_WIDTH);
    bool image_align_ok_raw = image_align.ok();
    if (image_align.ok()) {
        yaw_angle = YAW_LOWPASS_RATE * yaw_angle + (1.0 - YAW_LOWPASS_RATE) * image_align.get_yaw();
    }
    image_align.set_yaw(yaw_angle);
    PRINT(image_align.ok());
    PRINT(image_align.get_yaw());
    IMSHOW(line_mask);

    // rotate the image with the yaw angle
    cv::Mat aligned_line_mask;
    cv::Mat aligned_image;
    cv::Point2f aligned_center;
    {
        image_align.align(line_mask, aligned_line_mask);
        image_align.align(image, aligned_image);
        image_align.align(CENTER, aligned_center);
    }
    float car_left_pos = aligned_center.x - VEHICLE_HALF_WIDTH;
    float car_right_pos = aligned_center.x + VEHICLE_HALF_WIDTH;

    // clustering lane lines
    std::vector<cluster::Cluster_<int>> aligned_clusters;
    cluster::cluster_lanes(aligned_line_mask, aligned_clusters);
    PRINT(aligned_clusters.size());

    std::vector<Polynomial> observations;
    float min_y = 0.f;
    bool min_y_updated = false;
    for(cluster::Cluster_<int>& cluster : aligned_clusters){
        Polynomial poly(cluster, y_bias);
        observations.push_back(poly);
        min_y = std::min(min_y, poly.min_y());
        min_y_updated = true;
    }
    //PRINT(min_y_filtered);
    if (min_y_updated) {
        min_y_filtered = MIN_Y_LOWPASS_RATE * min_y_filtered + (1 - MIN_Y_LOWPASS_RATE) * min_y;
    }
    //PRINT(min_y);
    //PRINT(min_y_filtered);
    //PRINT(observations.size());
    filters_poly.update(observations);
    std::vector<Survived<Polynomial> > survived_polys = filters_poly.get_survived_targets();
    //PRINT(survived_polys.size());
    std::vector<cluster::Cluster_<float> > filtered_clusters;
    for(Survived<Polynomial>& survived_poly : survived_polys) {
        //PRINT(survived_poly);
        filtered_clusters.push_back(survived_poly.make_target().resample());
    }
    if (DEBUG) {
        std::sort(aligned_clusters.begin(), aligned_clusters.end(), cluster::less_than<int>);
        cv::Mat aligned_clusters_image = aligned_image.clone();
        visualize::visualize(aligned_clusters, aligned_clusters_image);
        IMSHOW(aligned_clusters_image);
    }
    if (DEBUG) {
        std::sort(filtered_clusters.begin(), filtered_clusters.end(), cluster::less_than<float>);
        cv::Mat filtered_clusters_image = aligned_image.clone();
        visualize::visualize(filtered_clusters, filtered_clusters_image);
        IMSHOW(filtered_clusters_image);
    }


    // construct road model !
    lm::Road<float> road(filtered_clusters);

    // fit polynomial model
    std::map<lm::Line<float>*, Polynomial> params;
    {
        float confidence;
        params = makeSharedPolynomial(road, y_bias, &confidence, 0.);

        if(confidence < 0.3f) {
            //PROCESS_FAILED(-4);
            params.clear();
        }
    }


    {//process narrow lanes
        const float NARROW_LANEWIDTH_RATE = 0.75f;
        auto lines = road.lines();
        auto lanes = road.lanes();
        PRINT(lanes.size());

        std::vector<int> narrow_counts;
        int narrow_count = 0;
        for(auto lane : lanes){ //count maximum continuous narrow lanes
            lane->lanewidth = dynamic_cast<lm::Line<float>*>(lane->right())->points().front().x - dynamic_cast<lm::Line<float>*>(lane->left())->points().front().x;
            if(lane->lanewidth < NARROW_LANEWIDTH_RATE * LANEWIDTH) {
                narrow_count++;
                PRINT(lane->lanewidth)
            }
            else if(narrow_count > 0){
                narrow_counts.push_back(narrow_count);
                narrow_count = 0;
            }
        }
        if(narrow_count > 0) narrow_counts.push_back(narrow_count);

        //NEW ADD :
        if(narrow_counts.size() > 0){
            if(lanes.size() == 1) {
                //PROCESS_FAILED(-8);
                road.clear();
            }
            if(lanes.size() > 1){
                int now_count;
                auto p_lane = lanes.begin();
                for (int i=0; i<narrow_counts.size(); i++){
                    now_count = narrow_counts[i];
                    while ((*p_lane)->lanewidth > NARROW_LANEWIDTH_RATE * LANEWIDTH){ p_lane++;}
                    if (now_count > 2) {road.clear(); break;} // too many narrow lanes, failed
                    else if (now_count == 2) { // two narrow lanes, delete the line between.
                        road.del_line(dynamic_cast<lm::Line<float>*>((*p_lane)->left()));
                        p_lane += 2;
                    }
                    else if (now_count == 1){ // only one narrow lane, disdinguishing between curb and guideline.
                        // when the lane is not at the side of road, we merge the two lines,
                        // they may be guidlines or one of them is a false positive detection.
                        if (p_lane != lanes.begin() && (p_lane != lanes.end()-1)){
                            auto tmp_line = dynamic_cast<lm::Line<float>*>((*p_lane)->right()->right()->right());
                            road.del_line(dynamic_cast<lm::Line<float>*>(tmp_line->left()->left()));
                            road.del_line(dynamic_cast<lm::Line<float>*>(tmp_line->left()->left()));
                            float tmp_lanewidth = tmp_line->points().front().x - dynamic_cast<lm::Line<float>*>(tmp_line->left()->left())->points().front().x;
                            road.push_left(tmp_line, tmp_lanewidth / 2);
                        }else{ // it is curb, we delete the line on the side of the road.
                            float line_left_pos = dynamic_cast<lm::Line<float>*>((*p_lane)->left())->points().front().x;
                            float line_right_pos = dynamic_cast<lm::Line<float>*>((*p_lane)->right())->points().front().x;

                            if(line_left_pos < car_left_pos && line_right_pos > car_right_pos){
                                if(p_lane == lanes.begin()) {road.del_line(dynamic_cast<lm::Line<float>*>((*p_lane)->left()));}
                                else{ road.del_line(dynamic_cast<lm::Line<float>*>((*p_lane)->right()));}
                            }
                            else if (line_left_pos > car_right_pos){ road.del_line(dynamic_cast<lm::Line<float>*>((*p_lane)->right()));}
                            else if (line_right_pos < car_left_pos){ road.del_line(dynamic_cast<lm::Line<float>*>((*p_lane)->left()));}
                            else{
                                if (line_left_pos > car_left_pos && line_left_pos < car_right_pos){
									road.del_line(dynamic_cast<lm::Line<float>*>((*p_lane)->left()));}
                                if (line_right_pos > car_left_pos && line_right_pos < car_right_pos){
									road.del_line(dynamic_cast<lm::Line<float>*>((*p_lane)->right()));}
                            }
                        }
                        p_lane++;
                    }
                }

            }
        }
    }

    // resample polynomial model
    std::vector<cluster::Cluster_<float>> resampled_clusters;
    {
        auto lines = road.lines();
        for(auto& line : lines){
            params[line].set_min_y(min_y_filtered);
            cluster::Cluster_<float> resampled_cluster = params[line].resample();
            if (!resampled_cluster.size()) continue;
            resampled_clusters.push_back(resampled_cluster);
        }
    }
    bool add_right = false;
    bool add_left = false;
    float dis_right_curb = 0.;
    float dis_left_curb = 0.;
    int right_curb_size = 0;
    int left_curb_size = 0;
    const int CURB_POINTS_THRESH = 5;
//    printf("before_resampled_clusters.size() %d = ", resampled_clusters.size());
    //todo: add curb mask cluster
    {
        std::sort(resampled_clusters.begin(), resampled_clusters.end(), cluster::less_than<float>);
        if(resampled_clusters.size() > 0){
            auto right_cluster = resampled_clusters[0];
            auto left_cluster = resampled_clusters.back();
            cluster::Cluster_<float> right_curb;
            cluster::Cluster_<float> left_curb;
            std::vector<cluster::Cluster_<int>> curb_clusters;
            printf("ok-2\n");
            // the right curb
            {
                float min_x_temp = std::min(right_cluster[0].x, right_cluster.back().x);
                float min_y_temp = std::min(right_cluster[0].y , right_cluster.back().y);
                cv::Point2f corner_point(min_x_temp, min_y_temp);
                image_align.revert(corner_point, corner_point);
                cv::Mat temp = cv::Mat::zeros(LASERMAPBEV_HEIGHT,LASERMAPBEV_WIDTH, CV_8UC1);
                std::cout<< corner_point.y <<"   " << "RIGHT" << "   " << static_cast<int>(corner_point.x) << "   " << std::endl;
                corner_point.y = std::max(0, static_cast<int>(corner_point.y+0.5));
                corner_point.x = std::max(0, static_cast<int>(corner_point.x+0.5));
                int x_range = static_cast<int>(corner_point.x+(LASERMAPBEV_WIDTH - VISUALBEV_WIDTH)/2+0.5);
                cv::Mat right_curb_roi = temp(cv::Range(corner_point.y, LASERMAPBEV_HEIGHT),
                                              cv::Range(x_range, std::min(static_cast<int>(x_range+LANEWIDTH*1.5+0.5) , LASERMAPBEV_WIDTH)));//get visual area from lasermap, car(lader) locat at botoom center. Real area 60m * 14m
                right_curb_roi.setTo(1);
                cv::Mat lasermap_bev_rightcurb_roi = temp.mul(laser_map_bev);
                cluster::Cluster right_curb;
                for(int i= corner_point.y; i<LASERMAPBEV_HEIGHT; i++){
                    for(int j= x_range; j<std::min(static_cast<int>(x_range+LANEWIDTH*1.5+0.5) , LASERMAPBEV_WIDTH); j++){
                        if(lasermap_bev_rightcurb_roi.at<uchar>(i, j) > 0){
                            right_curb.push_back(cv::Point2i(static_cast<int>(j-(LASERMAPBEV_WIDTH - VISUALBEV_WIDTH)/2+0.5), i));
                        }
                    }
                }
                if(right_curb.size()>CURB_POINTS_THRESH){
                    right_curb_size = right_curb.size();
                    curb_clusters.push_back(right_curb);
                }
                cv::imshow("right", lasermap_bev_rightcurb_roi*255);
//                cv::Mat tempshow = cv::Mat::zeros(LASERMAPBEV_HEIGHT,LASERMAPBEV_WIDTH, CV_8UC1);
//                cv::polylines(tempshow, right_curb, false, cvScalar(255), 4);
//                cv::imshow("curb", tempshow);
            }

             //the left curb
            {
                float min_x_temp = std::max(left_cluster[0].x, left_cluster.back().x);
                float min_y_temp = std::min(left_cluster[0].y , left_cluster.back().y);
                cv::Point2f corner_point(min_x_temp, min_y_temp);
                image_align.revert(corner_point, corner_point);
                cv::Mat temp2 = cv::Mat::zeros(LASERMAPBEV_HEIGHT,LASERMAPBEV_WIDTH, CV_8UC1);
                std::cout<< corner_point.y <<"   " << "LEFT" << "   " << static_cast<int>(corner_point.x) << "   " << std::endl;
                corner_point.y = std::max(0, static_cast<int>(corner_point.y+0.5));
                corner_point.x = std::max(0, static_cast<int>(corner_point.x+0.5));
                int x_range = static_cast<int>(corner_point.x+(LASERMAPBEV_WIDTH - VISUALBEV_WIDTH)/2+0.5);
                cv::Mat left_curb_roi = temp2(cv::Range(corner_point.y, LASERMAPBEV_HEIGHT),
                                              cv::Range(std::max(static_cast<int>(x_range-LANEWIDTH*1.5+0.5),0) , x_range));//get visual area from lasermap, car(lader) locat at botoom center. Real area 60m * 14m
                left_curb_roi.setTo(1);
                cv::Mat lasermap_bev_leftcurb_roi = temp2.mul(laser_map_bev);
                cluster::Cluster left_curb;
                for(int i= corner_point.y; i<LASERMAPBEV_HEIGHT; i++){
                    for(int j= std::max(static_cast<int>(x_range-LANEWIDTH*1.5+0.5),0); j<x_range; j++){
                        if(lasermap_bev_leftcurb_roi.at<uchar>(i, j) > 0){
                            left_curb.push_back(cv::Point2i(static_cast<int>(j-(LASERMAPBEV_WIDTH - VISUALBEV_WIDTH)/2+0.5), i));
                        }
                    }
                }
                if(left_curb.size()>CURB_POINTS_THRESH){
                    left_curb_size = left_curb.size();
                    curb_clusters.push_back(left_curb);
                }
                cv::imshow("left", lasermap_bev_leftcurb_roi*255);
            }
            printf("ok-1\n");
            if(curb_clusters.size()>0){
                printf("curbs_num %zu\n", curb_clusters.size());
                std::vector<Polynomial> curb_observations;
                printf("ok01\n");
                for(cluster::Cluster_<int>& cluster : curb_clusters){
                    Polynomial poly(cluster, y_bias);
                    curb_observations.push_back(poly);
                    printf("ok02\n");
                }
                if(curb_observations.size() > 0){
                    filters_curb.update(curb_observations);
                    printf("ok1\n");
                    std::vector<Survived<Polynomial> > survived_curbs = filters_curb.get_survived_targets();
                    printf("survived_curbs_num:  %zu\n",  survived_curbs.size());
                    //PRINT(survived_polys.size());
                    for(Survived<Polynomial>& survived_poly : survived_curbs) {
                        //PRINT(survived_poly);
                        if(survived_poly.make_target().resample().size() > 0){
//                            cv::Mat tempshow = cv::Mat::zeros(LASERMAPBEV_HEIGHT,LASERMAPBEV_WIDTH, CV_8UC1);
                            cluster::Cluster_<float> ori_curb = survived_poly.make_target().resample();
//                            for(auto &p : ori_curb){
//                                printf("(%f, %f) ", p.x, p.y);
//                            }
//                            printf("\nafter aligne\n");
                            cluster::Cluster_<float> aligned_curb;
                            for(auto & pts : ori_curb){
                                cv::Point2f aligned_pts;
                                image_align.align(pts, aligned_pts);
                                aligned_curb.push_back(aligned_pts);
                            }
//                            for(auto &p : aligned_curb){
//                                printf("(%f, %f) ", p.x, p.y);
//                            }
//                            image_align.align(ori_curb, aligned_curb);
                            if(aligned_curb.front().x > aligned_center.x){//right curb
                                float dis_line_curb = std::abs(aligned_curb.front().x - right_cluster.front().x);
                                if(dis_line_curb * pixel2real_ratio > 2.5){
                                    add_right = true;
                                    printf("@@add ok1\n");
                                    dis_right_curb = dis_line_curb;
                                }
                            }
                            else{
                                float dis_line_curb = std::abs(aligned_curb.front().x - left_cluster.front().x);
                                if(dis_line_curb * pixel2real_ratio > 2.5){
                                    add_left = true;
                                    dis_left_curb = dis_line_curb;
                                }
                            }
                            printf("@@new ok3\n");
//                            cv::polylines(tempshow, resampled_clusters[0], false, cvScalar(255), 4);
//                            cv::imshow("curb", tempshow);
                        }
                    }
                }
            }
        }
    }
//    printf("after_resampled_clusters.size() %d = ", resampled_clusters.size());
    printf("ok5\n");
    lm::Road<float> road_resampled(resampled_clusters);

    printf("ok6\n");

    {// add line beside curb
        if(add_right){
//                printf("@@@@  add right ok3\n");
            road_resampled.push_right(dynamic_cast<lm::Line<float>*>(road_resampled.rightline()), dis_right_curb, TYPE_SOLID);
        }
        if(add_left){
            printf("!!!!!add left line!!!!\n");
            road_resampled.push_left(dynamic_cast<lm::Line<float>*>(road_resampled.leftline()), dis_left_curb, TYPE_SOLID);
        }
    }
    bool mit_yelloesolid = false;
    {//linetype
        auto lines = road_resampled.lines();
        for (auto& line : lines){
            cv::Mat line_mask = cv::Mat::zeros(solid_mask.rows, solid_mask.cols, CV_8UC1);
            cv::Point2f front_point, back_point;
            image_align.revert(line->points().front(), front_point);
            image_align.revert(line->points().back(), back_point);
            cv::line(line_mask, front_point, back_point, 1, 25);
            int solid_count  = cv::countNonZero(solid_mask.mul(line_mask));
            int dashed_count = cv::countNonZero(dashed_mask.mul(line_mask));
            int yellow_count = cv::countNonZero(yellow_mask.mul(line_mask));
            int white_count = cv::countNonZero(white_mask.mul(line_mask));

            line->linetype = static_cast<LineType>(
                             static_cast<int>((dashed_count > solid_count * 1.5 && dashed_count > 200) ? TYPE_DASHED : TYPE_SOLID) +
                             static_cast<int>((yellow_count > white_count && yellow_count > 200) ? TYPE_YELLOW : TYPE_WHITE));
            if(line == lines[0]){//right
                if(right_curb_size > CURB_POINTS_THRESH){
                    line->boundary_type = 2;
                    if(right_curb_size > left_curb_size){
                        line->boundary_confidence = 2;
                    }
                    else{
                        line->boundary_confidence = 1;
                    }
                }
            }

            if(line->linetype == TYPE_SOLID_YELLOW){
                    line->boundary_type = 1;
                    line->boundary_confidence = 2;
                    mit_yelloesolid = true;
                }
            if((!mit_yelloesolid) && line == lines.back()){//left
                if(left_curb_size > CURB_POINTS_THRESH){
                    line->boundary_type = 1;
                    if(left_curb_size > right_curb_size){
                        line->boundary_confidence = 2;
                    }
                    else{
                        line->boundary_confidence = 1;
                    }
                }
            }
        }
    }

    //append line
    {	float avg_lanewidth = 0;

        {//between too wide lines
            auto lines = road_resampled.lines();
            auto lanes = road_resampled.lanes();

            for (auto& lane : lanes){
                lane->lanewidth = dynamic_cast<lm::Line<float>*>(lane->right())->points().front().x
                                - dynamic_cast<lm::Line<float>*>(lane->left())->points().front().x;
                if (lane->lanewidth > LANEWIDTH * 1.8){
                    road_resampled.push_right(dynamic_cast<lm::Line<float>*>(lane->left()), lane->lanewidth / 2, TYPE_DASHED);
                    lane->lanewidth = lane->lanewidth / 2;
                }
				avg_lanewidth += lane->lanewidth; // calculate avg_lanewidth
            }

            if(lanes.size()) {
                avg_lanewidth = avg_lanewidth/lanes.size();
            }else{
                avg_lanewidth = LANEWIDTH;
            }
        }

        {//beside dashed line
            auto lines = road_resampled.lines();
            if(lines.size()){
                if(!IsSolid(lines.front()->linetype)){
                    PRINT("dashed line append");
                    road_resampled.push_right(lines.front(), avg_lanewidth);
                }
                if(!IsSolid(lines.back()->linetype)){
                    PRINT("dashed line append");
                    road_resampled.push_left(lines.back(), avg_lanewidth);
                }
            }
        }

        {//beside single solid line
            auto lines = road_resampled.lines();
            if(lines.size() == 1){
                auto line = lines.front();
                float line_pos = line->points().front().x;
                if(IsSolid(line->linetype) && (line_pos < car_left_pos)){//center.x = 1.23633765e+02 ->1.25
                    road_resampled.push_right(line, avg_lanewidth);
                }else if(IsSolid(line->linetype) && (line_pos > car_right_pos)){
                    road_resampled.push_left(line, avg_lanewidth);
                }
                else {
                    {
                        int key=0;
                        cv::Mat road_aligned_image = aligned_image.clone();
                        visualize::visualize(road_resampled, road_aligned_image, CENTER);
                        cv::Mat result_image;
                        image_align.revert(road_aligned_image, result_image);
                        DEPLOY_IMSHOW(result_image, key);
                        IMSHOW(result_image);
                        if (key == 27) exit(0);
                    }
                    road_resampled.clear();
                    //PROCESS_FAILED(-5); //single line, failed
                }
            }
            else if(lines.size() > 1){
                auto line = lines.front();
                float line_pos = line->points().front().x;
                if(IsSolid(line->linetype) && (line_pos < car_left_pos)){
                    road_resampled.push_right(line, avg_lanewidth);
                }
            }
        }

        {//if roadmark beyond road edge, append line
            auto lines = road_resampled.lines();
            float extend_pixel = 10;
            if(lines.size()){
                auto line = lines.front();
                std::vector<cv::Point> points;
                {
                    cv::Point2f left_front_point, left_back_point;
                    image_align.revert(cv::Point2f(line->points().front().x, line->points().front().y), left_front_point);
                    float tmp_y = line->points().back().y - extend_pixel > 0 ?  line->points().back().y - extend_pixel : line->points().back().y;
                    image_align.revert(cv::Point2f(line->points().back().x, tmp_y), left_back_point);

                    cv::Point2f right_front_point(solid_mask.cols, left_front_point.y);
                    cv::Point2f right_back_point(solid_mask.cols, tmp_y);
                    points.push_back(left_front_point);
                    points.push_back(left_back_point);
                    points.push_back(right_back_point);
                    points.push_back(right_front_point);
                }

                cv::Mat lane_mask = cv::Mat::zeros(solid_mask.rows, solid_mask.cols, CV_8UC1);
                cv::fillConvexPoly(lane_mask, points, 1);
                int lanemark_count = cv::countNonZero(lane_mask.mul(arrow_mask));

                if(lanemark_count > min_lanemark_count){
                    road_resampled.push_right(line, avg_lanewidth);
                    PRINT(avg_lanewidth)
                }
            }

            if(lines.size()){
                auto line = lines.back();
                std::vector<cv::Point> points;
                {
                    cv::Point2f right_front_point, right_back_point;
                    image_align.revert(cv::Point2f(line->points().front().x, line->points().front().y), right_front_point);
                    float tmp_y =line->points().back().y - extend_pixel > 0 ?  line->points().back().y - extend_pixel : line->points().back().y;
                    image_align.revert(cv::Point2f(line->points().back().x, tmp_y), right_back_point);

                    cv::Point2f left_front_point(0, right_front_point.y);
                    cv::Point2f left_back_point(0, tmp_y);
                    points.push_back(left_front_point);
                    points.push_back(left_back_point);
                    points.push_back(right_back_point);
                    points.push_back(right_front_point);
                }

                cv::Mat lane_mask = cv::Mat::zeros(solid_mask.rows, solid_mask.cols, CV_8UC1);
                cv::fillConvexPoly(lane_mask, points, 1);
                int lanemark_count = cv::countNonZero(lane_mask.mul(arrow_mask));

                if(lanemark_count > min_lanemark_count){
                    road_resampled.push_left(line, avg_lanewidth);
                }
            }
        }

    }

    //NODE NUM COMPLETE!!

    // road_marktype
    int stopline_sum_x = 0, stopline_sum_y = 0, stop_count = 0;
    std::vector<cv::Point2f> stop_points;
    bool stopline_detected = false;
    {
        auto lanes = road_resampled.lanes();
        auto lines = road_resampled.lines();
        float avg_stopline_y = 0.0f;
        int count_stopline_y = 0;
        bool met_yellow_line = false;
        bool has_any_dashed = false;
        for (auto& line : lines){
            if (!IsSolid(line->linetype)){
                has_any_dashed = true;
                break;
            }
        }
        for (auto it_lane = lanes.begin(); it_lane != lanes.end(); ++it_lane) {
            lm::Lane<float>* lane = *it_lane;
            lm::Line<float>* left_line = dynamic_cast<lm::Line<float>*>(lane->left());
            lm::Line<float>* right_line = dynamic_cast<lm::Line<float>*>(lane->right());
            float extend_pixel = 0;
            std::vector<cv::Point> points;
            {
                cv::Point2f left_front_point, right_front_point, left_back_point, right_back_point;
                image_align.revert(left_line->points().front(), left_front_point);
                image_align.revert(right_line->points().front(), right_front_point);
                float tmp_y;
                tmp_y = left_line->points().back().y - extend_pixel > 0? left_line->points().back().y - extend_pixel : left_line->points().back().y;
                image_align.revert(cv::Point2f(left_line->points().front().x, tmp_y), left_back_point);
                tmp_y = right_line->points().back().y - extend_pixel > 0? right_line->points().back().y - extend_pixel : right_line->points().back().y;
                image_align.revert(cv::Point2f(right_line->points().front().x, tmp_y), right_back_point);
                points.push_back(left_front_point);
                points.push_back(left_back_point);
                points.push_back(right_back_point);
                points.push_back(right_front_point);
            }

            cv::Mat lane_mask = cv::Mat::zeros(solid_mask.rows, solid_mask.cols, CV_8UC1);
            cv::fillConvexPoly(lane_mask, points, 1);

            std::vector<MarkType> typelist= {TYPE_STRAIGHT, TYPE_LEFT, TYPE_RIGHT, TYPE_UTURN, TYPE_STRAIGHT_LEFT, TYPE_STRAIGHT_RIGHT};
            int lanemark_count[6];
            lanemark_count[0] = cv::countNonZero(arrow_thru_mask.mul(lane_mask));
            lanemark_count[1] = cv::countNonZero(arrow_left_mask.mul(lane_mask));
            lanemark_count[2] = cv::countNonZero(arrow_right_mask.mul(lane_mask));
            lanemark_count[3] = cv::countNonZero(arrow_uturn_mask.mul(lane_mask));
            lanemark_count[4] = cv::countNonZero(arrow_thru_left_mask.mul(lane_mask));
            lanemark_count[5] = cv::countNonZero(arrow_thru_right_mask.mul(lane_mask));

			/* modify lanemark_count_thres
            for (int i=0; i< 6; ++i) {
                PRINT(i);
                PRINT(lanemark_count[i]);
            }
			*/

            float count_scaler[6];
            count_scaler[0] = (float)lanemark_count[0] / 600;
            count_scaler[1] = (float)lanemark_count[1] / 1300;
            count_scaler[2] = (float)lanemark_count[2] / 1000;
            count_scaler[3] = (float)lanemark_count[3] / 1000;
            count_scaler[4] = (float)lanemark_count[4] / 1500;
            count_scaler[5] = (float)lanemark_count[5] / 1500;

            int8_t lane_type = 0x00;//TYPE_NONE
            float min_scaler = 1;
            for(int i=0; i<6; i++){
                if(count_scaler[i] > min_scaler) {
                    min_scaler = count_scaler[i];
                    lane_type = typelist[i];
                }
            }

            lm::Mark mark(MarkType(lane_type), cv::Point2f(0, 0));
            lane->m_marks.push_back(mark);
            if(!has_any_dashed){//calculate stop_point for each lane ----- origin image
                if(IsYellow(dynamic_cast<lm::Line<float>*>(lane->right())->linetype)){
                    met_yellow_line = true;
                }
                if (!met_yellow_line) {
                    cv::Mat stopline_eachlane_mask = lane_mask.mul(stopline_mask);
                    unsigned char *ptr = (unsigned char *) stopline_eachlane_mask.data;
                    for (int y = 0; y < stopline_eachlane_mask.rows - 30; ++y) {
                        for (int x = 0; x < stopline_eachlane_mask.cols; ++x, ++ptr) {
                            if (*ptr == 0) continue;
                            stopline_sum_x += x;
                            stopline_sum_y += y;
                            ++stop_count;
                            stop_points.push_back(cv::Point2f(x, y));
                        }
                    }
                }
            }// end calculate stopline point
        }// end for each lane, detect mark type

        //PRINT(stop_count);

//        if (count_stopline_y) {
//            stopline_detected = true;
//            avg_stopline_y = avg_stopline_y / count_stopline_y;
//            int stopline_idx = y_bias - avg_stopline_y;
//            PRINT(avg_stopline_y);
//            PRINT(stopline_idx);
//            if(avg_stopline_y > 0){
//                auto lines = road_resampled.lines();
//                for(lm::Line<float>* line : lines) {
//                    std::vector<cv::Point2f>& points = line->points();
//                    PRINT(points.size());
//                    points.erase(points.begin() + stopline_idx+50, points.end());
//                }
//                //min_y_filtered = MIN_Y_LOWPASS_RATE * min_y_filtered + (1 - MIN_Y_LOWPASS_RATE) * (avg_stopline_y - y_bias);
//                min_y_filtered = (avg_stopline_y - y_bias);
//            }
//        }
    }

    {//mark opposite direction lines
        auto lanes = road_resampled.lanes();
        bool met_yellow_line = false;
        for (auto it_lane = lanes.begin(); it_lane != lanes.end(); ++it_lane) {
            auto lane = *it_lane;
            if(IsYellow(dynamic_cast<lm::Line<float>*>(lane->right())->linetype)){
                met_yellow_line = true;
            }
            lane->opposite_direction = met_yellow_line;
        }

    }
    if (stop_count > min_stop_count) {
        stopline_detected = true;
    }
    {// update state machine
        IntersectionMode::transition(image_align_ok_raw, stopline_detected);
        PRINT(IntersectionMode::mode);
        if(IntersectionMode::mode == IntersectionMode::INTERSECTION /*&& !image_align.ok()*/){
            filters_poly.clear();
            road_resampled.clear();
            //PROCESS_FAILED(-10);
        }
    }

    //calculate distance from front axis center to lines
    {

        auto lines = road_resampled.lines();
        for(auto& line : lines){
            line-> distance = (line->points().front().x - aligned_center.x) * pixel2real_ratio;
        }
    }

    //struct Road_reverted
    {
        auto lines = road_resampled.lines();
        for(auto& line : lines){
            cluster::Cluster_<float> revert;
            image_align.revert(line->points(), revert);
            line->change_points(revert);
        }
    }

    // struct zcm message
    int32_t valid_lane_size = 0; // TODO: seems every lane is valid
    static long pub_time = 0;
    if(road_resampled.lanes().size()){
        out_list.push_back(1);
        if(!zcm_udp.good()){
            out_list.push_back(0);
            return out_list;
        }
        out_list.push_back(1);
        structRoadMarkingList structlanes_zcm;
        structlanes_zcm.current_lane_id = road_resampled.cal_current_lane_id(CENTER.x);
        StopLine stopline_zcm;//constract stopline zcm
        {
            if (stop_count > min_stop_count) {
                stopline_zcm.exist = true;
                stopline_zcm.num = stop_count;
                for(int i=0; i<stop_count; i++){
                    LinePoint p;
                    cv::Point2f converted_stop_point = convert_to_lidar(stop_points[i]);
                    p.x = converted_stop_point.x;
                    p.y = converted_stop_point.y;
                    stopline_zcm.stop_points.push_back(p);
                }
                int mean_y = stopline_sum_y / stop_count;
                stopline_zcm.distance = CENTER.y - mean_y;
            } else {
                stopline_zcm.exist = false;
                stopline_zcm.num = 0;
                stopline_zcm.distance = -10;
            }
//            printf("stop_count:____%d", stop_count);
        }
        structlanes_zcm.stop_line = stopline_zcm;
        if(left_curb_size > CURB_POINTS_THRESH || right_curb_size > CURB_POINTS_THRESH || mit_yelloesolid)
            structlanes_zcm.boundary_detected = true;//
        else
            structlanes_zcm.boundary_detected = false;//
        auto lanes = road_resampled.lanes();

        for (auto& lane : lanes){
            if(!lane->opposite_direction){
                Lane lane_zcm;
                lane_zcm.lane_type = lane->m_marks.back().m_type;
                lane_zcm.width = lane->lanewidth;
//                if(lane->stop_point.y > 0){
//                    cv::Point2f converted_stop_point = convert_to_lidar(lane->stop_point);
//                    lane_zcm.stop_point.x = converted_stop_point.x;
//                    lane_zcm.stop_point.y = converted_stop_point.y;
//                }
//                else{
//                    lane_zcm.stop_point.x = -1;
//                    lane_zcm.stop_point.y = -1;
//                }
//                PRINT(lane_zcm.stop_point.y)
                struct_laneline(lane_zcm.left_line, dynamic_cast<lm::Line<float>*>(lane->left()), solid_mask.cols, solid_mask.rows);
                struct_laneline(lane_zcm.right_line, dynamic_cast<lm::Line<float>*>(lane->right()), solid_mask.cols, solid_mask.rows);
                structlanes_zcm.lanes.push_back(lane_zcm);
                valid_lane_size++;
            }
        }

        structlanes_zcm.num = valid_lane_size;
        zcm_udp.publish("LANE_info", &structlanes_zcm);
        long dur = clock() - pub_time;
        printf("pub_time = %ld", dur);
        pub_time = clock();
        {//creat dict return to python
            out_list.push_back(int(structlanes_zcm.num));
            for(int i=0; i<int(structlanes_zcm.num); i++){
                out_list.push_back(int(structlanes_zcm.lanes[i].lane_type));
            }
            for(int i=0; i<int(structlanes_zcm.num); i++){
                if(i==0){
                    out_list.push_back(int(structlanes_zcm.lanes[i].right_line.line_type));
                }
                out_list.push_back(int(structlanes_zcm.lanes[i].left_line.line_type));
            }
            out_list.push_back(int(structlanes_zcm.stop_line.exist));
            out_list.push_back(int(structlanes_zcm.boundary_detected));
        }
    }
    else{
        out_list.push_back(0);
    }
    //SHOW
    {
        int key=0;
        cv::Mat result_image = image;//.clone();
        visualize::visualize(road_resampled, result_image, CENTER);
        DEPLOY_IMSHOW(result_image, key);
        IMSHOW(result_image);
        if (key == 27) exit(0);
    }

//    if (DEBUG) {
//      cv::waitKey(0);
//    }

    return out_list;
}



#define DEFINE(func) m.def(#func, &func, "")

PYBIND11_MODULE(postproc_lane, m) {
    DEFINE(process_tensor);
}
