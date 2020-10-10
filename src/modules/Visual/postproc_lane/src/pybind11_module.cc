#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <Eigen/Dense>

#include <unistd.h>
#include "zcmmsg/structLANES.hpp"

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

cv::Point2f convert_to_lidar(const cv::Point2f& point_bev){
    cv::Point2f point_lidar;
    cv::Mat point_bev_ = (cv::Mat_<double>(3, 1)<< point_bev.x, point_bev.y, 1);
    static cv::Mat homography = (cv::Mat_<double>(3, 3) <<
                                4.49541030e-02,  2.78755872e-04, -5.85124919e+00,
                                3.22243481e-03, -4.76127770e-02,  4.97163722e+01,
                               -2.08830189e-04, -4.86239754e-05,  1.00000000e+00);
    cv::Mat point_lidar_ = homography * point_bev_;
    point_lidar.x = point_lidar_.at<double>(0, 0) / point_lidar_.at<double>(2, 0);
    point_lidar.y = point_lidar_.at<double>(1, 0) / point_lidar_.at<double>(2, 0);

    return point_lidar;
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
    laneline.num = line->points().size();
    fetch_lanelinepoints(line, laneline.points);
}

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

int process_tensor(py::array_t<uchar_t> _image, py::array_t<uchar_t> _predict) {

    float avg_lanewidth = 0;
    // init inputs
    cv::Mat image = convertToCvMat<3>(_image);
    std::vector<cv::Mat> predict = splitToCvMat(_predict);

    // static status
    static size_t frame_cnt;
    // MOT Kalman Filter for lines
    const float THRES_MATCH_DISTANCE_PIX = 15;
    const float THRES_DETECTED_NUM = 8;
    const float THRES_MISSED_NUM = 3;
    static FilterGroup<Polynomial> filters_poly(
        THRES_MATCH_DISTANCE_PIX, THRES_DETECTED_NUM, THRES_MISSED_NUM);
    const std::vector<cluster::Cluster_<float> > empty_clusters;
    // Low Pass Filter for yaw rotating angle
    static double yaw_angle = 0.0;
    static float y_bias = static_cast<float>(image.rows);
    const double YAW_LOWPASS_RATE = 0.7;
    static float min_y_filtered = 0.0f; //-y_bias;
    const float MIN_Y_LOWPASS_RATE = 0.8f;

    const cv::Point2f CENTER(1.25e+02, 1.05254885e+03f);
    const float VEHICLE_HALF_WIDTH = 0.65f / pixel2real_ratio;

    ++frame_cnt;
    PRINT("=======================================");
    PRINT(frame_cnt);

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
    PRINT(min_y_filtered);
    if (min_y_updated) {
        min_y_filtered = MIN_Y_LOWPASS_RATE * min_y_filtered + (1 - MIN_Y_LOWPASS_RATE) * min_y;
    }
    PRINT(min_y);
    PRINT(min_y_filtered);
    PRINT(observations.size());
    filters_poly.update(observations);
    std::vector<Survived<Polynomial> > survived_polys = filters_poly.get_survived_targets();
    PRINT(survived_polys.size());
    std::vector<cluster::Cluster_<float> > filtered_clusters;
    for(Survived<Polynomial>& survived_poly : survived_polys) {
        PRINT(survived_poly);
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

    //{
    //    auto lines = road.lines();
    //    if(lines.size() == 0) return 0; // PROCESS_FAILED(-6); //nothing
    //}

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
                                if (line_left_pos > car_left_pos && line_left_pos < car_right_pos){                    road.del_line(dynamic_cast<lm::Line<float>*>((*p_lane)->left()));}
                                if (line_right_pos > car_left_pos && line_right_pos < car_right_pos){                                   road.del_line(dynamic_cast<lm::Line<float>*>((*p_lane)->right()));}
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

    lm::Road<float> road_resampled(resampled_clusters);


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
        }
    }

    //append line
    {
        {//between too wide lines
            auto lines = road_resampled.lines();
            auto lanes = road_resampled.lanes();
            //int valid_lanesize = 0;
            for (auto& lane : lanes){
                lane->lanewidth = dynamic_cast<lm::Line<float>*>(lane->right())->points().front().x
                                - dynamic_cast<lm::Line<float>*>(lane->left())->points().front().x;

                avg_lanewidth += lane->lanewidth; // calculate avg_lanewidth

                //if(lane->lanewidth > (0.6 * LANEWIDTH)){
                //    avg_lanewidth += lane->lanewidth; // calculate avg_lanewidth
                //    valid_lanesize++;
                //}
                //else{lane->IsValid = 0;} //TODO: program seems never going to run this line

                if (lane->lanewidth > LANEWIDTH * 1.8){
                    road_resampled.push_right(dynamic_cast<lm::Line<float>*>(lane->left()), lane->lanewidth / 2, TYPE_DASHED);
                    lane->lanewidth = lane->lanewidth / 2;
                }
            }
            // if(valid_lanesize > 0) avg_lanewidth = (avg_lanewidth/valid_lanesize) > 60 ? (avg_lanewidth/valid_lanesize) : LANEWIDTH;
            //else avg_lanewidth = LANEWIDTH;
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
                }else if(IsSolid(line->linetype) && (line_pos > car_right_pos)){//center.x = 1.23633765e+02
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
                }//TODO: move!!!!!!!!!!!!!!!!!!!!!!11
            }
            else if(lines.size() > 1){
                auto line = lines.front();
                float line_pos = line->points().front().x;
                if(IsSolid(line->linetype) && (line_pos < car_left_pos)){//center.x = 1.23633765e+02 ->1.25
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


    //NODE NUM COMPLETE
    //{
    //    auto lines = road_resampled.lines();
    //    if(lines.size()<2) assert(0); // PROCESS_FAILED(-7);
    //}

    // road_marktype
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

            //for (int i=0; i< 6; ++i) {
            //    PRINT(i);
            //    PRINT(lanemark_count[i]);
            //}

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
                cv::Mat stopline_eachlane_mask = lane_mask.mul(stopline_mask);

                unsigned char * ptr = (unsigned char *)stopline_eachlane_mask.data;
                int sum_x = 0, sum_y = 0, stop_count = 0;
                for(int y = 0; y < stopline_eachlane_mask.rows - 30; ++y){
                    for(int x = 0; x < stopline_eachlane_mask.cols; ++x, ++ptr){
                        if (*ptr == 0) continue;
                        sum_x += x; sum_y += y; ++stop_count;
                    }
                }
                PRINT(stop_count);
                if (stop_count > min_stop_count) {
                    int mean_x = sum_x / stop_count; int mean_y = sum_y / stop_count;
                    lane->stop_point = cv::Point2f(mean_x, mean_y);
                    if(!met_yellow_line){// remove the points further than stopline
                        cv::Point2f stop_point;
                        assert(0 == image_align.align(lane->stop_point, stop_point));
                        PRINT(lane->stop_point);
                        PRINT(stop_point.y);
                        avg_stopline_y += stop_point.y;
                        ++count_stopline_y;
                    }

                }
                else{
                    lane->stop_point.x = -1;
                    lane->stop_point.y = -1;
                }
            }// end calculate stopline point
        }// end for each lane, detect mark type
        if (count_stopline_y) {
            stopline_detected = true;
            avg_stopline_y = avg_stopline_y / count_stopline_y;
            int stopline_idx = y_bias - avg_stopline_y;
            PRINT(avg_stopline_y);
            PRINT(stopline_idx);
            if(avg_stopline_y > 0){
                auto lines = road_resampled.lines();
                for(lm::Line<float>* line : lines) {
                    std::vector<cv::Point2f>& points = line->points();
                    PRINT(points.size());
                    points.erase(points.begin() + stopline_idx, points.end());
                }
                //min_y_filtered = MIN_Y_LOWPASS_RATE * min_y_filtered + (1 - MIN_Y_LOWPASS_RATE) * (avg_stopline_y - y_bias);
                min_y_filtered = (avg_stopline_y - y_bias);
            }
        }
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
    if(road_resampled.lanes().size()){
        if(!zcm_udp.good())
            return 0;
        structLANES structlanes_zcm;
        structlanes_zcm.current_lane_id = road_resampled.cal_current_lane_id(CENTER.x);

        auto lanes = road_resampled.lanes();

        for (auto& lane : lanes){
            if(!lane->opposite_direction){
                LANE lane_zcm;
                lane_zcm.lane_type = lane->m_marks.back().m_type;
                lane_zcm.width = lane->lanewidth;
                if(lane->stop_point.y > 0){
                    cv::Point2f converted_stop_point = convert_to_lidar(lane->stop_point);
                    lane_zcm.stop_point.x = converted_stop_point.x;
                    lane_zcm.stop_point.y = converted_stop_point.y;
                }
                else{
                    lane_zcm.stop_point.x = -1;
                    lane_zcm.stop_point.y = -1;
                }
                PRINT(lane_zcm.stop_point.y)
                struct_laneline(lane_zcm.left_line, dynamic_cast<lm::Line<float>*>(lane->left()), solid_mask.cols, solid_mask.rows);
                struct_laneline(lane_zcm.right_line, dynamic_cast<lm::Line<float>*>(lane->right()), solid_mask.cols, solid_mask.rows);
                structlanes_zcm.lanes.push_back(lane_zcm);
                valid_lane_size++;
            }
        }
        structlanes_zcm.num = valid_lane_size;
        //structlanes_zcm.num = lanes.size();
        //publish
        zcm_udp.publish("LANE_info", &structlanes_zcm);
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

    //return road_resampled.lanes().size();
    return valid_lane_size;
}



#define DEFINE(func) m.def(#func, &func, "")

PYBIND11_MODULE(postproc_lane, m) {
    DEFINE(process_tensor);
}
