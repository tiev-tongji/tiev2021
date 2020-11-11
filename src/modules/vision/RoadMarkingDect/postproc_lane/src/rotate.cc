#include "rotate.h"

template<typename T>
T median(std::vector<T> medi)
{
    std::sort(medi.begin(), medi.end());

    T tmedian;
    if (medi.size() == 0)
        throw std::runtime_error("rotate: try to find median from empty set.");
    else if (medi.size() % 2 == 0)           // even
        tmedian = (medi[medi.size() / 2 - 1] + medi[medi.size() / 2]) / 2;
    else                                // odd
        tmedian = medi[medi.size() / 2];

    return tmedian;
}

ImageAlign::ImageAlign(cv::Mat lane_mask, int single_side_widen_width) {
    {// init rows and cols
        this->rows = lane_mask.rows;
        this->cols = lane_mask.cols;
        this->single_side_widen_width = single_side_widen_width;
    }
    auto roi_rows = std::min(rows*3/4, rows - 256);
    cv::Mat roi_lane_mask = lane_mask(
            cv::Range(roi_rows, rows), cv::Range(0, cols));
    //todo continue shuli
    cv::Mat hough_buff;
    {//binarize
        const int THRESHOLD = 0;
        const int MAX_VALUE = 255;
        cv::threshold(roi_lane_mask, hough_buff,
                      THRESHOLD, MAX_VALUE, cv::THRESH_BINARY);
    }
    {//edge detection
        const double LOW_THRESHOLD = 50;
        const double HIGH_THRESHOLD = 200;
        const int KERNEL_SIZE = 3;
        const bool L2_GRADIENT = false;
        cv::Canny(hough_buff, hough_buff, LOW_THRESHOLD,
                  HIGH_THRESHOLD, KERNEL_SIZE, L2_GRADIENT);
    }
    std::vector<cv::Vec4i> lines;
    {//line hough detection
        const double RHO_RESOLUTION = 1;
        const double THETA_RESOLUTION = CV_PI / 180;
        const int THRESHOLD = 10;
        const double MIN_LINE_LENGTH = 30;
        const double MAX_LINE_GAP = 200;
        cv::HoughLinesP(hough_buff, lines, RHO_RESOLUTION,
                        THETA_RESOLUTION, THRESHOLD,
                        MIN_LINE_LENGTH, MAX_LINE_GAP);
    }
    PRINT(lines.size());
    {// calculate yaw angle
        const double MAX_VALID_YAW = 45. / 180. * CV_PI;
        std::vector<double> yaw_vec;
        for(cv::Vec4i& line : lines) {
            cv::Point p1(line[0], line[1]), p2(line[2], line[3]);
            cv::Point diffp = p1 - p2;
            if (abs(diffp.y) > 10) {
                double yaw = atan(- static_cast<double>(diffp.x) /
                                    static_cast<double>(diffp.y));
                yaw_vec.push_back(yaw);
            }
        }
        PRINT(yaw_vec.size());
        if (yaw_vec.size()){
            this->yaw = median<>(yaw_vec);
            PRINT(this->yaw);
            this->yaw_ok = fabs(this->yaw) < MAX_VALID_YAW;
        }else {
            this->yaw_ok = false;
        }
        PRINT(this->yaw_ok)
    }
    if(0) {// visualise: draw line on display
        cv::Mat detected_lines_view;
        cv::cvtColor(hough_buff, detected_lines_view, cv::COLOR_GRAY2BGR);
        for(size_t i = 0; i < lines.size(); i++)
        {
            const int LINE_THICKNESS = 2;
            const int LINE_TYPE = cv::LINE_8;
            cv::line(detected_lines_view,
                     cv::Point(lines[i][0], lines[i][1]),
                     cv::Point(lines[i][2], lines[i][3]),
                     cv::Scalar(0,0,255), LINE_THICKNESS, LINE_TYPE);
        }
        IMSHOW(detected_lines_view)
    }
    if(this->yaw_ok){// init affine_matrix
        this->set_yaw(yaw);
    }
}// ImageAlign

void ImageAlign::set_yaw(double p_yaw){
    this->yaw = p_yaw;
    this->yaw_ok = true;
    cv::Mat rotation_matrix = (cv::Mat_<double>(3, 3) <<
            cos(yaw),  sin(yaw), cols / 2 + single_side_widen_width,
           -sin(yaw),  cos(yaw), rows / 1,
            0       ,  0       , 1);
    cv::Mat translate_matrix = (cv::Mat_<double>(3, 3) <<
            1, 0, -cols / 2,
            0, 1, -rows / 1,
            0, 0, 1);
    cv::Mat affine_matrix= rotation_matrix * translate_matrix;
    cv::Mat affine_matrix_inv;
    cv::invert(affine_matrix, affine_matrix_inv, cv::DECOMP_LU);
    this->affine_matrix     = affine_matrix(cv::Rect(0,0,3,2));
    this->affine_matrix_inv = affine_matrix_inv(cv::Rect(0,0,3,2));
}

int ImageAlign::align(cv::InputArray in, cv::OutputArray out)
{
    if (in.rows() != rows || in.cols() != cols) {
        std::stringstream ss;
        ss << "input image size incorrect, expecting (" << rows << ',' << cols
            << "), got (" << in.rows() << ',' << in.cols() << ")\n";
        throw std::runtime_error(ss.str());
    }
    if (! yaw_ok) return -1;
    cv::warpAffine(in, out, affine_matrix, cv::Size(cols + single_side_widen_width * 2, rows), cv::INTER_LINEAR);
    return 0;
}// ImageAlign::align


int ImageAlign::revert(cv::InputArray in, cv::OutputArray out)
{
    if (in.rows() != rows || in.cols() != cols + single_side_widen_width * 2) {
        std::stringstream ss;
        ss << "input image size incorrect, expecting (" << rows << ',' << cols + single_side_widen_width * 2
            << "), got (" << in.rows() << ',' << in.cols() << ")\n";
        throw std::runtime_error(ss.str());
    }
    if (! yaw_ok) return -1;
    cv::warpAffine(in, out, affine_matrix_inv, cv::Size(cols, rows), cv::INTER_LINEAR);
    return 0;
}// ImageAlign::revert

int ImageAlign::align(const cluster::Cluster_<float> &in, cluster::Cluster_<float> &out)
{
    if (! yaw_ok) return -1;
    cv::Mat homo_pin, homo_pout;
    for (auto p_in = in.begin(); p_in != in.end(); ++p_in) {
        homo_pin = (cv::Mat_<double>(3, 1) << p_in->x, p_in->y, 1.0);
        homo_pout = affine_matrix * homo_pin;
        cv::Point2f pout(static_cast<float>(homo_pout.at<double>(0)),
                         static_cast<float>(homo_pout.at<double>(1)));
        out.push_back(pout);
    }
    return 0;
}// ImageAlign::align

int ImageAlign::revert(const cluster::Cluster_<float> &in, cluster::Cluster_<float> &out)
{
    if (! yaw_ok) return -1;
    cv::Mat homo_pin, homo_pout;
    for (auto p_in = in.begin(); p_in != in.end(); ++p_in) {
        homo_pin = (cv::Mat_<double>(3, 1) << p_in->x, p_in->y, 1.0);
        homo_pout = affine_matrix_inv * homo_pin;
        cv::Point2f pout(static_cast<float>(homo_pout.at<double>(0)),
                         static_cast<float>(homo_pout.at<double>(1)));
        out.push_back(pout);
    }
    return 0;
}// ImageAlign::revert

int ImageAlign::align(const cv::Point2f &in_point, cv::Point2f &out_point)
{ 
    cv::Mat before = (cv::Mat_<double>(3, 1) << in_point.x, in_point.y, 1);
    cv::Mat after = affine_matrix * before;
    out_point.x = static_cast<float>(after.at<double>(0, 0));
    out_point.y = static_cast<float>(after.at<double>(1, 0));
    return 0;
}// ImageAlign::align

int ImageAlign::revert(const cv::Point2f &in_point, cv::Point2f &out_point)
{ 
    cv::Mat before = (cv::Mat_<double>(3, 1) << in_point.x, in_point.y, 1);
    cv::Mat after = affine_matrix_inv * before;
    out_point.x = static_cast<float>(after.at<double>(0, 0));
    out_point.y = static_cast<float>(after.at<double>(1, 0));
    return 0;
}// ImageAlign::revert

