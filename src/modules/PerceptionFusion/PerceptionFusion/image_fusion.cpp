//
// Created by xlz on 17-10-21.
//

#include "image_fusion.h"
#include "common/nature.h"

namespace TiEV {
/*
* Merge two mats based on bit mask (in wiil be overlapped to in_out based on mask) added by John 2017.10
* both mat should be the same size
*/

    using namespace cv;
    using namespace TiEV;

    //can not call multiple times, X system will report error
    void debug_show(cv::Mat current_map, string name) {
        cv::namedWindow(name,0);
        cv::Mat show = current_map.clone();
        for (int i = 0; i < current_map.rows; ++i) {
            for (int j = 0; j < current_map.cols; ++j) {
                if (current_map.ptr<uchar>(i)[j] == 0x01) {
                    /* code */
                    show.ptr<uchar>(i)[j] = 255;
                }
                if (current_map.ptr<uchar>(i)[j] == 0x02) {
                    /* code */
                    show.ptr<uchar>(i)[j] = 240;
                }
                if (current_map.ptr<uchar>(i)[j] == 0x04) {
                    /* code */
                    show.ptr<uchar>(i)[j] = 200;
                }
                if (current_map.ptr<uchar>(i)[j] == 0x08) {
                    /* code */
                    show.ptr<uchar>(i)[j] = 150;
                }
            }
        }
        //cv::equalizeHist(current_map, current_map);
        cv::imshow(name, show);
        cv::waitKey(1);

    }

    void overlap_mat(Mat in_out, Mat &in, uint8_t mask) {
        if (in_out.rows != in.rows || in_out.cols != in.cols) {
            cout << __FILE__ << __LINE__ << "overlapping two mat with different size!!" << endl;
            return;
        }
        for (int i = 0; i < in_out.rows; ++i) {
            for (int j = 0; j < in_out.cols; ++j) {
                if (in.ptr<uchar>(i)[j] > 0) {
                    /* code */
                    in_out.ptr<uchar>(i)[j] = /*in_out.ptr<uchar>(i)[j] |*/ mask;
                }
            }
        }
    }

//TODO: MultiThread
    void map_fusion(Mat &src1_inout, pos pos1, Mat src2, pos pos2, float resolution,  float distance, uint8_t bitmask) //d:sensor to imu distance
    {
       // cout << "pos1.utmx = " << pos1.utmx << ' ' ;
       // cout << "pos1.utmy = " << pos1.utmy << ' ' ;
       // cout << "pos1.mheading = " << pos1.mHeading << endl; ;
       // cout << "pos2.utmy = " << pos2.utmy << ' ' ;
       // cout << "pos2.utmx = " << pos2.utmx << ' ' ;
       // cout << "pos2.mheading = " << pos2.mHeading << endl; ;

        //fixed size
        Mat src_temp1 = src1_inout.clone();
        Mat src_temp2 = src2.clone();

        //lux calibration
        // if(bitmask == 0x04)
        // {
        //     int dcol = round(0.2 / 0.2);
        //     int drow = -round(0.2 / 0.2);

        //     Mat trans_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
        //     translateTransform(src2, trans_temp, dcol, drow);

        //     Mat rotate_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));

        //     double delta_yaw = 1.5 / 180.0 *M_PI;
        //     rotate_temp = get_rotate_map(trans_temp, delta_yaw);

        //     src_temp2 = rotate_temp.clone();
        // }

        //sick calibration
        if(bitmask ==0x08)
        {
            int dcol = -round(0.2 / 0.2);
            int drow = -round(0.2 / 0.2);

            Mat trans_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
            translateTransform(src2, trans_temp, dcol, drow);

            Mat rotate_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));

            double delta_yaw = 0.2 / 180.0 *M_PI;
            rotate_temp = get_rotate_map(trans_temp, delta_yaw);

            src_temp2 = rotate_temp.clone();
        }

//        //velodyne64 calibration
//        if(bitmask ==0x02)
//        {
//            int dcol = 0;
//            int drow = 0;
//
//            Mat trans_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
//            translateTransform(src2, trans_temp, dcol, drow);
//
//            Mat rotate_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
//
//            double delta_yaw = -2.7 / 180.0 *M_PI;
//            rotate_temp = get_rotate_map(trans_temp, delta_yaw);
//
//            src_temp2 = rotate_temp.clone();
//        }

        //
        double delta_yaw = pos2.mHeading - pos1.mHeading;
        double delta_utmx = pos2.utmx - pos1.utmx ;
        double delta_utmy = pos2.utmy - pos1.utmy;
        // double delta_yaw = 0;
        // double delta_utmx = 0 ;
        // double delta_utmy = 0;
        Mat rotate_result(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
        //rotate_result = get_rotate_map_add_trans(src_temp2, delta_yaw,delta_utmx,delta_utmy , pos1.mHeading);


        // 先旋转 后平移
        //
        rotate_result = get_rotate_map(src_temp2, delta_yaw);
//        if (fabs(delta_yaw) > TiEV::deg2rad(TOL_ANGLE_DEG)) {
//            rotate_result = get_rotate_map(src_temp2, delta_yaw);
//            cout<<"++++ rotate"<<endl;
//        } else {
//            rotate_result = src_temp2;
//            cout<<"---no rotate"<<endl;
//        }
        //平移
        double local_deltax = delta_utmx;
        double local_deltay = delta_utmy;
        TiEV::rotate2d(local_deltax, local_deltay, TiEV::TiEV_PI / 2.0 -pos1.mHeading);//y become forward x become lateral

        int dcol = round (local_deltax/resolution);
        int drow = -round (local_deltay/resolution);
        Mat rotate_transform_result(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
        translateTransform(rotate_result, rotate_transform_result, dcol, drow);
//        if (delta_utmx * delta_utmx + delta_utmy * delta_utmy > TOL_DIST_METER * TOL_DIST_METER) {
//            translateTransform(rotate_result, rotate_transform_result, dcol, drow);
//            cout<<"++++ trans"<<endl;
//        } else {
//            rotate_transform_result = rotate_result;
//            cout<<"---no trans"<<endl;
//        }


        //叠加
        overlap_mat(src_temp1, rotate_transform_result, bitmask);
        src1_inout = src_temp1.clone();
    }

    void map_fusion_onestep(Mat &src1_inout ,  pos pos1,  Mat src2 ,  pos pos2, uint8_t mask){
        //fixed size
        Mat src_temp1 = src1_inout.clone();
        Mat src_temp2 = src2.clone();

        //sick calibration
        if(mask ==0x08)
        {
            int dcol = -round(0.2 / 0.2);
            int drow = -round(0.2 / 0.2);

            Mat trans_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
            translateTransform(src2, trans_temp, dcol, drow);

            Mat rotate_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));

            double delta_yaw = 0.2 / 180.0 *M_PI;
            rotate_temp = get_rotate_map(trans_temp, delta_yaw);

            src_temp2 = rotate_temp.clone();
        }

        //
        double delta_yaw = pos2.mHeading - pos1.mHeading;
        double delta_utmx = pos2.utmx - pos1.utmx ;
        double delta_utmy = pos2.utmy - pos1.utmy;

        Mat result(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
        //rotate_result = get_rotate_map_add_trans(src_temp2, delta_yaw,delta_utmx,delta_utmy , pos1.mHeading);

        double angle = TiEV::rad2deg(delta_yaw);
        double scale = 1;
        Mat rot_mat(2, 3, CV_32FC1);
        //cv::Point2f center(src.cols / 2, src.rows / 2);
        cv::Point2f center(CAR_CEN_COL ,CAR_CEN_ROW );
        rot_mat = getRotationMatrix2D(center, angle, scale);

        //平移
        double local_deltax = delta_utmx;
        double local_deltay = delta_utmy;
        TiEV::rotate2d(local_deltax, local_deltay, TiEV::TiEV_PI / 2.0 -pos1.mHeading);//y become forward x become lateral


        double dcol = local_deltax / TiEV::GRID_RESOLUTION;
        double drow = - local_deltay / TiEV::GRID_RESOLUTION;

        rot_mat.ptr<double>(0)[2] += dcol;
        rot_mat.ptr<double>(1)[2] += drow;

        // 输出
        cout << "--- trans_totoal by opencv --- : \n" << rot_mat << endl;

        warpAffine(src_temp2, result, rot_mat, src_temp2.size());

        for (int i = 0; i < result.rows; ++i) {
            for (int j = 0; j < result.cols; ++j) {
                if (result.ptr<uchar>(i)[j] > 0) {
                    src1_inout.ptr<uchar>(i)[j] = mask;
                }
            }
        }
    }

    Mat get_rotate_map(Mat &src, double &angle_rad) {
        Mat result(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
        double angle = TiEV::rad2deg(angle_rad);
        double scale = 1;
        Mat rot_mat(2, 3, CV_32FC1);
        //cv::Point2f center(src.cols / 2, src.rows / 2);
        cv::Point2f center(CAR_CEN_COL ,CAR_CEN_ROW );
        rot_mat = getRotationMatrix2D(center, angle, scale);

        warpAffine(src, result, rot_mat, src.size());

        return result;
    }

    void translateTransform(cv::Mat &src, cv::Mat &dst, int &dx, int &dy) {
        CV_Assert(src.depth() == CV_8U);
        int rows = src.rows;
        int cols = src.cols;
        dst.create(rows, cols, src.type());

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                //平移后坐标映射到原图像
                int x = j - dx;
                int y = i - dy;
                //保证映射后的坐标在原图像范围内
                if (x >= 0 && y >= 0 && x < cols && y < rows && src.ptr<uchar>(y)[x] != 0)
                    dst.ptr<uchar>(i)[j] = src.ptr<uchar>(y)[x];
            }
        }


    }

    void mapfusion_by_eigen(Mat &src1_inout ,  pos pos1,  Mat src2 ,  pos pos2, uint8_t mask)
    {
        //fixed size
        Mat src_temp1 = src1_inout.clone();
        Mat src_temp2 = src2.clone();

        //sick calibration
        if(mask ==0x08)
        {
            int dcol = -round(0.2 / 0.2);
            int drow = -round(0.2 / 0.2);

            Mat trans_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
            translateTransform(src2, trans_temp, dcol, drow);

            Mat rotate_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));

            double delta_yaw = 0.2 / 180.0 *M_PI;
            rotate_temp = get_rotate_map(trans_temp, delta_yaw);

            src_temp2 = rotate_temp.clone();
        }

        Eigen::Isometry3d lidar_pose;
        pos pos2_trans(pos2.utmx, pos2.utmy, pos2.mHeading - M_PI_2);
        pose2DTo3D(pos2_trans, lidar_pose);
        vector<Eigen::Vector3d> point3d;
        for (int i = 0; i < src_temp2.rows; ++i) {
            for (int j = 0; j < src_temp2.cols; ++j) {
                if (src_temp2.ptr<uchar>(i)[j] > 0) {
                    Eigen::Vector3d tmp;
                    float row_center = (float)i + 0.5;
                    float col_center = (float)j + 0.5;
                    pixel2point(i, j, tmp);
                    // pixel2point(row_center, col_center, tmp);
                    Eigen::Vector3d tmp_trans = lidar_pose * tmp;
                    point3d.push_back(tmp_trans);
                }
            }
        }

        Eigen::Isometry3d fusion_pose;
        pos pos1_trans(pos1.utmx, pos1.utmy, pos1.mHeading - M_PI_2);
        pose2DTo3D(pos1_trans, fusion_pose);
        Eigen::Isometry3d fusion_pose_inverse = fusion_pose.inverse();
        for (int i = 0; i < point3d.size(); ++i) {
            Eigen::Vector3d fusion_point = fusion_pose_inverse * point3d[i];
            int fusion_row, fusion_col;
            point2pixel(fusion_point, fusion_row, fusion_col);
            if( fusion_row >= 0 && fusion_row < TiEV::GRID_ROW
                && fusion_col >= 0 && fusion_col < TiEV::GRID_COL )
            {
                src_temp1.ptr<uchar>(fusion_row)[fusion_col] = mask;
            }
        }
        src1_inout = src_temp1.clone();
    }

    void mapfusion_by_eigen_one_stage(Mat &src1_inout ,  pos pos1,  Mat src2 ,  pos pos2, uint8_t mask)
    {
        //fixed size
        Mat src_temp1 = src1_inout.clone();
        Mat src_temp2 = src2.clone();

        //sick calibration
        if(mask ==0x08)
        {
            int dcol = -round(0.2 / 0.2);
            int drow = -round(0.2 / 0.2);

            Mat trans_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
            translateTransform(src2, trans_temp, dcol, drow);

            Mat rotate_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));

            double delta_yaw = 0.2 / 180.0 *M_PI;
            rotate_temp = get_rotate_map(trans_temp, delta_yaw);

            src_temp2 = rotate_temp.clone();
        }

        Eigen::Matrix3d pixel2point_matrix, point2pixel_matrix;
        pixel2point_matrix << 0, TiEV::GRID_RESOLUTION, -TiEV::GRID_RESOLUTION * TiEV::CAR_CEN_COL,
                -TiEV::GRID_RESOLUTION, 0, TiEV::GRID_RESOLUTION * TiEV::CAR_CEN_ROW,
                0, 0, 1;
        point2pixel_matrix = pixel2point_matrix.inverse();


        Eigen::Matrix3d lidar_pose;
        pos pos2_trans(pos2.utmx, pos2.utmy, pos2.mHeading - M_PI_2);
        pose2D_matrix(pos2_trans, lidar_pose);

        Eigen::Matrix3d fusion_pose;
        pos pos1_trans(pos1.utmx, pos1.utmy, pos1.mHeading - M_PI_2);
        pose2D_matrix(pos1_trans, fusion_pose);
        Eigen::Matrix3d fusion_pose_inverse = fusion_pose.inverse();

        Eigen::Matrix3d trans_totoal = point2pixel_matrix * fusion_pose_inverse * lidar_pose * pixel2point_matrix;
        // 输出
        cout << "--- trans_totoal by eigen --- : \n" << trans_totoal << endl;

        for (int i = 0; i < src_temp2.rows; ++i) {
            for (int j = 0; j < src_temp2.cols; ++j) {
                if (src_temp2.ptr<uchar>(i)[j] > 0) {

                    Eigen::Vector3d origin( i, j, 1 );
                    Eigen::Vector3d result;
                    result = trans_totoal * origin;
                    src1_inout.ptr<uchar>( (int)result[0] )[ (int)result[1] ] = mask;
                }
            }
        }
    }

    void pose2D_matrix(const pos &robotPose, Eigen::Matrix3d& trans)
    {
        Eigen::Isometry2d T = Eigen::Isometry2d::Identity();
        T.rotate( robotPose.mHeading );
        T.pretranslate( Eigen::Vector2d( robotPose.utmx, robotPose.utmy) );
        trans = T.matrix();
    };

    void pose2DTo3D(const pos &robotPose, Eigen::Isometry3d& trans)
    {
        Eigen::AngleAxisd rotation_vector( robotPose.mHeading, Eigen::Vector3d( 0, 0, 1 ) );
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.rotate( rotation_vector );
        T.pretranslate( Eigen::Vector3d( robotPose.utmx, robotPose.utmy, 0 ) );
        trans = T;
    }

    void point2pixel(const Eigen::Vector3d& pp, int &row, int &col)
    {
        row = TiEV::CAR_CEN_ROW - (int)(pp[1] / TiEV::GRID_RESOLUTION);
        col = TiEV::CAR_CEN_COL + (int)(pp[0] / TiEV::GRID_RESOLUTION);
    }

    void pixel2point(const float &row, const float &col, Eigen::Vector3d& pp)
    {
        pp[1] = (TiEV::CAR_CEN_ROW - row) *  TiEV::GRID_RESOLUTION;
        pp[0] = (col - TiEV::CAR_CEN_COL) *  TiEV::GRID_RESOLUTION;
        pp[2] = 0;
    }


}