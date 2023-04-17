//
// Created by xlz on 17-9-27.
// modified by xlz on 19-9-27 for indoor slam
// Created by john on 20-9-27 for adding vlp16 and cleaning.
//

#include "node.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/sensor/point_cloud.h"
#include "map_offset.h"
#include "common/nature.h"

using namespace cartographer::transform;
using ::cartographer::transform::Rigid3d;

//zz
#include "structFUSIONMAP.hpp"
#include "structSICKMAP.hpp"
#include "structLASERMAP.hpp"
#include "structPointCloud.hpp"
#include "structSLAMLOC.hpp"
#include "structLUXMAP.hpp"
#include "structVLPMAP.hpp"
#include "structNAVINFO.hpp"
#include "structSLAMCONTROL.hpp"


int64_t laser_map_num=0, laser_map_num_old=0;
int frame_count = 0;
ParameterReader pd("../parameters.txt");
bool UpdateMapFlag = atoi(pd.getData("UpdateMapFlag").c_str());
bool debug_show_map = atoi(pd.getData("debug_show_map").c_str());

float ths = atof( pd.getData("ths").c_str() );
float diku_utmx = atof( pd.getData("diku_utmx").c_str() );
float diku_utmy = atof( pd.getData("diku_utmy").c_str() );

float IMU_LASER_DISTANCE =0.83;

zcm::ZCM pub_udpm{"udpm://239.255.76.67:7667?ttl=1"};
zcm::ZCM pub_ipc{"ipc"};

/*draw obstacle rect*/
void gridCoor(double x, double y, int &row, int &col)
{
    row = TiEV::CAR_CEN_ROW - (int)(y / TiEV::GRID_RESOLUTION);
    col = TiEV::CAR_CEN_COL + (int)(x / TiEV::GRID_RESOLUTION);
}

bool inGrid(int row, int col)
{
    if (row >= 0 && row < TiEV::GRID_ROW && col >= 0 && col < TiEV::GRID_COL)
        return true;
    else 
        return false;
}

void drawLine(double x1, double y1, double x2, double y2, Scalar sca, cv::Mat image)
{
    int rows1, cols1, rows2, cols2;
    gridCoor(x1, y1, rows1, cols1);
    gridCoor(x2, y2, rows2, cols2);

    if (inGrid(rows1, cols1) && inGrid(rows2, cols2))
    {
        int thickness = 1;
        int lineType = 8;
        line( image,
            cv::Point(cols1, rows1),
            cv::Point(cols2, rows2),
            sca,
            thickness,
            lineType );
    }
}
/*draw obstacle rect*/

namespace TiEV{
    bool bStopMapping = false;
    int odom_num = 0;

    structLASERMAP LASER_temp;
    bool bRecvLASERMAP = false;

    structLUXMAP LUX_temp;
    bool bRecvLUXMAP = false;

    structVLPMAP VLP_temp;
    bool bRecvVLPMAP = false;

    structSICKMAP SICK_temp;
    bool bRecvSICKMAP = false;

    structNAVINFO NAVI_temp;
    bool bRecvNAVINFO = false;

    structPointCloud PointCloud_temp;
    bool bRecvPointCloud = false;

    structSLAMCONTROL SLAMCONTROL_cur,SLAMCONTROL_old;

    mutex LASER_mtx;
    mutex SICK_mtx;
    mutex LUX_mtx;
    mutex VLP_mtx;
    mutex PointCloud_mtx;
    mutex NAVI_mtx;
    mutex FUSION_mtx;

    class Handler
    {
    public:
        ~Handler() {}

        void handleMessage_odom(const zcm::ReceiveBuffer* rbuf,
                                const std::string& chan,
                                const structNAVINFO* msg)
        {
            NAVI_mtx.lock();
            memcpy(&NAVI_temp, msg, sizeof(*msg));
            bRecvNAVINFO = true;
            NAVI_mtx.unlock();
        }
        void handleMessage_scan(const zcm::ReceiveBuffer* rbuf,
                                const std::string& chan,
                                const structLASERMAP* msg)
        {
            LASER_mtx.lock();
            memcpy(&LASER_temp, msg, sizeof(*msg));
            bRecvLASERMAP = true;
            laser_map_num ++ ;
            LASER_mtx.unlock();
        }

        void handleMessage_SickMap(const zcm::ReceiveBuffer* rbuf,
                                   const std::string& chan,
                                   const structSICKMAP* msg)
        {
            SICK_mtx.lock();
            memcpy(&SICK_temp, msg, sizeof(*msg));
            bRecvSICKMAP = true;
            SICK_mtx.unlock();
        }

       void handleMessage_LuxMap(const zcm::ReceiveBuffer* rbuf,
                                 const std::string& chan,
                                 const structLUXMAP* msg)
       {
           LUX_mtx.lock();
           memcpy(&LUX_temp, msg, sizeof(*msg));
           bRecvLUXMAP = true;
           LUX_mtx.unlock();
       }
        
       void handleMessage_VlpMap(const zcm::ReceiveBuffer* rbuf,
                                 const std::string& chan,
                                 const structVLPMAP* msg)
       {
           VLP_mtx.lock();
           memcpy(&VLP_temp, msg, sizeof(*msg));
           bRecvVLPMAP = true;
           VLP_mtx.unlock();
       }
        // no longer needed, marked in Multibeam
        // void handleMessage_ObjList(const zcm::ReceiveBuffer* rbuf,
        //                            const std::string& chan,
        //                            const structOBJECTLIST* msg)
        // {
        //     OBJLIST_mtx.lock();
        //     OBJLIST_temp = *msg;
        //     bRecvOBJLIST = true;
        //     OBJLIST_mtx.unlock();
        // }

        void handleMessage_PointCloud(const zcm::ReceiveBuffer* rbuf,
                                   const std::string& chan,
                                   const structPointCloud* msg)
        {
            PointCloud_mtx.lock();
            memcpy(&PointCloud_temp, msg, sizeof(*msg));
            frame_count++;
            bRecvPointCloud = true;
            PointCloud_mtx.unlock();
        }

        //TODO:待测试效果 及 bool转变方向
        void handleMessage_SLAMCONTROL(const zcm::ReceiveBuffer* rbuf,
                                   const std::string& chan,
                                   const structSLAMCONTROL* msg)
        {
            memcpy(&SLAMCONTROL_cur, msg, sizeof(*msg));
            if(bStopMapping == false && SLAMCONTROL_cur.mapping == true)
            {
                if ( SLAMCONTROL_old.mapping == false)
                {
                    bStopMapping = true;//模式切换 停掉第一个above ground模式
                }
            }
            SLAMCONTROL_old = SLAMCONTROL_cur;
        }
};

//for zcm
void zcm_ipc_func()
{
    zcm::ZCM zcm_ipc{"ipc"};

    if (!zcm_ipc.good())
    {
        cout << "zcm init ipc error!" << endl;
        return;
    }

    Handler handlerObject;
    // zcm_ipc.subscribe("LASERMAP", &Handler::handleMessage_scan, &handlerObject);
    // zcm_ipc.subscribe("SICKMAP", &Handler::handleMessage_SickMap, &handlerObject);
    zcm_ipc.subscribe("CLOUD", &Handler::handleMessage_PointCloud, &handlerObject);
    zcm_ipc.subscribe("LUXMAP", &Handler::handleMessage_LuxMap, &handlerObject);
    zcm_ipc.subscribe("VLPMAP", &Handler::handleMessage_VlpMap, &handlerObject);
    // while (0 == zcm_ipc.handle() );
    zcm_ipc.run();

}

void zcm_udpm_func()
{
    zcm::ZCM zcm_udpm{"udpm://239.255.76.67:7667?ttl=1"};

    if (!zcm_udpm.good())
    {
        cout << "zcm init udpm error!" << endl;
        return;
    }

    Handler handlerObject;
    zcm_udpm.subscribe("LASERMAP", &Handler::handleMessage_scan, &handlerObject);
    zcm_udpm.subscribe("SICKMAP", &Handler::handleMessage_SickMap, &handlerObject);
    zcm_udpm.subscribe("NAVINFO", &Handler::handleMessage_odom, &handlerObject);
    //zcm_udpm.subscribe("OBJECTLIST", &Handler::handleMessage_ObjList, &handlerObject);
    //TODO：待确定消息
    zcm_udpm.subscribe("SLAMCONTROL", &Handler::handleMessage_SLAMCONTROL, &handlerObject);
    zcm_udpm.run();    
}

void key_record()
{
    while(true)
    {
        cv::Mat captureMat(200, 200, CV_8UC1);
        cv::namedWindow("keycapture=s");
        cv::imshow("keycapture=s", captureMat);
        char ch = cv::waitKey(1);
        //char ch;
        if(ch == 's')
        {

            cout<< "=-=======================================================bStopMapping is modified: "<<ch <<endl;
            bStopMapping = true;
        }
        //sleep(0.01);
        usleep(10000);
    }
}

perception_Node::perception_Node(const NodeOptions& node_options , int mode)
    : node_options_(node_options),
    map_builder_bridge_(node_options_) {
        cartographer::common::MutexLocker lock(&mutex_);

        for (int i = 0; i < 3; ++i)
        {
            initial_tran[i] = 0.0;
            initial_rot[i] = 0.0;
        }
        initial_rot[3] = 0.0;
        bLoadFrmMap = false;
        bInitialPosSet = false;
        bGlobalLoc = false;
        bInitialPosSet_cur_map = false;

        if ( mode == 1)
        {
            mode_ = MODE::ABOVE_GROUND;
        }
        if ( mode == 2)
        {
            SLAMCONTROL_old.mapping = true;
            mode_ = MODE::UNDERGROUND_MAPPING;
        }
        if ( mode == 3)
        {
            SLAMCONTROL_old.mapping = true;
            mode_ = MODE::UNDERGROUND_LOCATION;
        }

        if (!pub_udpm.good())
        {
            cout<<"zcm for publishing udpm is not good";
            return;
        }

        if (!pub_ipc.good())
        {
            cout<<"zcm for publishing ipc is not good";
            return;
        }
        //be careful
        std::thread Zcm_ipc_thread(zcm_ipc_func);
        Zcm_ipc_thread.detach();
        std::thread Zcm_udpm_thread(zcm_udpm_func);
        Zcm_udpm_thread.detach();

        if (mode == 1)
        {
            std::thread Pub_thread(genFUSIONMap, (void *)this);
            Pub_thread.detach();
        }

        if (mode == 3)
        {
            getchar();
            std::thread Pub_thread(genSLAMLocation, (void *)this);
            Pub_thread.detach();
        }

        if(!debug_show_map)
        {
            std::thread tr2(key_record);
            tr2.detach();
        }

    bStopMapping = false;
    }

perception_Node::~perception_Node() {}

void perception_Node::AddExtrapolator(const int trajectory_id,
                           const TrajectoryOptions& options) {
    constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms
    CHECK(extrapolators_.count(trajectory_id) == 0);
    const double gravity_time_constant =
            node_options_.map_builder_options.use_trajectory_builder_3d()
            ? options.trajectory_builder_options.trajectory_builder_3d_options()
                    .imu_gravity_time_constant()
            : options.trajectory_builder_options.trajectory_builder_2d_options()
                    .imu_gravity_time_constant();
    extrapolators_.emplace(
            std::piecewise_construct, std::forward_as_tuple(trajectory_id),
            std::forward_as_tuple(
                    ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
                    gravity_time_constant));
}

void perception_Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
    //cartographer::common::MutexLocker lock(&mutex_);
    CHECK(ValidateTrajectoryOptions(options));
    //AddTrajectory  expected_sensor_ids set here
    const std::unordered_set<string> expected_sensor_ids =
            ComputeExpectedTopics(options);

    cout << "check out -> Need input data :  ";
    for(const auto& ite : expected_sensor_ids)
        std::cout << ite << " ; ";
    cout << endl;

    int trajectory_id = AddTrajectory(options,expected_sensor_ids);
    Add_data(trajectory_id);
}

std::unordered_set<string> perception_Node::ComputeExpectedTopics(
        const TrajectoryOptions& options) {
    std::unordered_set<string> expected_topics;
    if (options.num_laser_scans > 0) {
        expected_topics.insert("scan");
    }

    if (options.num_multi_echo_laser_scans > 0) {
        expected_topics.insert("echoes");
    }

    if (options.num_point_clouds > 0) {
        expected_topics.insert("points2");
    }

    // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
    // required.
    if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
        (node_options_.map_builder_options.use_trajectory_builder_2d() &&
         options.trajectory_builder_options.trajectory_builder_2d_options()
                 .use_imu_data())) {
        expected_topics.insert("imu");
    }
    // Odometry is optional.
    if (options.use_odometry) {
        expected_topics.insert("odom");
    }
    return expected_topics;
}


int perception_Node::AddTrajectory(const TrajectoryOptions& options,
    const std::unordered_set<string> expected_sensor_ids) {

    const int trajectory_id =
    map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
    AddExtrapolator(trajectory_id, options);
    is_active_trajectory_[trajectory_id] = true;

    return trajectory_id;
}

//add odometry and Range data in on funtion?? may have to be separated JOHN 2017.10
void perception_Node::Add_data(int trajectory_id)
{
    cout<<"bStopMapping"<<bStopMapping<<endl;
    while(!bStopMapping)
    {
        //cout << " MODE" << mode_ <<endl;
        //读取gps的oc建图模式
        if ( mode_ == MODE::ABOVE_GROUND )
        {
            if(!bRecvLASERMAP)
            {
                usleep(10000);
                continue;
            }
            if(laser_map_num == laser_map_num_old)
                continue;

            gps_odom local_odom;
            local_odom.header.frame_id = "odom";
            local_odom.header.seq = odom_num;
            int64_t timestamp_temp = LASER_temp.timestamp/pow(10,6);
            local_odom.header.stamp.sec = timestamp_temp;
            long double time_all = LASER_temp.timestamp;
            long double nsec = (time_all / ( long double )pow(10,6) - timestamp_temp) * ( long double )pow(10,9);
            local_odom.header.stamp.nsec = nsec;

            local_odom.pose.position.x = LASER_temp.utmX - (CAR_WHEEL_BASE -IMU_LASER_DISTANCE ) * cos(LASER_temp.mHeading);
            local_odom.pose.position.y = LASER_temp.utmY - (CAR_WHEEL_BASE -IMU_LASER_DISTANCE ) * sin(LASER_temp.mHeading);
            local_odom.pose.position.z = 0;

            local_odom.pose.orientation = createQuaternionMsgFromYaw(LASER_temp.mHeading);
            double local_mheading = LASER_temp.mHeading;
            //set inital pose
            if (!bLoadFrmMap && !bInitialPosSet)
            {
                initial_tran[0] = local_odom.pose.position.x;
                initial_tran[1] = local_odom.pose.position.y;
                initial_tran[2] = local_odom.pose.position.z;
                initial_rot[0] = local_odom.pose.orientation.x;
                initial_rot[1] = local_odom.pose.orientation.y;
                initial_rot[2] = local_odom.pose.orientation.z;
                initial_rot[3] = local_odom.pose.orientation.w;
                initial_yaw = local_mheading;
                bInitialPosSet = true;
            }

            //generate and add virtualscan
            std::vector<Eigen::Vector3f> PointCloud_from_scan;
            LASER_mtx.lock();
            structLASERMAP local_LASERMAP = LASER_temp;
            memcpy(local_LASERMAP.cells, LASER_temp.cells, GRID_ROW * GRID_COL * sizeof(uint8_t));
            //memset(LASER_temp.cells,0,GRID_ROW*GRID_COL* sizeof(uint8_t));
            LASER_mtx.unlock();

            //global utm to local coordinates
            local_odom.pose.position.x -= initial_tran[0];//local
            local_odom.pose.position.y -= initial_tran[1];//local
            if (map_builder_bridge_.GetTrajectoryOption()[trajectory_id].use_odometry == true)
            {
                if(UpdateMapFlag)
                    Add_Odom_Data("odom",local_odom,trajectory_id);
                odom_num ++;
            }

            TiEV::generate_virtualscans(local_LASERMAP, PointCloud_from_scan);
            if(UpdateMapFlag)
            {
                Add_Scan_Data("scan",PointCloud_from_scan, local_odom.header.stamp.sec,local_odom.header.stamp.nsec,trajectory_id);
            }
            laser_map_num_old ++ ;
        } else{
            //只读取雷达的scanmatch建图模式
            if(!bRecvPointCloud)
            {
                usleep(10000);
                continue;
            }

            structNAVINFO currentNavi;
            NAVI_mtx.lock();
            currentNavi = NAVI_temp;
            NAVI_mtx.unlock();

            if (mode_ == 2)
            {
                if (!bLoadFrmMap && !bInitialPosSet)
                {
                    initial_tran[0] = currentNavi.utmX;
                    initial_tran[1] = currentNavi.utmY;
                    initial_tran[2] = 0;
                    initial_yaw = currentNavi.mHeading;
                    bInitialPosSet = true;
                    cout << "first origin utmX: " << initial_tran[0] <<" utmY" <<  initial_tran[1] << endl;
                }
            } else
            {
                if (!bInitialPosSet_cur_map)
                {
                    initial_tran_cur_map[0] = currentNavi.utmX;
                    initial_tran_cur_map[1] = currentNavi.utmY;
                    initial_tran_cur_map[2] = 0;
                    initial_yaw_cur_map = currentNavi.mHeading;
                    bInitialPosSet_cur_map = true;
                    cout << "second origin utmX: " << initial_tran_cur_map[0] <<" utmY" <<  initial_tran_cur_map[1] << endl;
                }
            }

            PointCloud_mtx.lock();
            structPointCloud local_PointCloud = PointCloud_temp;
            PointCloud_mtx.unlock();

            vector<Eigen::Vector3f> myCloud;
            Eigen::Vector3f point_(frame_count,frame_count,frame_count);
            myCloud.push_back(point_);

            //pointcloud move to front-axis
            double dy = CAR_WHEEL_BASE -IMU_LASER_DISTANCE;  //here compensates the translation from LiDAR to front axle.
            for( int m = 0; m < local_PointCloud.total; m++)
            {
                Eigen::Vector3f point_tmp(local_PointCloud.frame[m].px_,
                                          local_PointCloud.frame[m].py_ - dy,
                                          local_PointCloud.frame[m].pz_);
                myCloud.push_back(point_tmp);
            }
            int64_t sec = local_PointCloud.timestamp/pow(10,6);
            long double time_all = local_PointCloud.timestamp;
            long double nsec = (time_all / ( long double )pow(10,6) - sec) * ( long double )pow(10,9);

            Add_PointCloud_Data("points2",myCloud, sec,nsec,trajectory_id);
        }
    }
    bStopMapping = false;
}

void * perception_Node::genFUSIONMap(void* __this)
{
    perception_Node* _this = (perception_Node*) __this;
    while (true) {
        structNAVINFO currentNavi;
        NAVI_mtx.lock();
        currentNavi = NAVI_temp;
        NAVI_mtx.unlock();

        structFUSIONMAP fusionmap;
        fusionmap.utmX = currentNavi.utmX ;
        fusionmap.utmY = currentNavi.utmY;
        fusionmap.mHeading = currentNavi.mHeading;
        fusionmap.rows = GRID_ROW;
        fusionmap.cols = GRID_COL;
        fusionmap.center_row = CAR_CEN_ROW;
        fusionmap.center_col = CAR_CEN_COL;
        fusionmap.resolution = GRID_RESOLUTION;
        fusionmap.timestamp = TiEV::getTimeStamp();
        memset(fusionmap.cells, 0, sizeof(uint8_t) * GRID_ROW * GRID_COL);
        pos FusionMap_pos(fusionmap.utmX, fusionmap.utmY, fusionmap.mHeading);
        //
        structSICKMAP currentSick;
        pos Sick_pos(0, 0, 0);
        if (bRecvSICKMAP) {
            SICK_mtx.lock();
            currentSick = SICK_temp;
//            array_trans(currentSick.cells, SICK_temp.cells, 0.93);
            SICK_mtx.unlock();
            Sick_pos = pos(currentSick.utmX, currentSick.utmY, currentSick.mHeading);
        }

       structLUXMAP currentLux;
       pos Lux_pos(0, 0, 0);
       if (bRecvLUXMAP) {
           LUX_mtx.lock();
           currentLux = LUX_temp;
           //array_trans(currentLux.cells, LUX_temp.cells, 0.93);
           LUX_mtx.unlock();
           Lux_pos = pos(currentLux.utmX, currentLux.utmY, currentLux.mHeading);
       }

       structVLPMAP currentVlp;
       pos Vlp_pos(0, 0, 0);
       if (bRecvVLPMAP) {
           VLP_mtx.lock();
           currentVlp = VLP_temp;
           //array_trans(currentLux.cells, VLP_temp.cells, 0.93);
           VLP_mtx.unlock();
           Vlp_pos = pos(currentVlp.utmX, currentVlp.utmY, currentVlp.mHeading);
       }

        structLASERMAP currentLaser;
        pos Laser_pos(0, 0, 0);
        if (bRecvLASERMAP) {
            LASER_mtx.lock();
            //currentLaser = LASER_temp;
            currentLaser.timestamp = LASER_temp.timestamp;
            currentLaser.utmX = LASER_temp.utmX;
            currentLaser.utmY = LASER_temp.utmY;
            currentLaser.mHeading = LASER_temp.mHeading;
            array_trans(currentLaser.cells, LASER_temp.cells, IMU_LASER_DISTANCE - CAR_WHEEL_BASE);//IMU_LASER_DISTANCE - CAR_WHEEL_BASE
            LASER_mtx.unlock();
            Laser_pos = pos(currentLaser.utmX, currentLaser.utmY, currentLaser.mHeading);
        }

        cv::Mat SickMap_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
	cv::Mat LuxMap_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
        cv::Mat VlpMap_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
        cv::Mat LaserMap_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
        cv::Mat OBJMAP_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));

	//Generate temp maps for messages to be fused
        for (int m = 0; m < GRID_ROW; m++) {
            for (int n = 0; n < GRID_COL; n++) {
                if (bRecvSICKMAP && currentSick.cells[m][n] == 0) {
                    SickMap_temp.ptr<uchar>(m)[n] = 255;
                }
                if (bRecvLUXMAP && currentLux.cells[m][n] != 0) {
                    LuxMap_temp.ptr<uchar>(m)[n] = 255;
                }
                if (bRecvVLPMAP && currentVlp.cells[m][n] != 0) {
                    VlpMap_temp.ptr<uchar>(m)[n] = 255;
		}
                if (bRecvLASERMAP && currentLaser.cells[m][n] != 0) {
                    // LaserMap_temp.ptr<uchar>(m)[n] = 255;
                    //1 static 2 dynamic, marked in MultibeamLaser
                   if(currentLaser.cells[m][n] == 1)
                   {
                       LaserMap_temp.ptr<uchar>(m)[n] = 255;
                   } else{
                       OBJMAP_temp.ptr<uchar>(m)[n] = 255;
                   }
                }
            }
        }

        cv::Mat current_map = _this->GetImageByPose(fusionmap.utmX - _this->initial_tran[0],
                                                    fusionmap.utmY - _this->initial_tran[1], fusionmap.mHeading,
                                                    GRID_ROW, GRID_COL, CAR_CEN_ROW, CAR_CEN_COL, GRID_RESOLUTION,
                                                    14 , ths);

        //RKT not precise
        if(currentNavi.mRTKStatus == 0)
        {
            memset(current_map.data,0,GRID_ROW*GRID_COL* sizeof(uint8_t));
        }
        //for jiugongge
       if(fusionmap.utmX - _this->initial_tran[0] < 540 + 25 && 540 - 25 < fusionmap.utmX - _this->initial_tran[0]
         && fusionmap.utmY - _this->initial_tran[1] < 60 +60 && fusionmap.utmY - _this->initial_tran[1] > 60 -70)
        //for gaojia
        // if(fusionmap.utmY - _this->initial_tran[1] >2800
        //         ||fusionmap.utmY - _this->initial_tran[1]<500)
        {
           memset(current_map.data,0,GRID_ROW*GRID_COL* sizeof(uint8_t));
        }
      //for jiugongge traffic light ghost car 
       if(fusionmap.utmX  < 288878.307 + 10 && 288878.307 - 10 < fusionmap.utmX
         && fusionmap.utmY  < 3497489.420 + 10 && 3497489.420 - 10 < fusionmap.utmY)
        {
           memset(current_map.data,0,GRID_ROW*GRID_COL* sizeof(uint8_t));
        }
       //for jiugongge zhinengche dasha ghost parking car 
       if(fusionmap.utmX  < 288734.701 + 20 && 288734.701 - 20 < fusionmap.utmX
         && fusionmap.utmY  < 3497470.977 + 20 && 3497470.977 - 20 < fusionmap.utmY)
        {
           memset(current_map.data,0,GRID_ROW*GRID_COL* sizeof(uint8_t));
        }
        //for jiugongge under bridge 20201112 john
       if(fusionmap.utmX  < 289700.619 + 50 && 289700.619 - 50 < fusionmap.utmX
         && fusionmap.utmY  < 3497211.064 + 50 && 3497211.064 -50 < fusionmap.utmY ||
         fusionmap.utmX  < 289567.404 + 50 && 289567.404 - 50 < fusionmap.utmX
         && fusionmap.utmY  < 3497347.990 +50 && 3497347.990 -50 < fusionmap.utmY)
        {
           memset(current_map.data,0,GRID_ROW*GRID_COL* sizeof(uint8_t));
        }

       //fusion TODO JOHN
        if (bRecvSICKMAP) {
            //bit 3
            map_fusion(current_map, FusionMap_pos, SickMap_temp, Sick_pos, GRID_RESOLUTION, 0.93, 0x08);
            //TODO: testing
            // map_fusion_onestep(current_map, FusionMap_pos, SickMap_temp, Sick_pos, 0x08);
            // mapfusion_by_eigen(current_map, FusionMap_pos, SickMap_temp, Sick_pos, 0x08);
        }

        if (bRecvLUXMAP) {
            //bit 4
            map_fusion(current_map, FusionMap_pos, LuxMap_temp, Lux_pos, GRID_RESOLUTION, 0.93, 0x10);
        }

        if (bRecvVLPMAP) {
            //bit 5
            map_fusion(current_map, FusionMap_pos, VlpMap_temp, Vlp_pos, GRID_RESOLUTION, 0.93, 0x20);
        }

	//historical map --- 0x01
        for (int i = 0; i < current_map.rows; ++i) {
            for (int j = 0; j < current_map.cols; ++j) {
                if (current_map.ptr<uchar>(i)[j] > 0) {
                    /* code */
                    current_map.ptr<uchar>(i)[j] = 0x01;
                }
            }
        }


        if (bRecvLASERMAP) {
            //static obstacle --- 0x02
            map_fusion(current_map, FusionMap_pos, LaserMap_temp, Laser_pos, GRID_RESOLUTION,0.93,0x02); //d:sensor to imu distance
            // moving obstacle --- 0x04
            map_fusion(current_map, FusionMap_pos, OBJMAP_temp, Laser_pos, GRID_RESOLUTION,0.93, 0x04);
            //TODO: testing
            // map_fusion_onestep(current_map, FusionMap_pos, LaserMap_temp, Laser_pos, 0x02);
            // map_fusion_onestep(current_map, FusionMap_pos, OBJMAP_temp, Laser_pos, 0x04);
            // mapfusion_by_eigen(current_map, FusionMap_pos, LaserMap_temp, Laser_pos, 0x02);
            // mapfusion_by_eigen(current_map, FusionMap_pos, OBJMAP_temp, Laser_pos, 0x04);
        }

        if(debug_show_map)
        {
            debug_show(current_map, "current_map");
            debug_show(SickMap_temp, "SickMap_temp");
            debug_show(LuxMap_temp, "LuxMap_temp");
            debug_show(VlpMap_temp, "VlpMap_temp");
            debug_show(LaserMap_temp, "LaserMap_temp");

        }


        //Move to forward wheel center
//        int dx = 0;
//        int dy = CAR_WHEEL_DIST / TiEV::GRID_RESOLUTION;
//        cv::Mat map_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
//        translateTransform(current_map,map_temp,dx,dy);
//        memcpy(fusionmap.cells, map_temp.data, sizeof(uint8_t) * current_map.rows * current_map.cols);


        memcpy(fusionmap.cells, current_map.data, sizeof(uint8_t) * current_map.rows * current_map.cols);
       // memcpy(fusionmap.cells, currentLaser.cells, sizeof(uint8_t) * current_map.rows * current_map.cols);

        pub_udpm.publish("FUSIONMAP", &fusionmap);
        usleep(10*1000);
    }
}

Rigid3d pose2DTo3D(const Eigen::Vector3f &robotPose)
{
    Eigen::Vector3d translation(robotPose[0], robotPose[1], 0);
    Eigen::Quaterniond quaternion;
    quaternion = Eigen::AngleAxisd(robotPose[2], Eigen::Vector3d::UnitZ());
    Rigid3d robotpose3D(translation, quaternion);
    return robotpose3D;
}

double quaternion2yaw(Eigen::Quaterniond quaternion)
{
    Eigen::Vector3d euler_angles = quaternion.matrix().eulerAngles ( 0,1,2 ); // ZYX顺序，即roll pitch yaw顺序
    return euler_angles[2];
}

void * perception_Node::genSLAMLocation(void* __this){
    perception_Node* _this = (perception_Node*) __this;

    while(!_this->bLoadFrmMap && !_this->bInitialPosSet)
    {
        usleep(10000);
    }
    Eigen::Vector3f robotpose2D_map( _this->initial_tran[0], _this->initial_tran[1], _this->initial_yaw - M_PI_2);
    Rigid3d robotpose3D_map = pose2DTo3D(robotpose2D_map);

    while(!_this->bInitialPosSet_cur_map)
    {
        usleep(10000);
    }
    Eigen::Vector3f robotpose2D_cur_slam( _this->initial_tran_cur_map[0], _this->initial_tran_cur_map[1], _this->initial_yaw_cur_map - M_PI_2 );
    Rigid3d robotpose3D_cur_slam = pose2DTo3D(robotpose2D_cur_slam);
    cout << " coordinate setting finish! " << endl;

    while (true){
        Rigid3d global_pose = _this->PublishTrajectoryStates();
        Eigen::Quaterniond quaternion = global_pose.rotation();
        double heading = quaternion2yaw(quaternion);

        cout << setprecision(16) << "map_pose: ( "
                 << global_pose.translation().x() << ", " 
                 << global_pose.translation().y() << " ) yaw: " 
                 << (quaternion2yaw(global_pose.rotation())) * 180.0 / M_PI << endl;


        Rigid3d UTM_pose;
        //cout<< "global_pose : ( " << global_pose.translation().x() << ", " << global_pose.translation().y() << " ) yaw: " << heading <<endl;
        if( _this->bGlobalLoc == true)
        {
            UTM_pose = robotpose3D_map * global_pose;
            cout << setprecision(16) << "Global_localizaion OK: UTM_pose : ( "
                 << UTM_pose.translation().x() << ", " 
                 << UTM_pose.translation().y() << " ) yaw: " 
                 << (quaternion2yaw(UTM_pose.rotation()) + M_PI_2) * 180.0 / M_PI  <<endl;
        } else{
            UTM_pose = robotpose3D_cur_slam * global_pose;
            cout << setprecision(16) << "Global_localizaion not complete: UTM_pose : ( "
                 << UTM_pose.translation().x() << ", " 
                 << UTM_pose.translation().y() << " ) yaw: " 
                 << (quaternion2yaw(UTM_pose.rotation()) + M_PI_2) * 180.0 / M_PI  <<endl;
        }
        // TiEV::WGS84Coor wgs_coor = MapXYToLatLon(UTM_pose.translation().x(),UTM_pose.translation().y());
        structSLAMLOC location;
        location.timestamp = TiEV::getTimeStamp();
        location.x = UTM_pose.translation().x();
        location.y = UTM_pose.translation().y();
        location.mHeading = quaternion2yaw(UTM_pose.rotation()) + M_PI_2;
        location.SLAMStatus = _this->bGlobalLoc;
        pub_udpm.publish("SLAMLOC", &location);

        //todo:ARRIVE specific position and publish( dist or region)
        double dist = (location.x - diku_utmx)*(location.x - diku_utmx) + (location.y - diku_utmy)*(location.y - diku_utmy);
        if(sqrt(dist) < 4.0)
        {
            structSLAMCONTROL slamcontrol;
            slamcontrol.mapping = true;
            pub_ipc.publish("UNDERGROUND_CONTROL", &slamcontrol);
        }

        structFUSIONMAP fusionmap;
        fusionmap.utmX = location.x;
        fusionmap.utmY = location.y;
        fusionmap.mHeading = location.mHeading;
        fusionmap.rows = GRID_ROW;
        fusionmap.cols = GRID_COL;
        fusionmap.center_row = CAR_CEN_ROW;
        fusionmap.center_col = CAR_CEN_COL;
        fusionmap.resolution = GRID_RESOLUTION;
        fusionmap.timestamp = TiEV::getTimeStamp();
        memset(fusionmap.cells, 0, sizeof(uint8_t) * GRID_ROW * GRID_COL);

        structLASERMAP currentLaser;
        pos Laser_pos(0, 0, 0);
        if (bRecvLASERMAP) {
            LASER_mtx.lock();
            //currentLaser = LASER_temp;
            currentLaser.timestamp = LASER_temp.timestamp;
            currentLaser.utmX = LASER_temp.utmX;
            currentLaser.utmY = LASER_temp.utmY;
            currentLaser.mHeading = LASER_temp.mHeading;
            array_trans(currentLaser.cells, LASER_temp.cells, IMU_LASER_DISTANCE - CAR_WHEEL_BASE);//IMU_LASER_DISTANCE - CAR_WHEEL_BASE
            LASER_mtx.unlock();
        }

        structSICKMAP currentSick;
        pos Sick_pos(0, 0, 0);
        if (bRecvSICKMAP) {
            SICK_mtx.lock();
            currentSick = SICK_temp;
//            array_trans(currentSick.cells, SICK_temp.cells, 0.93);
            SICK_mtx.unlock();
        }

        for (int m = 0; m < GRID_ROW; m++) {
            for (int n = 0; n < GRID_COL; n++) {
                if (bRecvLASERMAP && currentLaser.cells[m][n] != 0) {
                    fusionmap.cells[m][n] = 0x02;
                }
                if (bRecvSICKMAP && currentSick.cells[m][n] != 0) {
                    fusionmap.cells[m][n] = 0x02;
                }
            }
        }
        pub_udpm.publish("FUSIONMAP", &fusionmap);

        if(debug_show_map)
        {
            //cv::Mat current_map = _this->GetImageByPose(global_pose.translation().x(), global_pose.translation().y(), heading, 300, 300, 150, 150, GRID_RESOLUTION, 7 , ths);
            //use the new map size 202011
            // TO be verified
            cv::Mat current_map = _this->GetImageByPose(global_pose.translation().x(), global_pose.translation().y(), heading, CAR_CEN_ROW, CAR_CEN_ROW, GRID_COL, GRID_COL, GRID_RESOLUTION, 7 , ths);
            for (int i = current_map.rows/2 - 5; i < current_map.rows/2 +5; ++i) {
                for (int j= current_map.cols/2 - 10; j < current_map.cols/2 + 10; ++j){
                    current_map.at<uchar>(i,j) = 255;
                }
            }
            cv::Mat rotate_map;
            double angle = -90;
            double scale = 1;
            Mat rot_mat(2, 3, CV_32FC1);
            //cv::Point2f center(src.cols / 2, src.rows / 2);
            cv::Point2f center(current_map.cols/2 ,current_map.rows/2 );
            rot_mat = getRotationMatrix2D(center, angle, scale);
            warpAffine(current_map, rotate_map, rot_mat, current_map.size());
            debug_show(rotate_map, "current_map");

            cv::Mat FUSION_map(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
            memcpy(FUSION_map.data, fusionmap.cells, sizeof(uint8_t) * GRID_ROW * GRID_COL);
            debug_show(FUSION_map, "FUSIONMAP");
        }

        usleep(10*1000);
    }
}

void perception_Node::FinishAllTrajectories() {
    cartographer::common::MutexLocker lock(&mutex_);
    for (auto& entry : is_active_trajectory_) {
        const int trajectory_id = entry.first;
        if (entry.second) {
            map_builder_bridge_.FinishTrajectory(trajectory_id);
            entry.second = false;
        }
    }
}

void perception_Node::FinishTrajectory(const int trajectory_id) {
    cartographer::common::MutexLocker lock(&mutex_);
    CHECK(is_active_trajectory_.at(trajectory_id));
    map_builder_bridge_.FinishTrajectory(trajectory_id);
    is_active_trajectory_[trajectory_id] = false;
}

void perception_Node::RunFinalOptimization(){
    {
        cartographer::common::MutexLocker lock(&mutex_);
        for (const auto& entry : is_active_trajectory_) {
            CHECK(!entry.second);
        }
    }
    // Assuming we are not adding new data anymore, the final optimization
    // can be performed without holding the mutex.
    map_builder_bridge_.RunFinalOptimization();
}

void perception_Node::SerializeState(const string& filename)
{
    cartographer::common::MutexLocker lock(&mutex_);
    map_builder_bridge_.SerializeState(filename);
}

void perception_Node::LoadMap(const std::string& map_filename) {
    cartographer::common::MutexLocker lock(&mutex_);
    map_builder_bridge_.LoadMap(map_filename);
}

bool perception_Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
    if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
        return options.trajectory_builder_options
        .has_trajectory_builder_2d_options();
    }
    if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
        return options.trajectory_builder_options
        .has_trajectory_builder_3d_options();
    }
    return false;
}

Rigid3d perception_Node::PublishTrajectoryStates() {
    ::cartographer::common::MutexLocker lock(&mutex_);
    for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
        const auto& trajectory_state = entry.second;

        auto& extrapolator = extrapolators_.at(entry.first);
        // We only publish a point cloud if it has changed. It is not needed at high
        // frequency, and republishing it would be computationally wasteful.
        if (trajectory_state.pose_estimate.time != extrapolator.GetLastPoseTime()) {
            //不发送配准点云
//            scan_matched_point_cloud_publisher_.publish(ToPointCloud2Message(
//                    carto::common::ToUniversal(trajectory_state.pose_estimate.time),
//                    node_options_.map_frame,
//                    carto::sensor::TransformPointCloud(
//                            trajectory_state.pose_estimate.point_cloud,
//                            trajectory_state.local_to_map.cast<float>())));
            extrapolator.AddPose(trajectory_state.pose_estimate.time,
                                 trajectory_state.pose_estimate.pose);
        }
        // If we do not publish a new point cloud, we still allow time of the
        // published poses to advance. If we already know a newer pose, we use its
        // time instead. Since tf knows how to interpolate, providing newer
        // information is better.

        //定位成功后 trajectory_state.local_to_map 有值    则tracking_to_map为全局坐标
        const ::cartographer::common::Time now = extrapolator.GetLastPoseTime();
        const Rigid3d tracking_to_local = extrapolator.ExtrapolatePose(now);
        const Rigid3d tracking_to_map =
                trajectory_state.local_to_map * tracking_to_local;

//        LOG(INFO) << "tracking_to_local:"<< tracking_to_local.DebugString() << std::endl;
//        LOG(INFO) << "tracking_to_map:"<< tracking_to_map.DebugString() << std::endl;
//        LOG(INFO) << "trajectory_state.local_to_map:"<< trajectory_state.local_to_map.DebugString() << std::endl;

        Rigid3d global_pose;
        if (trajectory_state.published_to_tracking != nullptr) {
            if (trajectory_state.trajectory_options.provide_odom_frame) {
                global_pose = tracking_to_local * (*trajectory_state.published_to_tracking);
            } else {
                global_pose = tracking_to_map * (*trajectory_state.published_to_tracking);
            }
        }

        if ( bGlobalLoc == false && abs(trajectory_state.local_to_map.translation().x())
             + abs(trajectory_state.local_to_map.translation().y())
             + abs(trajectory_state.local_to_map.translation().z()) >  0.5 )
        {
            bGlobalLoc = true;
        }

       // cout << setprecision(16) << "tracking_to_map: ( "
       //           << tracking_to_map.translation().x() << ", " 
       //           << tracking_to_map.translation().y() << " ) yaw: " 
       //           << (quaternion2yaw(tracking_to_map.rotation())) * 180.0 / M_PI  <<endl;

       // cout << setprecision(16) << "tracking_to_local: ( "
       //           << tracking_to_local.translation().x() << ", " 
       //           << tracking_to_local.translation().y() << " ) yaw: " 
       //           << (quaternion2yaw(tracking_to_local.rotation())) * 180.0 / M_PI  <<endl;

       //  cout << setprecision(16) << "local_to_map: ( "
       //           << trajectory_state.local_to_map.translation().x() << ", " 
       //           << trajectory_state.local_to_map.translation().y() << " ) yaw: " 
       //           << (quaternion2yaw(trajectory_state.local_to_map.rotation())) * 180.0 / M_PI  <<endl;
                    
        return tracking_to_map;
//        cout << global_pose.translation().x()
//                <<"," << global_pose.translation().y()
//                <<"," << global_pose.translation().z()
//                <<"," << global_pose.rotation().x()
//                <<"," << global_pose.rotation().y()
//                <<"," << global_pose.rotation().z()
//                <<"," << global_pose.rotation().w()
//                <<std::endl;
    }
}

void perception_Node::Add_Odom_Data(const string& sensor_id , const gps_odom& odom , int trajectory_id)
{
    cartographer::common::MutexLocker lock(&mutex_);
    auto msg_bridge_ptr = map_builder_bridge_.msg_bridge(trajectory_id);
    auto odometry_data_ptr = msg_bridge_ptr->ToOdometryData(odom);
    if (odometry_data_ptr != nullptr) {
        extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
    }
    map_builder_bridge_.msg_bridge(trajectory_id)
    ->HandleOdometryMessage("odom", odom);
}

void perception_Node::Add_Scan_Data( const string& sensor_id , const vector<Eigen::Vector3f>& msg ,const int64_t& sec,const int64_t& nsec , int trajectory_id )
{
    cartographer::common::MutexLocker lock(&mutex_);


    Time time_temp;
    time_temp.sec = sec;
    time_temp.nsec = nsec;

    map_builder_bridge_.msg_bridge(trajectory_id)
            ->HandleLaserScanMessage("scan", msg ,time_temp);
}

void perception_Node::Add_PointCloud_Data(const string& sensor_id , const vector<Eigen::Vector3f>& msg ,const int64_t& sec,const int64_t& nsec , int trajectory_id )
{
    cartographer::common::MutexLocker lock(&mutex_);

    Time time_temp;
    time_temp.sec = sec;
    time_temp.nsec = nsec;

    map_builder_bridge_.msg_bridge(trajectory_id)
    ->HandlePointCloudMessage("points2", msg ,time_temp);

}


cv::Mat perception_Node::GetImageByPose(double reckon_x , double reckon_y ,double yaw, int rows, int cols, int cen_row, int cen_col, float reso ,int search_num ,float threshold)
{
    cartographer::common::MutexLocker lock(&mutex_);
    return map_builder_bridge_.GetImageByPose(reckon_x , reckon_y , yaw,  rows,  cols,  cen_row,  cen_col, reso , search_num , threshold );
}
}
