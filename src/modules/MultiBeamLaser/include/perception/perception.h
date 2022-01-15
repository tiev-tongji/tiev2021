#ifndef DGC_PERCEPTION_H
#define DGC_PERCEPTION_H

#define USE_GRID_SEGMENTER
#define MULTITHREAD

#ifndef USE_GRID_SEGMENTER
#define USE_LASER_SEGMENTER
#else
#undef USE_LASER_SEGMENTER
#endif

#undef USE_LASER_SEGMENTER

#include <roadrunner.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <mutex>
#include <thread>
#include "grid.h"
#include <limits.h>
#include <string>
#include <passat_constants.h>
#include <stdint.h>
#include <tr1/memory>
#include "applanix_history.h"
#include "obstacle.h"
#include "velocore.h"
#include "tracked_obstacle.h"
#include "second_receiver.h"
#include <perception_messages.h>
#include "perception_defines.h"
#include "perception_types.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <zcm/zcm-cpp.hpp>
#include "msg/include/POSITION.hpp"
#include "msg/include/BOUNDINGBOX.hpp"
#include "msg/include/OBJECT.hpp"
#include "msg/include/structOBJECTLIST.hpp"
#include "msg/include/structLASERMAP.hpp"
#include "msg/include/structNAVINFO.hpp"
#include "msg/include/structSLAMCONTROL.hpp"
#include "msg/include/point3d.hpp"
#include "msg/include/structPointCloud.hpp"
#include "common/nature.h"

using namespace cv;
using namespace std;
using namespace TiEV;

static string Directory = "/home/autolab/tiev/src/modules/MultiBeamLaser/";

namespace TiEV
{
    class Handler
    {
    public:
        structNAVINFO navinfo;
        structSLAMCONTROL mySlamControl;
        std::mutex navinfoMutex, slamControlMutex;

        void handleNavinfoMessage(const zcm::ReceiveBuffer *rbuf, const std::string &chan, const structNAVINFO *msg)
        {
            navinfoMutex.lock();
            navinfo = *msg;
            navinfoMutex.unlock();
        }
        void handleSlamControlMessage(const zcm::ReceiveBuffer *rbuf, const std::string &chan, const structSLAMCONTROL *msg)
        {
            slamControlMutex.lock();
            mySlamControl = *msg;
            slamControlMutex.unlock();
        }
    };

    class MyZcm
    {
    public:
        zcm::ZCM ipc_sub{"ipc"};
        zcm::ZCM lasermap_pub{"ipc"};
        zcm::ZCM msg_sub{"udpm://239.255.76.67:7667?ttl=1"};
        zcm::ZCM pointcloud_pub{"ipc"};
        zcm::ZCM objectlists_pub{"udpm://239.255.76.67:7667?ttl=1"};

        structLASERMAP myLaserMap;
        structOBJECTLIST myObjectList;
        structPointCloud myPointCloud;

        void sub_udpm_msgs();
        void sub_ipc_msgs();

        void pub_lasermap()
        {
            lasermap_pub.publish("LASERMAP", &myLaserMap);
            memset(&myLaserMap, 0, sizeof(myLaserMap));
        }

        void pub_cloud()
        {
            pointcloud_pub.publish("CLOUD", &myPointCloud);
            // memset(&myPointCloud, 0, sizeof(myPointCloud));
        }

        void pub_objectlists()
        {
            objectlists_pub.publish("OBJECTLIST", &myObjectList);
            myObjectList.obj.clear();
            myObjectList.count = 0;
        }
    };

    class MyVisualization
    {
    public:
        Mat image1 = Mat::zeros(TiEV::GRID_ROW, TiEV::GRID_COL, CV_8UC3);
        string window = "GridMap";

        bool inGrid(int row, int col)
        {
            if (row >= 0 && row < TiEV::GRID_ROW && col >= 0 && col < TiEV::GRID_COL)
                return true;
            else
                return false;
        }

        void getRowCol(const point2d_t &pp, int &row, int &col)
        {
            row = TiEV::CAR_CEN_ROW - (int)(pp.y / TiEV::GRID_RESOLUTION);
            col = TiEV::CAR_CEN_COL + (int)(pp.x / TiEV::GRID_RESOLUTION);
        }

        void drawText(const point2d_t &pp, char text[])
        {
            int r, c;
            getRowCol(pp, r, c);
            if (inGrid(r, c))
            {
                CvPoint textPos = cvPoint(c, r);
                putText(image1, text, textPos, CV_FONT_HERSHEY_PLAIN, 1.0, Scalar(0, 0, 255), 0.1);
            }
        }

        void drawCircle(const point2d_t &pp, Scalar sca, int radis)
        {
            int r, c;
            getRowCol(pp, r, c);
            if (inGrid(r, c))
            {
                circle(image1, cv::Point(c, r), radis, sca);
            }
        }

        void drawLine(const point2d_t &p1, const point2d_t &p2, Scalar sca)
        {
            int rows1, cols1, rows2, cols2;
            getRowCol(p1, rows1, cols1);
            getRowCol(p2, rows2, cols2);

            if (inGrid(rows1, cols1) && inGrid(rows2, cols2))
            {
                int thickness = 1;
                int lineType = LINE_8;
                line(image1,
                     cv::Point(cols1, rows1),
                     cv::Point(cols2, rows2),
                     sca,
                     thickness,
                     lineType);
            }
        }

        void show_image1()
        {
            namedWindow(window.c_str(), 0);
            imshow(window.c_str(), image1);
            waitKey(1);
        }
    };
}

extern MyVisualization myVisual;
extern MyZcm myzcm;
extern bool sourcePointcloudEnable;
extern bool dynamicObjectsEnable;
extern structNAVINFO latestNavInfo;
extern Handler handler;
extern vector<point3d> sourceGridCloud;
extern bool gridCloudFlag;
extern mutex gridCloudMutex;

extern double velodyne_ts;
extern dgc_grid_p grid;
extern dgc_perception_map_cell_p default_map_cell;
extern dgc_perception_map_cell_p default_terrain_cell;
extern short default_z_cell;
extern grid_stat_t grid_stat;

extern dgc_grid_p terrain_grid;
extern dgc_grid_p z_grid;

extern perception_settings_t settings;

extern laser_scan_p lscan;
extern laser_scan_p nscan;

extern dgc_perception_map_cells_p obstacles_s;

extern std::vector<std::tr1::shared_ptr<TiEV::TrackedObstacle>> obstacles_tracked;
extern std::vector<std::tr1::shared_ptr<TiEV::Obstacle>> obstacles_second;

extern std::vector<dgc_perception_map_region_p> regions;

extern unsigned short counter;

/************************************************************************
   *
   * FUNCTIONS
   *
   ************************************************************************/

void perception_init(void);

void velodyne_init(dgc_velodyne_data_p velodyne);

void integrate_sensors(dgc_velodyne_data_p velo);

// void pythonSecond();

void integrate_velodyne(dgc_velodyne_data_p velodyne, unsigned short counter);

void integrate_velodyne_new(dgc_velodyne_data_p velodyne_temp, dgc_grid_p grid);

unsigned short counter_diff(unsigned short last, unsigned short now);

ApplanixPose *applanix_history_pose(applanix_elem_p elem);
applanix_elem_p applanix_history_elem(applanix_history_p history);
applanix_elem_p applanix_history_next(applanix_elem_p elem);
applanix_elem_p applanix_history_prev(applanix_elem_p elem);

ApplanixPose *applanix_current_pose(void);
ApplanixPose *applanix_pose(double timestamp);
void applanix_history_init(int size);
void applanix_history_add(ApplanixPose *msg);

void set_cell_min(dgc_perception_map_cell_p cell, float z, unsigned short counter);
void set_cell_max(dgc_perception_map_cell_p cell, float z, unsigned short counter);

void perception_track_frame(double timestamp);

void dgc_transform_integrate_pose(dgc_transform_t t, dgc_pose_t pose);

double pose_dist(ApplanixPose *p1, ApplanixPose *p2);

void display_time(char *label, double time);

#endif
