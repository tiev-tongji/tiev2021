#include "perception.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <perception_types.h>
 #include <fstream>
#include <iomanip>
#include "tinystr.h"
#include "tinyxml.h"
#include "hdl64parse.h"

#include "rapidjson/rapidjson.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"


using namespace std;
using namespace TiEV;
using namespace rapidjson;

#define HDL_NUM_ROT_ANGLES 36000
#define HDL64_FIRE_NUM 64
#define DIS_LSB 0.2
#define ONEDEGREE_2_RAD 0.0174533
#define ENET_H_LEN 0

#define Deg2Rad 0.01745329251994329576923690768     // (PI / 180.0)
#define Rad2Deg 57.2957795130823208767981548141     // (180.0 / PI)

#define PERCEPTION_MAP_OBSTACLE_FREE         0
#define PERCEPTION_MAP_OBSTACLE_LOW          1
#define PERCEPTION_MAP_OBSTACLE_HIGH         2
#define PERCEPTION_MAP_OBSTACLE_DYNAMIC      3
#define PERCEPTION_MAP_OBSTACLE_STREET       4
#define PERCEPTION_MAP_OBSTACLE_UNKNOWN      255


dgc_perception_map_cells_p      obstacles_s;
std::vector<std::tr1::shared_ptr<Obstacle> >        obstacles_second;
std::vector<std::tr1::shared_ptr<TrackedObstacle> > obstacles_tracked;

double                          velodyne_ts          = 0;
dgc_velodyne_data_p             velodyne;
grid_stat_t                     grid_stat;
dgc_grid_p                      grid;
dgc_grid_p                      z_grid;
dgc_perception_map_cell_p       default_map_cell     = NULL;
dgc_perception_map_cell_p       default_terrain_cell = NULL;
short                           default_z_cell       = std::numeric_limits<short>::max();
perception_settings_t           settings;
unsigned short                  counter = 0;

bool            sourcePointcloudEnable = false;
bool            dynamicObjectsEnable = true;
structNAVINFO   latestNavInfo;
Handler         handler;
mutex           packet_mutex;
mutex           multiFrameMutex;

vector<point3d> sourceGridCloud;
bool            gridCloudFlag = false;
mutex           gridCloudMutex;

MyVisualization myVisual;
MyZcm           myzcm;


double yaw = 0;
double pitch = 0;
double roll = 0;
double dx = 0;
double dy = 0;
double dz = 0;

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

namespace TABLE
{
    std::vector<double> cos_vertc;
    std::vector<double> sin_vertc;
    std::vector<double> sin_rot;
    std::vector<double> cos_rot;
    std::vector<double> sin_rotc;
    std::vector<double> cos_rotc;
}

struct laserLine
{
    int id;
    double rotCorrection;
    double vertCorrection;
    double distCorrection;
    double distCorrectionX;
    double distCorrectionY;
    double vertOffsetCorrection;
    double horizOffsetCorrection;
    double focalDistance;
    double focalSlope;
};

struct MaxIntensity
{
    int maxintensity_;
};
struct MinIntensity
{
    int minintensity_;
};

vector<laserLine> correction;
vector<MaxIntensity> maxintensity_set;
vector<MinIntensity> minintensity_set;

int correctionLoad()
{
    string dbfile = (Directory + "db.xml");
    const char* filepath = dbfile.c_str();
    TiXmlDocument doc(filepath);
    bool loadOkay = doc.LoadFile();
    if (!loadOkay) {
        printf("Could not load test file %s. Error='%s'. Exiting.\n", filepath, doc.ErrorDesc());
        exit(1);
    }

    TiXmlElement* root = doc.RootElement();
    TiXmlNode* points = root->FirstChild("DB")->FirstChild("points_");
    TiXmlNode* minIntensity = root->FirstChild("DB")->FirstChild("minIntensity_");
    TiXmlNode* maxIntensity = root->FirstChild("DB")->FirstChild("maxIntensity_");
    int corretioncount = 0;
    for (TiXmlNode* item = points->FirstChild("item"); item; item = item->NextSibling("item"))
    {
        laserLine temp;
        temp.id = atoi(item->FirstChild("px")->FirstChild("id_")->ToElement()->GetText());
        temp.rotCorrection = atof(item->FirstChild("px")->FirstChild("rotCorrection_")->ToElement()->GetText());
        temp.rotCorrection *= ONEDEGREE_2_RAD;
        temp.vertCorrection = atof(item->FirstChild("px")->FirstChild("vertCorrection_")->ToElement()->GetText());
        temp.vertCorrection *= ONEDEGREE_2_RAD;
        temp.distCorrection = atof(item->FirstChild()->FirstChild("distCorrection_")->ToElement()->GetText());
        temp.distCorrectionX = atof(item->FirstChild("px")->FirstChild("distCorrectionX_")->ToElement()->GetText());
        temp.distCorrectionY = atof(item->FirstChild("px")->FirstChild("distCorrectionY_")->ToElement()->GetText());
        temp.vertOffsetCorrection = atof(item->FirstChild("px")->FirstChild("vertOffsetCorrection_")->ToElement()->GetText());
        temp.horizOffsetCorrection = atof(item->FirstChild("px")->FirstChild("horizOffsetCorrection_")->ToElement()->GetText());
        temp.focalDistance = atof(item->FirstChild("px")->FirstChild("focalDistance_")->ToElement()->GetText()) ;
        temp.focalSlope = atof(item->FirstChild("px")->FirstChild("focalSlope_")->ToElement()->GetText()) ;
        corretioncount++;
        correction.push_back(temp);
    }
    int laser = 0;
    for (TiXmlNode* item = maxIntensity->FirstChild("item"); item; item = item->NextSibling("item"))
    {
        MaxIntensity intentemp;
        intentemp.maxintensity_ = atoi(item->ToElement()->GetText());
        maxintensity_set.push_back(intentemp);
    }
    for (TiXmlNode* item = minIntensity->FirstChild("item"); item; item = item->NextSibling("item"))
    {
        MinIntensity intentemp;
        intentemp.minintensity_ = atoi(item->ToElement()->GetText());
        minintensity_set.push_back(intentemp);
    }
}

void HDLTableInit()
{
    int beam;
    int angle;
    //ONEDEGREE_2_RAD = 0.0174533
    for (beam = 0; beam < HDL64_FIRE_NUM; beam++)
    {
        TABLE::cos_rotc.push_back(cos(correction[beam].rotCorrection));
        TABLE::cos_vertc.push_back(cos(correction[beam].vertCorrection));
        TABLE::sin_rotc.push_back(sin(correction[beam].rotCorrection));
        TABLE::sin_vertc.push_back(sin(correction[beam].vertCorrection));
    }
    //HDL_NUM_ROT_ANGLES 36000
    for (angle = 0; angle < HDL_NUM_ROT_ANGLES; angle++)
    {
        TABLE::cos_rot.push_back(cos(angle * 0.01 * ONEDEGREE_2_RAD));
        TABLE::sin_rot.push_back(sin(angle * 0.01 * ONEDEGREE_2_RAD));
    }
}

struct hdl64point
{
    double x_;
    double y_;
    double z_;
    double range_;
    int intensity_;
    unsigned char block_id;
    int spin_;
    double timestamp;
};

struct oneFrame //this frame's pointcloud data
{
    vector<hdl64point> frameData;
};

oneFrame multibeamFrame;
vector<structNAVINFO> carNavinfoVector;
vector<oneFrame> multiFrameVector;//frame data queue


void  perception_init( void )
{
    obstacles_s = (dgc_perception_map_cells_p) malloc(sizeof(dgc_perception_map_cells_t));
    obstacles_s->num = 0;
    obstacles_s->cell = (dgc_perception_map_cell_p *) malloc( MAX_NUM_POINTS * sizeof(dgc_perception_map_cell_p) );
}

bool firstFlag = true;

struct onePacket{
    char packetData[1206];
};
vector<onePacket> myFrame;
vector<onePacket> TempFrame;

//receiver the lidar packet by ethernet
void receiverPackets(){
    int sockSrv, length;
    socklen_t clientlen;
    struct sockaddr_in server;
    struct sockaddr_in client;

    sockSrv=socket(AF_INET, SOCK_DGRAM, 0);
    if (sockSrv < 0)
        error("Opening socket");

    length = sizeof(server);
    memset(&server,0,length);
    server.sin_family=AF_INET;
    server.sin_addr.s_addr= htonl(INADDR_ANY);
    server.sin_port=htons(2368);

    if (bind(sockSrv, (struct sockaddr *) &server, length) < 0)
        error("binding");
    clientlen = sizeof(struct sockaddr_in);
    char recvBuf[1206];        

    int packet_num = 0;
    while(true){

        packet_num++;
        if(1206 == recvfrom(sockSrv, recvBuf, 1206, 0, (struct sockaddr *)&client, &clientlen)) {
            onePacket newPacket;
            memcpy(newPacket.packetData, recvBuf, 1206);
            packet_mutex.lock();
            myFrame.emplace_back(newPacket);
            packet_mutex.unlock();
        }
        else{
            error("HDL64 receiver shutdown");
            close(sockSrv);
        }
        usleep(0);
    }
}

//get 346 packets and calibration point x y z
void calibrationHDL64()
{
    correctionLoad();
    HDLTableInit();
    
    int packetCounter = 0;
    int isstart = true;

    while (true)
    {
        //TODO: adjust the batch size to find an optimal does not affect the lasermap JOHN 2017.10
        if(myFrame.size() > 346)//694
        {
            packet_mutex.lock();
            TempFrame .swap(myFrame);
            myFrame.clear();
            packet_mutex.unlock();

            int radionum = 0,rotationstart = 0;

            for(auto thisframe : TempFrame)
            {
                char thisPacket[1206];
                memcpy(thisPacket, thisframe.packetData, 1206);
                for (size_t j = 0; j < 12; j++)
                {
                    beam_n = 0;
                    double total_us = (unsigned char) thisPacket[ENET_H_LEN + 1203] * pow(2, 24) +
                        (unsigned char) thisPacket[ENET_H_LEN + 1202] * pow(2, 16) +
                        (unsigned char) thisPacket[ENET_H_LEN + 1201] * pow(2, 8) +
                        (unsigned char) thisPacket[ENET_H_LEN + 1200] * pow(2, 0);
                    double timestamp1 = (total_us) / pow(10, 6);
                    unsigned char block_id;
                    int block = (unsigned char) thisPacket[ENET_H_LEN + 1 + j * 100] * 16 * 16 +
                        (unsigned char) thisPacket[ENET_H_LEN + j * 100];

                    if (block == 0xEEFF)
                        block_id = 1;
                    else if (block == 0xDDFF)
                        block_id = 0;

                    spin_now = (unsigned char) thisPacket[ENET_H_LEN + 3 + j * 100] * 16 * 16 +
                        (unsigned char) thisPacket[ENET_H_LEN + 2 + j * 100]; 

                    //start record
                    if ((spin_now < 18000 || spin_now > 18028) && firstFlag)
                    {
                        radionum = 0;
                        continue;
                    }
                    else 
                        firstFlag = false;

                    if(isstart)
                    {
                        isstart = false;
                        rotationstart = spin_now;
                        // std::cout << "rotationstart = " << rotationstart << std::endl;
                    }

                    int distancelocal = 0;
                    if (j % 2 == 0 ) 
                    {
                        beam_n = 0;
                        for (int i = 4; i < 99; i += 3) 
                        {
                            //****************************coordinate calibration begin************************************************
                            dis = (unsigned char) thisPacket[ENET_H_LEN + i + j * 100 + 1] * 16 * 16 +
                                (unsigned char) thisPacket[ENET_H_LEN + i + j * 100];
                            intensity = (unsigned char) thisPacket[ENET_H_LEN + i + j * 100 + 2];

                            int intensity_raw = intensity;
                            int intensity_dif = 0;

                            COS_VC_n = TABLE::cos_vertc[beam_n];
                            SIN_VC_n = TABLE::sin_vertc[beam_n];

                            COS_RC_n = TABLE::cos_rotc[beam_n];
                            SIN_RC_n = TABLE::sin_rotc[beam_n];

                            SIN_ROT_n = TABLE::sin_rot[spin_now];
                            COS_ROT_n = TABLE::cos_rot[spin_now];

                            disC_n = correction[beam_n].distCorrection;
                            disCX_n = correction[beam_n].distCorrectionX;
                            disCY_n = correction[beam_n].distCorrectionY;

                            vertoffC_n = correction[beam_n].vertOffsetCorrection;
                            horioffC_n = correction[beam_n].horizOffsetCorrection;
                            rotc = correction[beam_n].rotCorrection;

                            distancetemp = (dis * DIS_LSB);
                            distancelocal= distancetemp + disC_n;

                            cosRotAngle = ((COS_ROT_n) * (COS_RC_n) + (SIN_ROT_n) * (SIN_RC_n));
                            sinRotAngle = ((SIN_ROT_n) * (COS_RC_n) - (COS_ROT_n) * (SIN_RC_n));

                            disxy = (COS_VC_n) * distancelocal - vertoffC_n * SIN_VC_n;//modi ROS

                            xx = fabs((disxy * sinRotAngle - horioffC_n * cosRotAngle));//
                            yy = fabs((disxy * cosRotAngle + horioffC_n * sinRotAngle)); //add ROS

                            distanceCorrX = ((disC_n - disCX_n) * (xx - 240) / (2504 - 240)) + disCX_n; //modi ROS
                            distanceCorrX -= disC_n;  //add ROS

                            distanceCorrY = ((disC_n - disCY_n) * (yy - 193) / (2504 - 193) + disCY_n); //modi ROS
                            distanceCorrY -= disC_n;  //add ROS

                            distanceX = distancelocal + distanceCorrX;//
                            disxy = (COS_VC_n) * distanceX - vertoffC_n * SIN_VC_n;  //modi ROS

                            x = (disxy * sinRotAngle - horioffC_n * cosRotAngle);//

                            distanceY = distancelocal + distanceCorrY;//
                            disxy = ((COS_VC_n) * distanceY) - vertoffC_n * SIN_VC_n;  //modi ROS
                            y = (disxy * cosRotAngle + horioffC_n * sinRotAngle);  //
                            z = (distanceY * SIN_VC_n + vertoffC_n * COS_VC_n); //modi ROS

                            //****************************coordinate calibration done************************************************

                            //****************************intensity calibration begin************************************************

                            int min_intensity = 0, max_intensity = 255;
                            float focaldistance = correction[beam_n].focalDistance;
                            float focalslope = correction[beam_n].focalSlope;
                            float focaloffset = 256 * (1 - focaldistance / 13100) * (1 - focaldistance / 13100);

                            intensity += focalslope * (fabs(focaloffset - 256 * (1 - (dis * 1.0 / 65535)) * (1 - (dis * 1.0 / 65535))));
                            if (intensity < min_intensity) intensity = min_intensity;
                            if (intensity > max_intensity) intensity = max_intensity;

                            intensity_dif = intensity - intensity_raw;
                            //****************************intensity calibration done************************************************

                            multibeamFrame.frameData.push_back( hdl64point{ x , y , z , (dis * 0.2), intensity, block_id, spin_now, timestamp1} );

                            beam_n++;
                        }
                    }
                    else
                    {
                        beam_n = 0;
                        for (int i = 4; i < 99; i += 3)
                        {
                            //****************************coordinate calibration begin************************************************

                            dis = (unsigned char) thisPacket[ENET_H_LEN + i + j * 100 + 1] * 16 * 16 +
                                (unsigned char) thisPacket[ENET_H_LEN + i + j * 100];
                            intensity = (unsigned char) thisPacket[ENET_H_LEN + i + j * 100 + 2];

                            int intensity_raw = intensity;
                            int intensity_dif = 0;

                            COS_VC_n = TABLE::cos_vertc[beam_n + 32];
                            SIN_VC_n = TABLE::sin_vertc[beam_n + 32];

                            COS_RC_n = TABLE::cos_rotc[beam_n + 32];
                            SIN_RC_n = TABLE::sin_rotc[beam_n + 32];

                            SIN_ROT_n = TABLE::sin_rot[spin_now];
                            COS_ROT_n = TABLE::cos_rot[spin_now];

                            disC_n = correction[beam_n + 32].distCorrection;
                            disCX_n = correction[beam_n + 32].distCorrectionX;
                            disCY_n = correction[beam_n + 32].distCorrectionY;

                            vertoffC_n = correction[beam_n + 32].vertOffsetCorrection;
                            horioffC_n = correction[beam_n + 32].horizOffsetCorrection;
                            rotc = correction[beam_n + 32].rotCorrection;

                            distancetemp = (dis * DIS_LSB);
                            distancelocal = distancetemp + disC_n;

                            cosRotAngle = ((COS_ROT_n) * (COS_RC_n) + (SIN_ROT_n) * (SIN_RC_n));
                            sinRotAngle = ((SIN_ROT_n) * (COS_RC_n) - (COS_ROT_n) * (SIN_RC_n));

                            disxy = (COS_VC_n) * distancelocal- vertoffC_n * SIN_VC_n;//modi ROS

                            xx = fabs((disxy * sinRotAngle - horioffC_n * cosRotAngle));//
                            yy = fabs((disxy * cosRotAngle + horioffC_n * sinRotAngle)); //add ROS

                            distanceCorrX = ((disC_n - disCX_n) * (xx - 240) / (2504 - 240)) + disCX_n; //modi ROS
                            distanceCorrX -= disC_n;  //add ROS

                            distanceCorrY = ((disC_n - disCY_n) * (yy - 193) / (2504 - 193) + disCY_n); //modi ROS
                            distanceCorrY -= disC_n;  //add ROS

                            distanceX = distancelocal+ distanceCorrX;//
                            disxy = (COS_VC_n) * distanceX - vertoffC_n * SIN_VC_n;  //modi ROS

                            x = (disxy * sinRotAngle - horioffC_n * cosRotAngle);//

                            distanceY = distancelocal + distanceCorrY;//
                            disxy = ((COS_VC_n) * distanceY) - vertoffC_n * SIN_VC_n;  //modi ROS
                            y = (disxy * cosRotAngle + horioffC_n * sinRotAngle);  //
                            z = (distanceY * SIN_VC_n + vertoffC_n * COS_VC_n); //modi ROS

                            //****************************coordinate calibration done************************************************

                            //****************************intensity calibration************************************************

                            int min_intensity = 0, max_intensity = 255;
                            float focaldistance = correction[beam_n + 32].focalDistance;
                            float focalslope = correction[beam_n + 32].focalSlope;
                            float focaloffset = 256 * (1 - focaldistance / 13100) * (1 - focaldistance / 13100);

                            intensity += focalslope * (fabs(focaloffset - 256 * (1 - (dis * 1.0 / 65535)) * (1 - (dis * 1.0 / 65535))));  //ÕâÀïµÄdisÊÇÄÄ¸ö¾àÀëÓÐ´ý¿¼¾¿

                            if (intensity < min_intensity) intensity = min_intensity;
                            if (intensity > max_intensity) intensity = max_intensity;

                            intensity_dif = intensity - intensity_raw;
                            //****************************intensity calibration done************************************************

                            multibeamFrame.frameData.push_back( hdl64point{ x , y , z , (dis * 0.2), intensity, block_id, spin_now, timestamp1} );

                            beam_n++;
                        }
                    }
                }

                radionum ++ ;

                //belong to one circle
                if (radionum + packetCounter > 300 && spin_now > 18000 && spin_now < 18220)
                {
                    handler.navinfoMutex.lock();
                    structNAVINFO carNavinfo = handler.navinfo;
                    handler.navinfoMutex.unlock();

                    multiFrameMutex.lock();
                    carNavinfoVector.clear(); //clean history car pose and push the current latest car pose
                    carNavinfoVector.push_back(carNavinfo);
                    multiFrameVector.clear(); //clean history frame and push the current latest frame
                    multiFrameVector.push_back(multibeamFrame);
                    multiFrameMutex.unlock();

                    multibeamFrame.frameData.clear();//clear the current data

                    isstart = true;
                    packetCounter = 0;
                    radionum = 0;
                }
            }
            packetCounter = radionum;
        }
        usleep(0);
    }
    return;
}


void ObjectTracking()
{

    SecondPython secondNet;

    float compensatePitch;

    //initial grid
    grid =  dgc_grid_initialize(grid_stat.resolution,
                                grid_stat.mapsize.x,            
                                grid_stat.mapsize.y,               
                                sizeof(dgc_perception_map_cell_t),    
                                default_map_cell);                 
    dgc_grid_recenter_grid(grid, 0, 20.1);
    applanix_history_init(APPLANIX_HISTORY_LENGTH);

    velodyne = (dgc_velodyne_data_p) malloc(sizeof(dgc_velodyne_data_t));
    velodyne_init(velodyne); 
    perception_init();  

    int scan_num = 0,linectr = 0;
    dgc_pose_t robot = {0,0,0,0,0,0};
    
    vector<float> buffer;

    while(true) 
    {
        if( multiFrameVector.size() && carNavinfoVector.size() )
        {
            multiFrameMutex.lock();
            oneFrame latestMultiFrame = multiFrameVector.back();
            latestNavInfo = carNavinfoVector.back();
            carNavinfoVector.clear();
            multiFrameVector.clear();
            multiFrameMutex.unlock();

            //pitch compensation for lidar pointcloud
            if(latestNavInfo.mRTKStatus != 1)
                compensatePitch = pitch;    //default compensate pitch
            else 
                compensatePitch = latestNavInfo.mPitch;

            double yawRad = TiEV::deg2rad(yaw);
            double pitchRad = TiEV::deg2rad(compensatePitch);
            double rollRad = TiEV::deg2rad(roll);

            cout << "pitch compensatePitch: " << compensatePitch << endl;

            Eigen::Vector3d ea0(yawRad,rollRad,pitchRad);
            Eigen::Matrix3d RotationM;
            RotationM = 
              Eigen::AngleAxisd(ea0[0], Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(ea0[1], Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(ea0[2], Eigen::Vector3d::UnitX());

            Eigen::Vector3d translateVec(dx * 100, dy * 100, dz * 100);

            for (auto line : latestMultiFrame.frameData)
            {
                linectr++;
                Eigen::Vector3d originPC(line.x_,line.y_,line.z_);
                Eigen::Vector3d transformPC = RotationM * originPC + translateVec;
                velodyne->scans[scan_num].p[linectr-1].x = transformPC(0);
                velodyne->scans[scan_num].p[linectr-1].y = transformPC(1);
                velodyne->scans[scan_num].p[linectr-1].z = transformPC(2);
                velodyne->scans[scan_num].p[linectr-1].range = line.range_;
                velodyne->scans[scan_num].p[linectr-1].intensity = line.intensity_;
                if(linectr % 32 == 0){
                    velodyne->scans[scan_num].block = line.block_id;
                    velodyne->scans[scan_num].robot = robot;
                    velodyne->scans[scan_num].timestamp = line.timestamp;
                    velodyne->scans[scan_num].encoder = line.spin_;
                    scan_num++;
                    linectr = 0;
                }

                //cloud to second
                float x = transformPC(0) / 100.0;
                float y = transformPC(1) / 100.0;
                float z = transformPC(2) / 100.0;
                float intensity = line.intensity_ / 255.0;

                if(fabs(x) < 25 && y < 70 && y > -30) //extend area
                {
                    //cloud coordinate change for Second Net detection
                    buffer.push_back(y);
                    buffer.push_back(-x);
                    buffer.push_back(z);
                    buffer.push_back(intensity);
                }
            }

            //pass the pointcloud, and start detect by second Net and return result
            secondNet.startReceiver(buffer, velodyne->scans->timestamp);

            buffer.clear();

            cout << "multibeamlaser total points number = " << scan_num * 32 << std::endl;

            velodyne->num_scans = scan_num;
            scan_num = 0;

            velodyne->preprocessed = false;

            //pass the pointcloud, and start build grid map
            integrate_sensors(velodyne);

            free(velodyne->scans);
            velodyne->scans = (dgc_velodyne_scan_p) malloc(50000 * sizeof(dgc_velodyne_scan_t));

            free(obstacles_s->cell);
            free(obstacles_s);
            obstacles_s = (dgc_perception_map_cells_p) malloc(sizeof(dgc_perception_map_cells_t));
            obstacles_s->num = 0;
            obstacles_s->cell = (dgc_perception_map_cell_p *) malloc( MAX_NUM_POINTS * sizeof(dgc_perception_map_cell_p) );
        }
        usleep(10 * 1000);
    }
}

//publish indoor pointcloud, this is only for indoor localization
void gridCloudForLocationIndoor()
{
    int sendCounter = 0;
    while(1)
    {
        if(gridCloudFlag)
        {
            memset(&myzcm.myPointCloud, 0, sizeof(myzcm.myPointCloud));

            gridCloudMutex.lock();
            gridCloudFlag = false;
            for (int i = 0; i < min((int)sourceGridCloud.size(), 100000); i++)
            {
                myzcm.myPointCloud.frame[i].px_ = sourceGridCloud[i].px_;
                myzcm.myPointCloud.frame[i].py_ = sourceGridCloud[i].py_;
                myzcm.myPointCloud.frame[i].pz_ = sourceGridCloud[i].pz_;
                sendCounter++;
            }
            sourceGridCloud.clear();
            gridCloudMutex.unlock();

            myzcm.myPointCloud.total = sendCounter;
            sendCounter = 0;
        }
        myzcm.myPointCloud.timestamp = TiEV::getTimeStamp();
        myzcm.pub_cloud();
        usleep(50 * 1000); //20 hz
    }
}

//subscribe slam_control, regenerate new paramter only for indoor localization
void MyZcm::sub_ipc_msgs()
{
    if (!ipc_sub.good())
    {
        perror("ZCM NOT GOOD");
        exit(0);
    }
    ipc_sub.subscribe("UNDERGROUND_CONTROL", &Handler::handleSlamControlMessage, &handler);
    ipc_sub.run();
}

//subscribe navinfo msg, get latest gps localiztion data
void MyZcm::sub_udpm_msgs()
{
    if (!msg_sub.good())
    {
        perror("ZCM NOT GOOD");
        exit(0);
    }
    msg_sub.subscribe("NAVINFO", &Handler::handleNavinfoMessage, &handler);
    msg_sub.run();
}

//read json param 
void paramRead(const string fileName)
{
    ifstream in(fileName);
    if(!in.is_open()) {
        cout << "can't open json : " << fileName << endl;
        assert(false);
        return;
    }
    stringstream buffer;
    buffer << in.rdbuf();
    Document doc;
    doc.Parse(buffer.str().c_str());

    sourcePointcloudEnable = doc["sourcePointcloudEnable"].GetBool();
    dynamicObjectsEnable = doc["dynamicObjectsEnable"].GetBool();

    settings.overpass_height = doc["overpass_height"].GetDouble();
    settings.z_resolution = doc["z_resolution"].GetDouble();
    settings.z_obstacle_height = doc["z_obstacle_height"].GetDouble();
    
    yaw = doc["yaw"].GetDouble();
    pitch = doc["pitch"].GetDouble();
    roll = doc["roll"].GetDouble();
    dx = doc["dx"].GetDouble();
    dy = doc["dy"].GetDouble();
    dz = doc["dz"].GetDouble();
}

int main()
{
    paramRead(Directory + "param.json");
    settings.rate_in_hz = 8.0;
    settings.map_cell_threshold = 0.3;
    settings.map_cell_min_hits = 2;
    settings.map_cell_increase = 5;
    settings.map_ray_tracing = 1;
    settings.show_ray_tracing = 0;
    settings.velodyne_sync = 1;
    settings.segmentation_settings.min_height = 0.3;
    settings.tracker_settings.filter_rndf_max_distance = 5.0;
    settings.tracker_settings.filter_rndf_max_pedestrian_distance = 50.0;
    settings.tracker_settings.filter_rndf = 0;
    settings.tracker_settings.merge_dist = 5.0;
    settings.tracker_settings.lateral_merge_dist = 0.75;
    settings.clear_sensor_data = 0;
    settings.max_sensor_delay = 0.4;
    settings.extract_dynamic = 1;
    settings.num_threads = 6;

    //this is for grid initialize
    settings.map_resolution = TiEV::GRID_RESOLUTION;
    settings.map_size_x = TiEV::GRID_RESOLUTION * TiEV::GRID_ROW;
    settings.map_size_y = TiEV::GRID_RESOLUTION * TiEV::GRID_COL;
    //this is for kalman tracker 
    settings.kf_settings.correspondence_threshold = 1e-40;
    settings.kf_settings.pruning_threshold = 0.3;
    settings.kf_settings.measurement_variance = 0.1;
    settings.kf_settings.position_variance = 0.7;//0.1;
    settings.kf_settings.velocity_variance = 0.001;
    settings.kf_settings.heading_variance = 0.01;
    settings.kf_settings.heading_velocity_variance = 0.01;
    settings.kf_settings.initial_position_variance = 0.05;
    settings.kf_settings.initial_velocity_variance = 1.0;
    settings.kf_settings.initial_heading_variance = 0.5;
    settings.kf_settings.initial_heading_velocity_variance = 0.5;

    settings.use_velodyne = 1;
    settings.velodyne_threshold_factor = 0.0013;
    settings.velodyne_max_range = 80.0;
    settings.velodyne_min_beam_diff = 0.50;
    string calfile = Directory + "ID89.cal";
    settings.velodyne_cal = const_cast<char *>(calfile.c_str());
    settings.segmentation_settings.max_points = 500;
    settings.segmentation_settings.min_points = 5;
    settings.segmentation_settings.kernel_size = 4;
    string classifierfile = Directory + "classifier1.mb";
    settings.tracker_settings.classifier_filename = const_cast<char *>(classifierfile.c_str());

    default_map_cell = (dgc_perception_map_cell_p) calloc(1, sizeof(dgc_perception_map_cell_t));
    dgc_test_alloc(default_map_cell);
    default_map_cell->max = -10000;
    default_map_cell->min = 10000;
    default_map_cell->hits = 0;
    default_map_cell->seen = 0;
    default_map_cell->last_min = 0;
    default_map_cell->last_max = 0;
    default_map_cell->last_observed = 0;
    default_map_cell->last_obstacle = 0;
    default_map_cell->last_dynamic = 0;
    default_map_cell->last_mod = 0;
    default_map_cell->region = 0;
    default_map_cell->obstacle = PERCEPTION_MAP_OBSTACLE_FREE;
    default_map_cell->street = 1;


    default_terrain_cell = (dgc_perception_map_cell_p) calloc(1, sizeof(dgc_perception_map_cell_t));
    dgc_test_alloc(default_terrain_cell);
    *default_terrain_cell = *default_map_cell;

    grid_stat.mapsize.x = (int) (settings.map_size_x / settings.map_resolution);
    grid_stat.mapsize.y = (int) (settings.map_size_y / settings.map_resolution);
    grid_stat.resolution = settings.map_resolution;
    grid_stat.center.x = 0;
    grid_stat.center.y = 0;
    grid_stat.z_resolution = settings.z_resolution;

    thread zcm_udpm_receiver_thread(&MyZcm::sub_udpm_msgs, &myzcm);
    thread zcm_ipc_receiver_thread(&MyZcm::sub_ipc_msgs, &myzcm);
    thread lidar_recieve_thread(receiverPackets);
    thread calibration_HDL64_thread(calibrationHDL64);
    thread object_tracking_thread(ObjectTracking);
    thread localization_indoor_thread(gridCloudForLocationIndoor);

    zcm_udpm_receiver_thread.join();
    zcm_ipc_receiver_thread.join();
    lidar_recieve_thread.join();
    calibration_HDL64_thread.join();
    object_tracking_thread.join();
    localization_indoor_thread.join();

    return 0;
}
