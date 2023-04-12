#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <map>
#include <vector>
#include <cstring>
#include <chrono>
#include <thread>
#include <cmath>
#include "mqtt/client.h"
#include "../proto/TJ_v2x.pb.h"

using namespace std;

struct DataPoint {
    long long timeStamp; // 时间戳
    double utmX;         // UTM X坐标
    double utmY;         // UTM Y坐标
    double heading;      // 方向（弧度）
    double velocity;     // 速度（m/s）
    double acceleration; // 加速度（m/s^2）
};

const std::string SERVER_ADDRESS("tcp://221.181.123.244:22683");
const std::string CLIENT_ID("mec2cloud_003");
const std::string PASSWORD("mec_123qwe$");
const std::string TOPIC("cloudproc_02");
const int QOS = 1;

class user_callback : public virtual mqtt::callback
{
	void connection_lost(const std::string &cause) override
	{
		std::cout << "\nConnection lost" << std::endl;
		if (!cause.empty())
			std::cout << "\tcause: " << cause << std::endl;
	}

	void delivery_complete(mqtt::delivery_token_ptr tok) override
	{
		std::cout << "\n\t[Delivery complete for token: "
				  << (tok ? tok->get_message_id() : -1) << "]" << std::endl;
	}

public:
};

u_int32_t PointPosConverter(double theta_){//Convert the counterclockwise angle with the x-axis to the clockwise angle with the y-axis.
		while(theta_ > M_PI){
			theta_ -= 2 * M_PI;
		}
		while(theta_ < -1 * M_PI){
			theta_ += 2 * M_PI;
		}

		if(theta_ <= M_PI_2){
			theta_ = M_PI_2 - theta_;
		}
		else{
			theta_ = 5 * M_PI_2 - theta_;
		}

		u_int32_t theta_deg = (double)(theta_ * 180 / M_PI / 0.0125);
		return theta_deg;
	}

void CreatePathPlanningPoint(cn::seisys::v2x::pb::PathPlanningPoint &pppoint, DataPoint &point, int time)
{
    try
    {
        //cout << "begin create PathPlanningPoint" << endl;
        auto *position3D = new cn::seisys::v2x::pb::Position3D();
        position3D->set_ele(1);
        position3D->set_lat(1e2 * point.utmX);
        position3D->set_lon(1e2 * point.utmY);
        pppoint.set_allocated_pos(position3D);
		pppoint.set_speed(point.velocity / 0.02);
		pppoint.set_heading(PointPosConverter(point.heading));
		pppoint.set_estimatedtime(time / 10);
        //cout << "Create PathPlanningPoint success" << endl;
    }
    catch (...)
    {
        std::cout << "Create Polygon error!" << std::endl;
    }
}

int main(int argc, char *argv[])
{
	std::cout << "Initialzing..." << std::endl;
	mqtt::client client(SERVER_ADDRESS, CLIENT_ID);

	user_callback cb;
	client.set_callback(cb);

	mqtt::connect_options connOpts;
	connOpts.set_keep_alive_interval(20);
	//connOpts.set_clean_session(true);
	connOpts.set_password(PASSWORD); //config password from emqx
	std::cout << "...OK" << std::endl;

	std::ifstream inputFile("../Trajectory_0313.txt");
    std::vector<DataPoint> dataPoints;
    if (inputFile.is_open()) {
        std::string line;
        while (std::getline(inputFile, line)) {
            DataPoint dataPoint;
            std::sscanf(line.c_str(), "%lld %lf %lf %lf %lf %lf",
                        &dataPoint.timeStamp, &dataPoint.utmX, &dataPoint.utmY,
                        &dataPoint.heading, &dataPoint.velocity, &dataPoint.acceleration);
            dataPoints.push_back(dataPoint);
        }
        inputFile.close();
    }

	try
	{
		std::cout << "\nConnecting..." << std::endl;
		client.connect(connOpts);
		std::cout << "...OK" << std::endl;



		// First use a message pointer.
		//int i = 0;
      while(1){
        cn::seisys::v2x::pb::CloudToV cloudToV;
		cloudToV.set_obuid(1);
        cloudToV.set_timestamp(123456789);
        cloudToV.set_sceneid(cn::seisys::v2x::pb::CloudToV_SceneId::CloudToV_SceneId_SCENE_ID_1);
        for (int i = 0; i < 100; i++)
        {
            auto *path_planning = cloudToV.add_pathplanning();
            CreatePathPlanningPoint(*path_planning, dataPoints[i], dataPoints[i].timeStamp - dataPoints[0].timeStamp);
        }
        


        std::string message_string;
        cloudToV.SerializeToString(&message_string);

		std::cout << "\nSending message..." << std::endl;
		auto pubmsg = mqtt::make_message(TOPIC, message_string.c_str());
		std::cout<<pubmsg;
		pubmsg->set_qos(QOS);
		client.publish(pubmsg);
		std::cout << "...OK" << std::endl;

		std::this_thread::sleep_for(std::chrono::milliseconds(100));

      }
		// Disconnect
		std::cout << "\nDisconnecting..." << std::endl;
		client.disconnect();
		std::cout << "...OK" << std::endl;
	}
	catch (const mqtt::persistence_exception &exc)
	{
		std::cerr << "Persistence Error: " << exc.what() << " ["
				  << exc.get_reason_code() << "]" << std::endl;
		return 1;
	}
	catch (const mqtt::exception &exc)
	{
		std::cerr << exc.what() << std::endl;
		return 1;
	}

	std::cout << "\nExiting" << std::endl;
	return 0;
}


