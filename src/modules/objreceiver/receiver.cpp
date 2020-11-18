#include <iostream>
#include <mutex>
#include <thread>
#include <string>
#include "receiver.h"

Mat image = Mat::zeros(TiEV::GRID_ROW, TiEV::GRID_COL, CV_8UC3);
char window[] = "objlist";

Handler handler;
// lcm::LCM             myLcm("udpm://239.255.76.67:7667?ttl=1");
//zcm::ZCM myzcm{"udpm://239.255.76.67:7667?ttl=1"};
zcm::ZCM myzcm{"ipc"};

void zcmFunc()
{
	cerr << "Start Receiver Thread" << endl;
	if (!myzcm.good())
	{
		cerr << "ZCM NOT GOOD" << endl;
	}
	//myzcm.subscribe("objectslist", &Handler::handleObjlistMessage, &handler);
	myzcm.subscribe("SICKMAP", &Handler::handleSickMessage, &handler);
	//myzcm.subscribe("ESROBJLIST", &Handler::handleEsrObjMessage, &handler);
	while (0 == myzcm.handle())
		;
}

void project_map(double x1, double y1, double x2, double y2)
{
	static int p(0);
	int rows1 = TiEV::CAR_CEN_ROW - (int)(y1 / 0.2);
	int cols1 = TiEV::CAR_CEN_COL + (int)(x1 / 0.2);
	int rows2 = TiEV::CAR_CEN_ROW - (int)(y2 / 0.2);
	int cols2 = TiEV::CAR_CEN_COL + (int)(x2 / 0.2);

	std::cout << rows1 << "\t" << cols1 << "\t" << rows2 << "\t" << cols2 << "\t" << p << std::endl;

	if (rows1 >= 0 && rows1 < TiEV::GRID_ROW && cols1 >= 0 && cols1 < TiEV::GRID_COL && rows2 >= 0 && rows2 < TiEV::GRID_ROW && cols2 >= 0 && cols2 < TiEV::GRID_COL)
	{
		MyLine(image, Point(cols1, rows1), Point(cols2, rows2));
		p++;
	}
}

void process()
{
	while (1)
	{

		// handler.esrmutex.lock();
		// structESROBJINFO esrobj = handler.esrobj;
		// handler.esrmutex.unlock();

		// for (int i = 0; i < esrobj.m_number_objs; ++i)
		// {
		// 	for (int p = esrobj.objects[i].m_vertical - 3; p < esrobj.objects[i].m_vertical + 3; ++p)
		// 	{
		// 		for (int q = esrobj.objects[i].m_horizon - 3; q < esrobj.objects[i].m_horizon + 3; ++q)
		// 		{
		// 			image.at<Vec3b>(p, q)[0] = 255;
		// 			image.at<Vec3b>(p, q)[1] = 255;
		// 			image.at<Vec3b>(p, q)[2] = 255;
		// 		}
		// 	}
		// }

		// handler.objlistMutex.lock();
		// structOBJECTLIST myobjlist = handler.objlist;
		// handler.objlistMutex.unlock();

		handler.sickMutex.lock();
		structSICKMAP mysickmap = handler.sickmap;
		handler.sickMutex.unlock();

		// int num_obstacles = myobjlist.count;
		// // std::cout<<num_obstacles<<std::endl;

		// for (int i = 0; i < num_obstacles; i++)
		// {
		// 	// myobjlist.obj[i].id;
		// 	// myobjlist.obj[i].type;
		// 	double width = myobjlist.obj[i].width;
		// 	double length = myobjlist.obj[i].length;
		// 	double x = myobjlist.obj[i].x;
		// 	double y = myobjlist.obj[i].y;
		// 	double direction = myobjlist.obj[i].theta + 3.1415936 / 2;
		// 	double velocity = myobjlist.obj[i].v;

		// 	transform_R R;
		// 	transform_identity(R);
		// 	transform_rotate_z(R, direction);

		// 	double w1 = (length / 2.0); //right upper
		// 	double l1 = (width / 2.0);

		// 	double w2 = w1; //right bottom
		// 	double l2 = -l1;

		// 	double w3 = -w1; //left bottom
		// 	double l3 = -l1;

		// 	double w4 = -w1; //left upper
		// 	double l4 = l1;

		// 	double unused_z = 0;
		// 	transform_point(&w1, &l1, &unused_z, R);
		// 	transform_point(&w2, &l2, &unused_z, R);
		// 	transform_point(&w3, &l3, &unused_z, R);
		// 	transform_point(&w4, &l4, &unused_z, R);

		// 	double x1 = x + w1;
		// 	double y1 = y + l1;

		// 	double x2 = x + w2;
		// 	double y2 = y + l2;

		// 	double x3 = x + w3;
		// 	double y3 = y + l3;

		// 	double x4 = x + w4;
		// 	double y4 = y + l4;

		// 	double x_end = x + velocity * cos(direction);
		// 	double y_end = y + velocity * sin(direction);

		// 	project_map(x1, y1, x2, y2);
		// 	project_map(x2, y2, x3, y3);
		// 	project_map(x3, y3, x4, y4);
		// 	project_map(x4, y4, x1, y1);
		// 	project_map(x, y, x_end, y_end);
		// }
		for (int i = TiEV::CAR_CEN_ROW - 10; i < TiEV::CAR_CEN_ROW + 10; ++i)
		{
			for (int j = TiEV::CAR_CEN_COL - 4; j < TiEV::CAR_CEN_COL + 4; ++j)
			{
				image.at<Vec3b>(i, j)[0] = 0;
				image.at<Vec3b>(i, j)[1] = 0;
				image.at<Vec3b>(i, j)[2] = 255;
			}
		}
		namedWindow(window, 0);
		imshow(window, image);
		waitKey(1);

		for (int i = 0; i < TiEV::GRID_ROW; ++i)
		{
			for (int j = 0; j < TiEV::GRID_COL; ++j)
			{
				image.at<Vec3b>(i, j)[0] = 0;
				image.at<Vec3b>(i, j)[1] = 0;
				image.at<Vec3b>(i, j)[2] = 0;
				if (mysickmap.cells[i][j] == 1)
				{
					image.at<Vec3b>(i, j)[0] = 255;
					image.at<Vec3b>(i, j)[1] = 255;
					image.at<Vec3b>(i, j)[2] = 255;
				}
			}
		}
		usleep(100); //todo
	}
}

int main()
{
	thread receiver(zcmFunc);
	thread showObjlist(process);
	receiver.join();
	showObjlist.join();
	return 0;
}