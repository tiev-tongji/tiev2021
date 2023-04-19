#include "second_receiver.h"

#define deg2rad 0.0174533
#define rad2deg 57.295780


SecondPython::SecondPython()
{
	
	Py_Initialize();
	PyRun_SimpleString("import sys");
	// PyRun_SimpleString("sys.path.append(\'/home/autolab/tiev/src/modules/second.pytorch/second/pytorch\')");
	PyRun_SimpleString("sys.path.append(\'/home/autolab/tiev/src/modules/OpenPCDet-av2_plus/tools')");
	//PyRun_SimpleString("sys.path.append(\'/home/autolab/txb/second.pytorch/second/pytorch\')");
	
	// pymodule = PyImport_ImportModule("SECOND");
	cout << "check 1" << endl;

	pymodule = PyImport_ImportModule("detect");
	PyRun_SimpleString("print(\"hhhhhhhh\")");
	if(pymodule != NULL)
	{
		cout << "load python module sucess" << endl;
	}
	else 
	{
		PyErr_Print();
	}
	pyfunction = PyObject_GetAttrString(pymodule, "get_boxes");
	if(pyfunction != NULL)
	{
		cout << "load python function sucess" << endl;
	}

	Py_XDECREF(pymodule);

}

SecondPython::~SecondPython()
{
	if(pymodule)
		Py_XDECREF(pymodule);
	if(pyfunction)
		Py_XDECREF(pyfunction);
}

void SecondPython::startReceiver(vector<float> &myBuffer, double timestamp)
{
	import_array1(); 

	npy_intp dim[] = {myBuffer.size() / 4, 4};
	PyObject* np_arg = PyArray_SimpleNewFromData(2, dim, NPY_FLOAT, &myBuffer[0]); 
	PyObject* return_value1 = PyObject_CallFunction(pyfunction, "O", np_arg);

	PyArrayObject *returnArray = NULL;
	returnArray = (PyArrayObject *)PyArray_FROM_OTF(return_value1, NPY_DOUBLE, NPY_ARRAY_IN_ARRAY);
	int objNum = PyArray_DIM(returnArray, 0);

    obstacles_second.clear();
    //TODO Verified
    //Detector coodinate system (f-l-u -> x-y-z, yaw start from x, ccw is positive, 0~2pi)
    //Lidar coodinate system (r-f-u -> x-y-z)
	for(int i = 0; i < objNum; i ++)
	{
		int type = *(double *)PyArray_GETPTR2(returnArray, i, 0);
		double y = *(double *)PyArray_GETPTR2(returnArray, i, 1);
		double x = 0.0 - *(double *)PyArray_GETPTR2(returnArray, i, 2);
		double z = *(double *)PyArray_GETPTR2(returnArray, i, 3);
		double length = *(double *)PyArray_GETPTR2(returnArray, i, 4);
		double width = *(double *)PyArray_GETPTR2(returnArray, i, 5);
		//NOTE direction is still in Detector coordinate system
		double direction = *(double *)PyArray_GETPTR2(returnArray, i, 7);
		if(fabs(x) < 1 && y < 1 && y > -1)
			continue;

	//	std::cout << "obj x = " << x << " y = " << y << " length = " << length << " width = " << width <<std::endl;

		//model_dir_apo_all_2.0
		/*
		 *   direction = 0 when x car right, and clockwise
		 *	double objyaw = 0 - direction;
		 */

		//model_dir_apo_all
		/*
		 *   direction = 0 when x car back, and clockwise
		 *	double objyaw = 0 - direction - M_PI_2;
		 */

		double velx = 0, vely = 3;
		if(length < width)
			swap(length, width); //max is y, min is the x

		//TODO change direction from 0~2pi to?? 
		double objyaw = - direction;// - M_PI_2;

		while(objyaw < - M_PI)
		{
			objyaw += M_PI * 2;
		}
		while(objyaw > M_PI)
		{
			objyaw -= M_PI * 2;
		}

		double x1,y1,x2,y2,x3,y3,x4,y4,x_end,y_end;
		double rotationTheta = objyaw - M_PI_2;//-direction; //objyaw - M_PI_2;
		transform( width/2.0,  length/2.0, rotationTheta, x, y, x1, y1);
		transform( width/2.0, -length/2.0, rotationTheta, x, y, x2, y2);
		transform(-width/2.0, -length/2.0, rotationTheta, x, y, x3, y3);
		transform(-width/2.0,  length/2.0, rotationTheta, x, y, x4, y4);
		transform(velx, vely, rotationTheta, x, y, x_end, y_end);
        
		std::tr1::shared_ptr<GridObstacle>* second_obstacle = new std::tr1::shared_ptr<GridObstacle>(new GridObstacle(i, grid));

		(*second_obstacle)->time_ = timestamp;
		(*second_obstacle)->setPose(x, y, z, objyaw, length, width);

		if(0) //draw bounding boxes of second objects 
		{
			char bufferprint[50];
			sprintf(bufferprint, "%.2f", objyaw * rad2deg);
			myVisual.drawText(point2d_t(x, y), bufferprint);
			Scalar sca;
			sca = Scalar(0, 255, 0);
			myVisual.drawLine(point2d_t(x1, y1), point2d_t(x2, y2), sca);
			myVisual.drawLine(point2d_t(x2, y2), point2d_t(x3, y3), sca);
			myVisual.drawLine(point2d_t(x3, y3), point2d_t(x4, y4), sca);
			myVisual.drawLine(point2d_t(x4, y4), point2d_t(x1, y1), sca);
			myVisual.drawLine(point2d_t(x, y), point2d_t(x_end, y_end), sca);
		}
		switch (type)
		{
			case OBSTACLE_CAR :
			(*second_obstacle)->second_type = OBSTACLE_CAR;
			break;

			case OBSTACLE_BICYCLIST :
			(*second_obstacle)->second_type = OBSTACLE_BICYCLIST;
			break;

			case OBSTACLE_PEDESTRIAN :
			(*second_obstacle)->second_type = OBSTACLE_PEDESTRIAN;
			break;

			default :
			(*second_obstacle)->second_type = OBSTACLE_UNKNOWN;
		}
		(*second_obstacle)->classified_this_frame_ = true;
		obstacles_second.push_back(*second_obstacle);
	}
	Py_XDECREF(np_arg);
	Py_XDECREF(return_value1);
	Py_XDECREF(returnArray);

}
