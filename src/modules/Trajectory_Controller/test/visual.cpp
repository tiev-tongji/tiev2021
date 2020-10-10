#include "visual.h"

using namespace cv;
using namespace std;

Visualization::Visualization()
{
	this->init();
}

void Visualization :: getAimPath(const std::vector<TrajectoryPoint> &_apath)
{
	cout << "call getAimPath Function!" << endl;
	cout << "getted path size: " << _apath.size() << endl;
    AIMPATHmtx.lock();
    apath.clear();
    for(int i = 0; i < _apath.size(); i++)
    {
        TrajectoryPoint tmp;
        tmp.x = _apath[i].x;
        tmp.y = _apath[i].y;
        tmp.v = _apath[i].v;
        apath.push_back(tmp);
    }
    AIMPATHmtx.unlock();
}

void Visualization :: clearAimPath()
{
    AIMPATHmtx.lock();
    apath.clear();
    AIMPATHmtx.unlock();
}

void Visualization :: init()
{
    oriheading = 0.0;
    res = 0.10;
    w = 151;
    h = 401;
    ori.x = 0.0;
    ori.y = 0.0;
    cen.x = 75.0;
    cen.y = h - 100.0;
    cen_rear.x = cen.x;
    cen_rear.y = cen.y + 23.05;
    
    maps_num = 0;
    image_maps = cv::Mat::zeros(450, 175 * maps_num, CV_8UC3);
}

cv::Point Visualization :: MyCoord(cv::Point2f star)
{
	cv::Point2f newpt = star - ori;
	cv::Point2f newpt3;
	newpt3.x = cen.x + star.y;
	newpt3.y = cen.y - star.x;
	return newpt3;
}

void Visualization :: MyCar(cv::Mat img)
{
	rectangle(img,
		cv::Point2f(cen.x - 0.75 / res, cen.y - 0.35 / res),
		cv::Point2f(cen.x + 0.75 / res, cen.y + 2.85 / res),
		CAR_COLOR);
	line(img,
		 cv::Point2f(cen.x - 0.675 / res, cen.y - 0.0 / res),
		 cv::Point2f(cen.x + 0.675 / res, cen.y + 0.0 / res),
		 WHEEL_COLOR);
	line(img,
		 cv::Point2f(cen.x - 0.695 / res, cen.y + 2.305 / res),
		 cv::Point2f(cen.x + 0.695 / res, cen.y + 2.305 / res),
		 WHEEL_COLOR);
	line(img,
		 cv::Point2f(cen.x - 0.0 / res, cen.y - 0.0 / res),
		 cv::Point2f(cen.x + 0.0 / res, cen.y + 2.305 / res),
		 WHEEL_COLOR);
	circle(img, cen, 1, CARPOINT_COLOR, -1);
	circle(img, cen_rear, 1, CARPOINT_COLOR, -1);
}

void Visualization :: MyLine(cv::Mat img, cv::Point2f start, cv::Point2f end, cv::Scalar color)
{
	//convert
	cv::Point localstart = MyCoord(start);
	cv::Point localend = MyCoord(end);

	//draw 
	int thickness = 1;
	int lineType = 8;

	cout << "point: x = " << localstart.x << "  y = " << localend.y << endl; 
	line(img, localstart, localend,
		color,
		thickness,
		lineType);
//	circle(img, localstart, 1, PATHPOINT_COLOR, -1);
}

void Visualization :: MyPath(cv::Mat img)
{
	if(apath.size() < 1) return;
	cout << "path size to draw: " << apath.size() << endl;
    AIMPATHmtx.lock();
	cv::Point2f cen_current(0.0, 0.0);
	cv::Point2f ref(apath[0].x, apath[0].y);
    for(int i = 0; i < apath.size(); i++)
    {
		if(apath[i].v < 0)
		{
			cen_current.x = -2.305;
			cen_current.y = 0.0;
		}
		else
		{
			cen_current.x = 0.0;
			cen_current.y = 0.0;
		}
		double dis = sqrt((cen_current.x - apath[i].x)*(cen_current.x - apath[i].x) + (cen_current.y - apath[i].y)*(cen_current.y - apath[i].y));
		double dis_min = sqrt((cen_current.x - ref.x)*(cen_current.x - ref.x) + (cen_current.y - ref.y)*(cen_current.y - ref.y));
		if(dis < dis_min)
		{
			ref.x =apath[i].x;
			ref.y =apath[i].y;
		}

        cv::Point2f start(0.0, 0.0);
        cv::Point2f end(0.0, 0.0);
        if(i == apath.size() - 1) break;
        start.x = apath[i].x / res;
        start.y = apath[i].y / res;
        end.x = apath[i+1].x / res;
        end.y = apath[i+1].y / res;
        MyLine(img, start, end, AIMPATH_COLOR);

//		cout << "point[" << i << "]: x = " << apath[i].x << "  y = " << apath[i].y << "  v = " << apath[i].v << endl; 
    }

	MyLine(img, cen_current / res, ref / res, DEVLINE_COLOR);
    AIMPATHmtx.unlock();
}

cv::Mat Visualization :: DrawControlMap()
{
    cv::Mat img_trajmap = cv::Mat::zeros(MAXN, MAXM, CV_8UC3);
    
    MyCar(img_trajmap);
    MyPath(img_trajmap);

    return img_trajmap;
}

cv::Mat Visualization :: AllMaps(const std::map<std::string, cv::Mat> &_maps)
{
    cv::Mat Window(450, 175 * maps_num, CV_8UC3, Scalar(0, 0, 0));
    cv::Mat imageROI;
    int i = 0;
    std::map<std::string, cv::Mat>::const_iterator iter;
    iter = _maps.begin();
    while(iter != _maps.end())
    {
        imageROI = Window(Rect(175 * i, FRAME_TOP_Y, MAXM, MAXN));
        (iter->second).copyTo(imageROI);
        
        putText(Window, iter->first, cv::Point(175 * i, FRAME_TOP_Y - 8),
                FONT_HERSHEY_COMPLEX, 0.6, STRING_COLOR, 1, 8);
        rectangle(Window, Point(175 * i, FRAME_TOP_Y - 1), 
                Point(175 * i + MAXM + 1, FRAME_TOP_Y + MAXN), FRAME_COLOR);
        iter++;
        i++;
    }
    return Window;
}

void Visualization :: Merge(cv::Mat &_visual)
{
    cv::Mat Window(450, 250 + 175 * maps_num, CV_8UC3, Scalar(0, 0, 0));
    cv::Mat imageROI; //Region Of Interest
    imageROI = Window(Rect(250, 0, 175 * maps_num, 450));
    image_maps.copyTo(imageROI);

    _visual = Window;
}

void Visualization :: Show()
{
    bool stop = false;
    int getKey = -1;
    while(!stop)
    {
        getKey = cv::waitKey(25);
        if(getKey == 'q')
        {
            stop = true;
            break;
        }

        image_maps = cv::Mat::zeros(450, 175 * maps_num, CV_8UC3);

        std::map<std::string, cv::Mat> maps;
        maps["ControlMap"] = DrawControlMap(); 
        maps_num = maps.size();
        image_maps = AllMaps(maps);

        cv::Mat visual;
        Merge(visual);

		namedWindow("Visualization", WINDOW_NORMAL);
        imshow("Visualization", visual);    
    }
    cv::destroyWindow("Visualization");
}
