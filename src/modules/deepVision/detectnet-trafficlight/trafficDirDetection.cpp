#include "trafficDirDetection.h"

LIGHTTYPE TraffirDir::JudgeLightType(Mat detected, const bool& isGreen){
	
	//threashold light
	Mat imgRed, imgGreen;
	imgRed.create(detected.rows, detected.cols, CV_8UC1);
	imgGreen.create(detected.rows, detected.cols, CV_8UC1);
	splitImage(detected, imgRed, imgGreen);

	if (isGreen){
		return findLight(imgGreen.clone(), imgGreen, 1);
	}
	else{
		return findLight(imgRed.clone(), imgRed, 1);
	}
}

void TraffirDir::splitImage(Mat img, Mat &imgRed, Mat &imgGreen){
	printf("Start to Split image\n");
	Mat imgYCrCb;
	// Convert to YCrCb color space
	cvtColor(img, imgYCrCb, CV_BGR2YCrCb);
	vector<Mat> planes;
	split(imgYCrCb, planes);
	
	MatIterator_<uchar> it_Cr = planes[1].begin<uchar>(), it_Cr_end = planes[1].end<uchar>();
	MatIterator_<uchar> it_Red = imgRed.begin<uchar>();
	MatIterator_<uchar> it_Green = imgGreen.begin<uchar>();

	// Parameters of brightness
	float gain_red_b = (1 - gain_red_a) * 125;
	float gain_green_b = (1 - gain_green_a) * 125;

	for (; it_Cr != it_Cr_end; ++it_Cr, ++it_Red, ++it_Green) {
		float cr = float(*it_Cr) * gain_red_a + gain_red_b;
		// RED, 145<Cr<470 
		if (cr > thresh_red_min && cr < thresh_red_max)
			*it_Red = 255;
		else
			*it_Red = 0;
		float cg = float(*it_Cr) * gain_green_a + gain_green_b;
		// GREEN£¬95<Cr<110
		if (cg > thresh_green_min && cg < thresh_green_max)
			*it_Green = 255;
		else
			*it_Green = 0;
	}
	printf("Split Image Finished\n");
}


LIGHTTYPE TraffirDir::findLight(Mat srcImg, Mat grayImg, int color){
	printf("Start to find light\n");
	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	
	// conver the image to binary
	threshold(grayImg, threshold_output, 100, 255, THRESH_BINARY);

	imwrite("Threhold_Image.jpg", grayImg);
	/// Find contours
	findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	// Sort contours
	sort(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2){
		return contourArea(c1, false) > contourArea(c2, false);
	});
	/// Approximate contours to polygons + get bounding rects and circles
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect;
	vector<Point2f>center;
	vector<float>radius;
	//vector<LightType> lts;
	vector<float> rats;

	for (int i = 0; i < contours.size(); i++){
		Rect rec = boundingRect(Mat(contours[i]));
		//cerr << "ROI: " << rec << endl;
		cout << "ROI: " << rec << endl;
		if (rec.width > min_size && rec.height > min_size && (float)rec.width / (float)rec.height < w_vs_h && (float)rec.width / (float)rec.height > 1 / w_vs_h){
			Mat tmpImg = srcImg;
			cout << "GRAY: width: " << grayImg.cols << ", height: " << grayImg.rows;
			Mat judgeImg = grayImg(rec);//problematic

			Rect recCen(rec.x + rec.width / 4.0, rec.y + rec.height / 4.0, rec.width / 2.0, rec.height / 2.0);

			Mat judgeImgCen = grayImg(recCen);

			int recw = rec.width;
			int rech = rec.height;

			//all 
			float count1 = countNonZero(judgeImg);

			//up and down
			float countu = countNonZero(judgeImg(Rect(0, 0, recw, rech / 2)));
			float countd = countNonZero(judgeImg(Rect(0, rech / 2, recw, rech / 2)));

			//left and right
			float countl = countNonZero(judgeImg(Rect(0, 0, recw / 2, rech)));
			float countr = countNonZero(judgeImg(Rect(recw / 2, 0, recw / 2, rech)));

			//center
			float count_cen = countNonZero(judgeImgCen);
			float up_down_ratio = countu / countd;
			float down_up_ratio = countd / countu;

			float left_right_ratio = countl / countr;
			float right_left_ratio = countr / countl;

			float rect_size = recw * rech;
			float density = count1 / rect_size;
			float density_cen = count_cen / recCen.area();

			//judegement

			if (density > density_cen){
				return FORWARD;
				cout << "Dir: Forward" << endl;
			}
			else if (left_right_ratio > thresh_left_right_ratio){
				return LEFT;
				cout << "Dir: Left" << endl;
			}
			else if (right_left_ratio > thresh_left_right_ratio){
				return RIGHT;
				cout << "Dir: Right" << endl;
			}
			/*else if (up_down_ratio > thresh_up_down_ratio){
				return FORWARD;
			}*/
			else {
				return FORWARD;
				cout << "Dir: Forward" << endl;
			}
			return FORWARD;
			cout << "Dir: Forward" << endl;
		}
		return UNKNOWN;
	}
	return UNKNOWN;
	printf("Return Error\n");	
}

bool TraffirDir::isCircle(Mat CutImg)
{
	Mat imgYCrCb;
	// Convert to YCrCb color space
	cvtColor(CutImg, imgYCrCb, CV_BGR2YCrCb);

	vector<Mat> planes;
	split(imgYCrCb, planes);

	Mat Cr_clone = planes[1];
	vector<Vec3f> circles;
	HoughCircles(Cr_clone, circles, CV_HOUGH_GRADIENT, 1, Cr_clone.rows / 8, 200, 10, 0, 0); //param for detecting circle

	if (circles.size() != 1)
	{
		return false;
	}
	else
	{
		Point center(cvRound(circles[0][0]), cvRound(circles[0][1]));
		int radius = cvRound(circles[0][2]);

		if (cvRound(circles[0][2]) > 6) // parameter for detetcting circle
		{
			return true;
		}
		else
		{
			return false;
		}
	}
}
