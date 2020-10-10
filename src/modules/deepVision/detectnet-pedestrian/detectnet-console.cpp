/*
 * Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
#include <zcm/zcm-cpp.hpp>
#include "structOBJECTLIST.hpp"

#include "detectNet.h"
#include "loadImage.h"
#include <fstream>
#include <unistd.h>
#include "cudaMappedMemory.h"

#include <pylon/PylonIncludes.h>
#include <sys/time.h>
#include <time.h>

#include <iostream>
using namespace Pylon;
using namespace cv;
using namespace std;

//#define USBCamera
//#define DEBUG

uint64_t current_timestamp() {
    struct timeval te; 
    gettimeofday(&te, NULL); // get current time
    return te.tv_sec*1000LL + te.tv_usec/1000; // caculate milliseconds
}

inline string getCurrentTime()
{
  time_t timep;
  time (&timep);
  char tmp[64];
  strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S",localtime(&timep) );
  return tmp;
}

int XMLRead(string& name, Mat& H);
int perspectiveTransformation(const int& x,const int& y, Mat& H, float& tx, float& ty);

// Number of images to be grabbed.
static const uint32_t c_countOfImagesToGrab = 10000000;

// main entry point
int main( int argc, char** argv )
{
	zcm::ZCM zcm;
        if (!zcm.good()){
                exit(-1);
        }
	ofstream Logger("log_peds.txt");
	Logger << getCurrentTime().c_str() << "System Start" << endl;
	string xmlName = "TiEV_HOMO_2018.yml";
	Mat H;
	XMLRead(xmlName, H);
	Logger << "H: " << H << endl;
	printf("detectnet-console\n  args (%i):  ", argc);
	
	for( int i=0; i < argc; i++ )
		printf("%i [%s]  ", i, argv[i]);
		
	printf("\n\n");
	
	
	// retrieve filename argument
	if( argc < 2 )
	{
		printf("detectnet-console:   input image filename required\n");
		return 0;
	}
	
	const char* imgFilename = argv[1];
	

	// create detectNet
	detectNet* net = detectNet::Create(argc, argv);

	if( !net )
	{
		printf("detectnet-console:   failed to initialize detectNet\n");
		return 0;
	}

	net->EnableProfiler();
	
	// alloc memory for bounding box & confidence value output arrays
	const uint32_t maxBoxes = net->GetMaxBoundingBoxes();		printf("maximum bounding boxes:  %u\n", maxBoxes);
	const uint32_t classes  = net->GetNumClasses();
	
	float* bbCPU    = NULL;
	float* bbCUDA   = NULL;
	float* confCPU  = NULL;
	float* confCUDA = NULL;
	
	if( !cudaAllocMapped((void**)&bbCPU, (void**)&bbCUDA, maxBoxes * sizeof(float4)) ||
	    !cudaAllocMapped((void**)&confCPU, (void**)&confCUDA, maxBoxes * classes * sizeof(float)) )
	{
		printf("detectnet-console:  failed to alloc output memory\n");
		return 0;
	}
	
#ifdef USBCamera	
	VideoCapture cap("output.avi"); // open the default camera
	if(!cap.isOpened()){  // check if we succeeded
        	cerr << "Video load error" << endl;
		return -1;
    	}
#else
	cout << "Load Camera: " << (int)(atoi(argv[1])) << endl;
	PylonInitialize();
	
	CTlFactory& tlFactory = CTlFactory::GetInstance();
	DeviceInfoList_t devices;
        if ( tlFactory.EnumerateDevices(devices) == 0 )
        {
		cerr << "No Basler Camera" << endl;
        	Logger << getCurrentTime().c_str() << " Not connected with basler Camera"  << endl;
		exit(-1);
        }
	//cout << "Camera Device: " << devices[0] << endl;

	//CInstantCamera camera( CTlFactory::GetInstance().CreateFirstDevice());
	CInstantCamera camera( CTlFactory::GetInstance().CreateDevice(devices[(int)(atoi(argv[1]))]));

	
	//CInstantCameraArray camera;
	//camera.Attach( tlFactory.CreateDevice(devices[i]));
	

        // Print the model name of the camera.
        Logger << getCurrentTime().c_str() << " Using device " << camera.GetDeviceInfo().GetModelName() << endl;
	sleep(5);

	camera.MaxNumBuffer = 5;
	// Start the grabbing of c_countOfImagesToGrab images.
        // The camera device is parameterized with a default configuration which
        //// sets up free-running continuous acquisition.
        camera.StartGrabbing( c_countOfImagesToGrab);

        //// This smart pointer will receive the grab result data.
        CGrabResultPtr ptrGrabResult;

        CImageFormatConverter fc;
        fc.OutputPixelFormat = PixelType_BGR8packed;
#endif

	Logger << getCurrentTime().c_str() << ", Video Start" << endl;	
	long long frameNum = 0;

#ifdef USBCamera
	while (1)
#else
	while ( camera.IsGrabbing())
#endif
        {
		//sleep(5);
		// load image from file on disk
		float* imgCPU    = NULL;
		float* imgCUDA   = NULL;
		int    imgWidth  = 0;
		int    imgHeight = 0;
		
		frameNum++;
		string ImageSavePath = "pedestrianImg/" + to_string(frameNum) + ".jpg";
#ifdef USBCamera
		Mat frame;
		cap >> frame;
		//if(frameNum % 2 == 0){
		//	frame = imread("35.jpg");
		//}
		//else{
		//	frame = imread("31.jpg");
		//}
#else
		// Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            	camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);
#endif
            	// Image grabbed successfully?
#ifdef USBCamera
		if( 1 )
#else
            	if (ptrGrabResult->GrabSucceeded())
#endif
            	{
#ifdef USBCamera
			cv::Mat cv_img = frame;
#else
            	    	// Access the image data.
            	    	cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
            	    	cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;
            	    	const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
            	    	cout << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0] << endl << endl;

            	    	CPylonImage image;
            	    	fc.Convert(image, ptrGrabResult);
            	    	cv::Mat cv_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3,(uint8_t*)image.GetBuffer());
            	    	//cv::imwrite("balserImage.jpg", cv_img);
			cv::flip(cv_img, cv_img, -1);
#endif
			if( !loadImageRGBAMat(cv_img, (float4**)&imgCPU, (float4**)&imgCUDA, &imgWidth, &imgHeight) )
			{
				Logger << getCurrentTime().c_str() << ", failed to load image";
				return 0;
			}
			// classify image
			int numBoundingBoxes = maxBoxes;
			
			printf("detectnet-console:  beginning processing network (%zu)\n", current_timestamp());
			clock_t startTime = clock();
			const bool result = net->Detect(imgCUDA, imgWidth, imgHeight, bbCPU, &numBoundingBoxes, confCPU);
			clock_t endTime = clock();
			Logger << getCurrentTime().c_str() << ", Net Time: "  << (double)(endTime - startTime)/CLOCKS_PER_SEC << endl;
			//printf("Net Time: %lf\n", (double)(endTime - startTime)/CLOCKS_PER_SEC);

			printf("detectnet-console:  finished processing network  (%zu)\n", current_timestamp());

			if( !result )
				Logger << getCurrentTime().c_str() << ", detectnet-console:  failed to classify";
			else    // if the user supplied an output filename
			{
				Logger << getCurrentTime().c_str() << ", " << numBoundingBoxes <<  " bounding boxes detected" << endl;
				
				int lastClass = 0;
				int lastStart = 0;
				
				structOBJECTLIST pedestrianMsg;
				pedestrianMsg.count = numBoundingBoxes;
				pedestrianMsg.data_source = 2;

				for( int n=0; n < numBoundingBoxes; n++ )
				{
					const int nc = confCPU[n*2+1];
					float* bb = bbCPU + (n * 4);
					float ncoverage = confCPU[n*2];
					if(ncoverage < 0.9){
						//continue;
					}	
				
					printf("bounding box %i   (%f, %f)  (%f, %f)  w=%f  h=%f\n", n, bb[0], bb[1], bb[2], bb[3], bb[2] - bb[0], bb[3] - bb[1]); 
#ifdef DEBUG
					Logger << "Bounding Box[" << n << "]: " << bb[0] << ", " << bb[1] << ", " << bb[2] << ", " << bb[3] << ", ? " << nc << endl;

					if( nc != lastClass || n == (numBoundingBoxes - 1) )
					{
						if( !net->DrawBoxes(imgCUDA, imgCUDA, imgWidth, imgHeight, bbCUDA + (lastStart * 4), (n - lastStart) + 1, lastClass) )
							printf("detectnet-console:  failed to draw boxes\n");
							
						lastClass = nc;
						lastStart = n;
					}
#endif
					float pos_x = (bb[2] + bb[0])/2;
					float pos_y = bb[3];
					if(pos_y > 1024){
						pos_y = 1024;
					}
					if(pos_x < 0){
						pos_x = 0;
					}
					else if(pos_x > 1280){
						pos_x = 1280;
					}
					float tx, ty;
				
					//Logger << getCurrentTime().c_str() << " Pos:(" << pos_x	 << ", " << pos_y << ")" << endl;
					perspectiveTransformation(pos_x, pos_y, H, tx, ty);

					Logger << getCurrentTime().c_str() << " Pos: (" << pos_x << ", " << pos_y << "), Transformation pos: (" << tx << ", " << ty << ")" << endl;
					//pedestrianMsg.obj[n].type = 2;
					//pedestrianMsg.obj[n].x = tx;
					//pedestrianMsg.obj[n].y = ty;
#ifdef DEBUG
					// save image to disk
					if( !saveImageRGBA(ImageSavePath.c_str(), (float4*)imgCPU, imgWidth, imgHeight, 255.0f) )
						cerr << "detectnet-console:  failed saving image" << endl; 
					else{	
						Logger << getCurrentTime().c_str() << ", detectnet-console:  successfully wrote image to " << ImageSavePath.c_str() << endl;
				
					}
#endif
				}

				zcm.publish("OBJECTLIST", &pedestrianMsg);
				
				CUDA(cudaThreadSynchronize());
				if(frameNum % 25 == 0){
					
					if( !saveImageRGBA(ImageSavePath.c_str(), (float4*)imgCPU, imgWidth, imgHeight, 255.0f) )
						cerr << "detectnet-console:  failed saving image" << endl; 
					else{	
						Logger << getCurrentTime().c_str() << ", Time Out, detectnet-console:  successfully wrote image to " << ImageSavePath.c_str() << endl;
				
					}
				}
			}
		}
		CUDA(cudaFreeHost(imgCPU));
	}
	
	printf("\nshutting down...\n");
	//CUDA(cudaFreeHost(imgCPU));
	delete net;
	return 0;
}


int XMLRead(string& name, Mat& H){
	FileStorage fs(name.c_str(), FileStorage::READ);
	if(!fs.isOpened()){
		cout << "The intri file cannot be locateded!" << endl;
		exit(-1);
	}
	fs["Homography_cam1"] >> H;
	fs.release();
	return 0;
}

int perspectiveTransformation(const int& x,const int& y, Mat& H, float& tx, float& ty){
	Mat real_pt = H * (Mat_<double>(3, 1) << x, y, 1);

	real_pt.at<double>(0, 0) /= real_pt.at<double>(2, 0);
	real_pt.at<double>(1, 0) /= real_pt.at<double>(2, 0);
	//real_pt.at<double>(2, 0) /= real_pt.at<double>(2, 0);
	cerr << " the realpt: " << real_pt << endl;
	tx = real_pt.at<double>(0, 0);
	ty = real_pt.at<double>(1, 0);
	return 0;
}
