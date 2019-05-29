#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include "opencv2/opencv_modules.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/video/tracking.hpp"
#include <Windows.h>

using namespace std;
using namespace cv;
#define PI 3.1415926


int frameWidth = 800;
int frameHeight = 450;

int main() {
	  
    string filename = "video4.mp4"; // File name
    VideoCapture capture(filename);
  
    Mat source, source1, destination;

	//Initialization of parameters
	int alpha_ = 90, beta_ = 90, gamma_ = 90; 
	int f_ = 500, dist_ = 500;
	int pxstep_ = 50;
	int image_h = 223;
	int image_w = 900;
	int turn_ = 45;

	Rect Rec(1280 - 200 - image_w, 720 - image_h, image_w, image_h); //ROI. for video 6,7 need change roi

	namedWindow("Original", 1);
	namedWindow("Result", 1);
	createTrackbar("Alpha", "Result", &alpha_, 180);
	createTrackbar("Beta", "Result", &beta_, 180);
	createTrackbar("Gamma", "Result", &gamma_, 180);
	createTrackbar("Focus", "Result", &f_, 2000);
	createTrackbar("Distance", "Result", &dist_, 2000);
	createTrackbar("Step grid", "Result", &pxstep_, 150);
	createTrackbar("Angle grid", "Result", &turn_, 180);


	while( true ) {
		
		capture >> source1;		
		rectangle(source1, Rec, Scalar(255), 1, 8, 0); //ROI
		source = source1(Rec);
		imshow("Original", source1);

		resize(source, source,Size(frameWidth, frameHeight));


		double focalLength, dist, alpha, beta, gamma, f; 

		alpha =((double)alpha_ -90) * PI/180;
		beta =((double)beta_ -90) * PI/180;
		gamma =((double)gamma_ -90) * PI/180;
		focalLength = (double)f_;
		dist = (double)dist_;
		f = ((double)turn_ - 90) * PI / 180;

		Size image_size = source.size();
		double w = (double)image_size.width, h = (double)image_size.height;
		
		// matrix projection 2D â 3D
		Mat A1 = (Mat_<float>(4, 3)<< 
			1, 0, -w/2,
			0, 1, -h/2,
			0, 0, 0,
			0, 0, 1 );
			
		// rotation matrix Rx, Ry, Rz
		Mat RX = (Mat_<float>(4, 4) << 
			1, 0, 0, 0,
			0, cos(alpha), -sin(alpha), 0,
			0, sin(alpha), cos(alpha), 0,
			0, 0, 0, 1);

		Mat RY = (Mat_<float>(4, 4) << 
			cos(beta), 0, -sin(beta), 0,
			0, 1, 0, 0,
			sin(beta), 0, cos(beta), 0,
			0, 0, 0, 1);

		Mat RZ = (Mat_<float>(4, 4) << 
			cos(gamma), -sin(gamma), 0, 0,
			sin(gamma), cos(gamma), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1);
		
		// R - total rotation matrix
		Mat R = RX * RY * RZ;

		// T - translation matrix
		Mat T = (Mat_<float>(4, 4) << 
			1, 0, 0, 0,  
			0, 1, 0, 0,  
			0, 0, 1, dist,  
			0, 0, 0, 1); 
		
		// K - camera parameter matrix 
		Mat K = (Mat_<float>(3, 4) << 
			focalLength, 0, w/2, 0,
			0, focalLength, h/2, 0,
			0, 0, 1, 0); 


		Mat transformationMat = K * (T * (R * A1));

		warpPerspective(source, destination, transformationMat, image_size, INTER_CUBIC | WARP_INVERSE_MAP);

		// grid
		int x = -500;
		int y = -1000;
		while (x < destination.cols + 3000)
		{
			line(destination, 
				Point(cos(f)* x - sin(f)*1500, -sin(f)*x - cos(f)*1500), 
				Point(cos(f)* x + sin(f)* destination.rows*3, -sin(f) * x + cos(f) * destination.rows*3),
				Scalar(0, 255, 0), 1);
			x += pxstep_;
		}
		while (y < destination.rows + 3000)
		{
			line(destination, 
				Point(-cos(f) * 1500 + sin(f) * y, +sin(f)*1500 + cos(f)*y),
				Point(cos(f) * destination.cols*2 + sin(f) * y, -sin(f) * destination.cols*2 + cos(f) * y),
				Scalar(0, 255, 0), 1);
			y += pxstep_;			
		}


		imshow("Result", destination);
		
		//Pause
		if (cv::waitKey(5) == 'p')
			while (cv::waitKey(5) != 'p');
		

		//waitKey(100);
	}


	return 0;
}