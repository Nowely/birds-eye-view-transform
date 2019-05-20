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
	  
    string filename = "video4.mp4"; // Имя файла тут
    VideoCapture capture(filename);
  
    Mat source, source1, destination;

	int alpha_ = 90, beta_ = 90, gamma_ = 90; // Инициализация параметров
	int f_ = 500, dist_ = 500;
	int pxstep_ = 75;
	int image_h = 223;
	int image_w = 900;
	int turn_ = 500;
	Rect Rec(1080 - image_w, 720 - image_h, image_w, image_h);

	namedWindow("Original", 1);
	namedWindow("Result", 1);
	createTrackbar("Alpha", "Result", &alpha_, 180);
	createTrackbar("Beta", "Result", &beta_, 180);
	createTrackbar("Gamma", "Result", &gamma_, 180);
	createTrackbar("Фокус", "Result", &f_, 2000);
	createTrackbar("Дистанция", "Result", &dist_, 2000);
	createTrackbar("Шаг Сетки", "Result", &pxstep_, 150);
	createTrackbar("Поворот", "Result", &turn_, 1000);


	while( true ) {
		
		capture >> source1;		
		rectangle(source1, Rec, Scalar(255), 1, 8, 0); //ROI
		source = source1(Rec);
		imshow("Original", source1);

		resize(source, source,Size(frameWidth, frameHeight));


		double focalLength, dist, alpha, beta, gamma; 

		alpha =((double)alpha_ -90) * PI/180;
		beta =((double)beta_ -90) * PI/180;
		gamma =((double)gamma_ -90) * PI/180;
		focalLength = (double)f_;
		dist = (double)dist_;

		Size image_size = source.size();
		double w = (double)image_size.width, h = (double)image_size.height;
		
		// Проекция матрицы 2D в 3D
		Mat A1 = (Mat_<float>(4, 3)<< 
			1, 0, -w/2,
			0, 1, -h/2,
			0, 0, 0,
			0, 0, 1 );
			
		// Матрица вращения Rx, Ry, Rz
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
		
		// R - итоговая матрица вращения
		Mat R = RX * RY * RZ;

		// T - translation матрица
		Mat T = (Mat_<float>(4, 4) << 
			1, 0, 0, 0,  
			0, 1, 0, 0,  
			0, 0, 1, dist,  
			0, 0, 0, 1); 
		
		// K - матрица параметрова камеры 
		Mat K = (Mat_<float>(3, 4) << 
			focalLength, 0, w/2, 0,
			0, focalLength, h/2, 0,
			0, 0, 1, 0); 


		Mat transformationMat = K * (T * (R * A1));

		warpPerspective(source, destination, transformationMat, image_size, INTER_CUBIC | WARP_INVERSE_MAP);

		int x = 0;
		int	y = 0;
		while (x < destination.cols + 3000)
		{
			line(destination, Point(x - 1000, 0), Point(x - 1000 + turn_ - 500, destination.rows), Scalar(0, 255, 0), 1);
			x += pxstep_;
		}
		while (y < destination.rows + 3000)
		{
			line(destination, Point(0, y - 1000), Point(destination.cols, y - 1000 + turn_ - 500), Scalar(0, 255, 0), 1);
			y += pxstep_;
		}

		imshow("Result", destination);
		

		if (cv::waitKey(5) == 'p')
			while (cv::waitKey(5) != 'p');
		

		//waitKey(100);
	}


	return 0;
}