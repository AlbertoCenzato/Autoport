
#include "stdafx.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include "ImgAnalysisFunctions.h"
#include "Functions.h"
#include "Simulations.h"

using namespace std;
using namespace cv;

//entrypoint function from the main (OpenCVtest2.cpp) 
void run() {

	string path = "C:\\Users\\alber\\OneDrive\\Documenti\\Universita\\Progetto Autoport\\Sensori\\foto\\Primo laboratorio\\";
	string imgName = path + "p7d500a30.bmp";
	Mat img = imread(imgName, ImreadModes::IMREAD_COLOR);
	Mat imageThresholded;

	//---Color filtering----
	int iLowH = 80;
	int iHighH = 110;

	int iLowS = 0;
	int iHighS = 255;

	int iLowV = 0;
	int iHighV = 255;

	//int64 start = getTickCount();

	//Convert the captured frame from BGR to HSV
	cvtColor(img, img, COLOR_BGR2HSV);

	//color filtering
	imageThresholded = filterByColor(img, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV));

	//double totalTime = (((double)getTickCount()) - start) / getTickFrequency();

	//---Blob detection---
	SimpleBlobDetector::Params params;
	params.filterByColor = true;
	params.blobColor = 255;
	params.filterByInertia = true;
	params.minInertiaRatio = 0.5;
	params.maxInertiaRatio = 1;
	params.filterByArea = true;
	params.minArea = 50;
	params.maxArea = 700;
	params.filterByConvexity = true;
	params.minConvexity = 0.5;
	params.maxConvexity = 1;
	params.filterByCircularity = true;
	params.minCircularity = 0.5;
	params.maxCircularity = 1;

	vector<Point2f> ledPoints = findBlobs(imageThresholded, params);

	for (int i = 0; i < ledPoints.size(); i++) {
		std::cout << "\nPoint " << i + 1 << ": x[" << ledPoints[i].x << "] y[" << ledPoints[i].y << "]";
	}

	namedWindow("Original", WINDOW_NORMAL);
	imshow("Original", img); //show the original image

	waitKey(25);

	int maxH = 0, maxS = 0, maxV = 0, minH = 255, minS = 255, minV = 255;
	for each(Point2f p in ledPoints) {
		Vec3b color = img.at<Vec3b>(p);
	}
	vector<Point2f> keyPoints = patternMirko(ledPoints, imageThresholded, 10);
	Matrix<double, 3, 4> realPoints;
	realPoints << -50, -50,  30, -30,  //1, 3, 7, 5
				  -30,  20, -20, -10,
					0,   0,  20,   0;
	double focale = 3.46031; //[mm]
	Vector3d p1 = { 1.4 / 1000 * keyPoints[0].x, 1.4 / 1000 * keyPoints[0].y, focale };
	Vector3d p2 = { 1.4 / 1000 * keyPoints[2].x, 1.4 / 1000 * keyPoints[2].y, focale };
	Vector3d p3 = { 1.4 / 1000 * keyPoints[6].x, 1.4 / 1000 * keyPoints[6].y, focale };
	Vector3d p4 = { 1.4 / 1000 * keyPoints[4].x, 1.4 / 1000 * keyPoints[4].y, focale };
	Vector3d translation = { 1.4 / 1000 * 2592 / 2, 1.4 / 1000 * 1944 / 2, 0 };
	Vector3d p1t = (translation - p1);
	Vector3d p2t = (translation - p2);
	Vector3d p3t = (translation - p3);
	Vector3d p4t = (translation - p4);
	p1t.normalize();
	p2t.normalize();
	p3t.normalize();
	p4t.normalize();
	Matrix<double, 3, 4> cameraSystemPoints;
	cameraSystemPoints << p1t, p2t, p3t, p4t;
	Matrix<double, 3, 4> ret = p3p_solver(realPoints, cameraSystemPoints);
	printMatrix(ret, 3, 4);
	getchar();
	return;
}