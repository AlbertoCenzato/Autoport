
#include "stdafx.h"
#include <iostream>
#include "ImgAnalysisFunctions.h"
#include "Functions.h"
#include "Simulations.h"

using namespace std;

//entrypoint function from the main (OpenCVtest2.cpp) 
void run() {

	string imgName = "patternMirko.jpg";
	//vidLedDetection(imgName);
	
	cv::Mat image;
	vector<cv::Point2f> keyPoints = patternMirko(imgLedDetection(imgName,image),image, 15);
	/*
	Eigen::Vector3d pointsCS1;
	Eigen::Vector3d pointsCS2;
	Eigen::Vector3d pointsCS3;
	Eigen::Vector3d pointsCS4;
	pointsCS1 << keyPoints[7].pt.x,  keyPoints[7].pt.y,  0;
	pointsCS2 << keyPoints[8].pt.x,  keyPoints[8].pt.y,  0;
	pointsCS3 << keyPoints[9].pt.x,  keyPoints[9].pt.y,  0;
	pointsCS4 << keyPoints[11].pt.x, keyPoints[11].pt.y, 0;
	pointsCS1.normalize();
	pointsCS2.normalize();
	pointsCS3.normalize();
	pointsCS4.normalize();
	Eigen::Matrix<double, 3, 4> pointsCS;
	pointsCS << pointsCS1, pointsCS2, pointsCS3, pointsCS4;
	Eigen::Matrix<double, 3, 4> pointsWP;
	pointsWP << 120, -120,  120,  150,
				100,  100, -100, -100,
				  0,	0,	  0,    0;

	Eigen::Matrix<double, 3, 4> output = p3p_solver(pointsWP, pointsCS);
	printMatrix(output, 3, 4);
	*/
	getchar();
	return;
}