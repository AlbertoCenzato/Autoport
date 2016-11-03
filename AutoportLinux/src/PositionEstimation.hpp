/*
 * PositionEstimation.h
 *
 *  Created on: Mar 14, 2016
 *      Author: alberto
 */

#pragma once

#include <chrono>
#include "GenPurpFunc.hpp"
#include "Settings.hpp"

using namespace std;
using namespace cv;


class PositionEstimation {

public:

	PositionEstimation() {
		Settings& settings = Settings::getInstance();
		focalX = (float)settings.focalX;
		focalY = (float)settings.focalY;
		pixelDimension = (float)settings.pixelDimension;
	}

	~PositionEstimation() {	}

	bool evaluate(vector<Point2f> &, Mat &evaluatedPoints);

	PositionEstimation* setPointsToEvaluate(uchar pointsToEvaluate);

private:

	float focalX;
	float focalY;
	float pixelDimension;

	float cx = 1.3081e+03;
	float cy = 964.6396;

	void ransacPnP(Mat imagePoints) {
		Mat objectPoints(Settings::getInstance().realWorldPoints);
		Mat cameraMatrix(Size(3,3), CV_32F);
		cameraMatrix.at<float>(0,0) = 2.0504e+03;
		cameraMatrix.at<float>(1,1) = 2.0513e+03;
		cameraMatrix.at<float>(0,2) = 1.3081e+03;
		cameraMatrix.at<float>(1,2) = 964.6396;
		cameraMatrix.at<float>(2,2) = 1;

		cout << "imagePoints:\n" << imagePoints << endl;

		Mat distCoeffs = Mat::zeros(4, 1, CV_64FC1);  // vector of distortion coefficients
		Mat rvec 	   = Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
		Mat tvec 	   = Mat::zeros(3, 1, CV_64FC1);    // output translation vector

		solvePnPRansac(objectPoints,imagePoints,cameraMatrix, vector<float>(0), rvec, tvec);
		cout << "rvec:\n" << rvec;
		cout << "\n\ntvec:\n" << tvec;
	}


	void kalmanFilter() {
		return;
	}

};
