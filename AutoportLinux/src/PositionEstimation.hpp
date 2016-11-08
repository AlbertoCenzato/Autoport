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
#include <iostream>
#include <fstream>
#include "LedDescriptor.hpp"

using namespace std;
using namespace cv;

extern ofstream stream;

class PositionEstimation {

public:

	PositionEstimation() {
		Settings& settings = Settings::getInstance();
		focalX = (float)settings.focalX;
		focalY = (float)settings.focalY;
		pixelDimension = (float)settings.pixelDimension;

	}

	~PositionEstimation() {	}

	bool evaluate(vector<LedDescriptor> &, Mat &evaluatedPoints);

	PositionEstimation* setPointsToEvaluate(uchar pointsToEvaluate);

private:

	float focalX;
	float focalY;
	float pixelDimension;

	float cx = 1.3081e+03;
	float cy = 964.6396;

	void ransacPnP(vector<LedDescriptor> &ledPoints, Mat &extrinsicFactors) {

		auto rwPoints = Settings::getInstance().realWorldPoints;
		auto objectPoints = vector<Point3f>();
		auto imagePoints  = vector<Point2f>();
		for(int i = 0; i < ledPoints.size(); ++i) {
			if(!ledPoints[i].isEmpty()) {
				objectPoints.push_back(rwPoints[i]);
				imagePoints .push_back(ledPoints[i].position);
			}
		}

		Mat cameraMatrix(Size(3,3), CV_32F);
		cameraMatrix.at<float>(0,0) = 2.0504e+03;
		cameraMatrix.at<float>(1,1) = 2.0513e+03;
		cameraMatrix.at<float>(0,2) = 1.3081e+03;
		cameraMatrix.at<float>(1,2) = 964.6396;
		cameraMatrix.at<float>(2,2) = 1;

		Mat distCoeffs = Mat::zeros(4, 1, CV_64FC1);  // vector of distortion coefficients
		Mat rvec 	   = Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
		Mat tvec 	   = Mat::zeros(3, 1, CV_64FC1);    // output translation vector

		solvePnPRansac(objectPoints,imagePoints,cameraMatrix, vector<float>(0), rvec, tvec);

		Mat rotationMat;
		Rodrigues(rvec,rotationMat);

		for(int i = 0; i < 3; ++i)
			rotationMat.col(i).copyTo(extrinsicFactors.col(i));
		tvec.col(0).copyTo(extrinsicFactors.col(3));
	}


	void kalmanFilter() {
		return;
	}

};
