/*
 * PositionEstimation.cpp
 *
 *  Created on: Mar 14, 2016
 *      Author: alberto
 */

#include "PositionEstimation.hpp"

#include <chrono>
#include "../Utils/GenPurpFunc.hpp"
#include "../Utils/Settings.hpp"
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

//--- public member functions ---

PositionEstimation::PositionEstimation() {
		Settings *settings = Settings::getInstance();
		focalX = (float)settings->focalX;
		focalY = (float)settings->focalY;
		//pixelDimension = (float)settings.pixelDimension;

		resetInitialPosition();

		cameraMatrix = Mat(Size(3,3), CV_32F);
		cameraMatrix.at<float>(0,0) = focalX;
		cameraMatrix.at<float>(1,1) = focalY;
		cameraMatrix.at<float>(0,2) = cx;
		cameraMatrix.at<float>(1,2) = cy;
		cameraMatrix.at<float>(2,2) = 1;

		distCoeffs = Mat::zeros(4, 1, CV_32FC1);
		distCoeffs.at<float>(0,0) = h1;
		distCoeffs.at<float>(1,0) = h2;

	}

PositionEstimation::~PositionEstimation() {}

bool PositionEstimation::evaluate(const vector<LedDescriptor> &cameraSystemPoints, Mat &extrinsicFactors) {

	//valuto posizione con ransac
	if(cameraSystemPoints.size() == 4 || cameraSystemPoints.size() == 5) {
		ransacPnP(cameraSystemPoints, extrinsicFactors);

		//Kalman
		kalmanFilter();

		return true;
	}
	return false;
}

bool PositionEstimation::resetInitialPosition() {
	Settings *settings = Settings::getInstance();
	tvec.at<float>(0,0) = settings->initialPosition.x;
	tvec.at<float>(1,0) = settings->initialPosition.y;
	tvec.at<float>(2,0) = settings->initialPosition.z;

	rvec.at<float>(0,0) = 0;
	rvec.at<float>(1,0) = 0;
	rvec.at<float>(2,0) = 0;

	return true;
}

// --- private member functions ---

void PositionEstimation::ransacPnP(const vector<LedDescriptor> &ledPoints, Mat &extrinsicFactors) {

		auto rwPoints = Settings::getInstance()->realWorldPoints;
		auto objectPoints = vector<Point3f>();
		auto imagePoints  = vector<Point2f>();
		for(uint i = 0; i < ledPoints.size(); ++i) {
			if(!ledPoints[i].isEmpty()) {
				objectPoints.push_back(rwPoints[i]);
				imagePoints .push_back(ledPoints[i].position);
			}
		}

		GenPurpFunc::addNoise(objectPoints, 0, 0.2);

		solvePnPRansac(objectPoints,imagePoints,cameraMatrix, distCoeffs, rvec, tvec);

		Mat rotationMat;
		Rodrigues(rvec,rotationMat);

		for(int i = 0; i < 3; ++i)
			rotationMat.col(i).copyTo(extrinsicFactors.col(i));
		tvec.col(0).copyTo(extrinsicFactors.col(3));
	}


	void PositionEstimation::kalmanFilter() {
		return;
	}
