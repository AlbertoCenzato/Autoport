/*==============================================================================
Software for Autoport project

// Copyright   : Copyright (c) 2016, Alberto Cenzato
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the FreeBSD Project.
//============================================================================ */

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
