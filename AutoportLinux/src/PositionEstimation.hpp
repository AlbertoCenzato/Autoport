/*
 * PositionEstimation.h
 *
 *  Created on: Mar 14, 2016
 *      Author: alberto
 */

#ifndef POSITION_ESTIMATION_HPP_
#define POSITION_ESTIMATION_HPP_

#include <chrono>
#include "GenPurpFunc.hpp"
#include "Settings.hpp"
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

extern ofstream stream;

class PositionEstimation {

public:

	PositionEstimation();
	~PositionEstimation();

	bool evaluate(vector<LedDescriptor> &, Mat &evaluatedPoints);

	bool resetInitialPosition();
	//PositionEstimation* setPointsToEvaluate(uchar pointsToEvaluate);

private:

	float focalX;
	float focalY;
	//float pixelDimension;

	float cx = 1.3081e+03;	// TODO: load the value from config file
	float cy = 	 964.6396;	// TODO: load the value from config file

	float h1 =  0.1768;		// TODO: load the value from config file
	float h2 = -0.3365;		// TODO: load the value from config file

	Mat rvec = Mat::zeros(3, 1, CV_32FC1);  // output rotation vector
	Mat tvec = Mat::zeros(3, 1, CV_32FC1);  // output translation vector

	Mat distCoeffs = Mat::zeros(4, 1, CV_32FC1);

	void ransacPnP(vector<LedDescriptor> &ledPoints, Mat &extrinsicFactors);
	void kalmanFilter();

};

#endif
