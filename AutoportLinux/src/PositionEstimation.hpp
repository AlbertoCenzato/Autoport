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

	PositionEstimation* setPointsToEvaluate(uchar pointsToEvaluate);

private:

	float focalX;
	float focalY;
	float pixelDimension;

	float cx = 1.3081e+03;
	float cy = 964.6396;

	void ransacPnP(vector<LedDescriptor> &ledPoints, Mat &extrinsicFactors);
	void kalmanFilter();

};

#endif
