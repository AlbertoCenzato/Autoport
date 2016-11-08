/*
 * PositionEstimation.cpp
 *
 *  Created on: Mar 14, 2016
 *      Author: alberto
 */

#include "PositionEstimation.hpp"

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

bool PositionEstimation::evaluate(vector<LedDescriptor> &cameraSystemPoints, Mat &extrinsicFactors) {


	//valuto posizione con ransac
	if(cameraSystemPoints.size() == 4 || cameraSystemPoints.size() == 5) {
		ransacPnP(cameraSystemPoints, extrinsicFactors);

		//Kalman
		kalmanFilter();
		return true;
	}
	return false;
}
