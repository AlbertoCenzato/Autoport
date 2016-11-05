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

bool PositionEstimation::evaluate(vector<Point2f> &cameraSystemPoints, Mat &extrinsicFactors) {

	//valuto posizione con ransac
	if(cameraSystemPoints.size() == 5) {
		ransacPnP(Mat(cameraSystemPoints), extrinsicFactors);

		//Kalman
		kalmanFilter();
		return true;
	}
	return false;
}
