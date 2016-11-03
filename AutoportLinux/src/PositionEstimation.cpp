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

bool PositionEstimation::evaluate(vector<Point2f> &cameraSystemPoints, Mat &evaluatedPoints) {

	//valuto posizione con ransac
	ransacPnP(Mat(cameraSystemPoints));

	//Kalman
	kalmanFilter();

	return true;
}
