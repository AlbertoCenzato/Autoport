/*
 * PositionEstimation.cpp
 *
 *  Created on: Mar 14, 2016
 *      Author: alberto
 */

#include "PositionEstimation.hpp"

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace Eigen;

bool PositionEstimation::evaluate(vector<Point2f> &cameraSystemPoints, Eigen::Matrix<double,3,2> &evaluatedPoints) {

	//valuto posizione con ransac
	ransacPnP(Mat(cameraSystemPoints));

	//Kalman
	kalmanFilter();

	return true;
}
