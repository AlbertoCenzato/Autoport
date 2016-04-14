/*
 * PositionEstimation.cpp
 *
 *  Created on: Mar 14, 2016
 *      Author: alberto
 */

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "PositionEstimation.h"

using namespace std;
using namespace cv;
using namespace Eigen;

Matrix<float,3,4>* PositionEstimation::evaluate(vector<Point2f> &cameraFramePoints) {


	//valuto posizione con Levenberg-Marquardt
	positionEstimation();

	//filtro passa basso (opzionale)
	lowPassFilter();

	//Kalman
	kalmanFilter();

	return new Matrix<float,3,4>;
}


