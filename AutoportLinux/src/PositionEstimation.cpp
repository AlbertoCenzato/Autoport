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

Matrix<double,3,2>* PositionEstimation::evaluate(vector<Point2f> *cameraSystemPoints) {

	this->cameraSystemPoints = cameraSystemPoints;
	//valuto posizione con Levenberg-Marquardt
	positionEstimation();

	//filtro passa basso (opzionale)
	lowPassFilter();

	//Kalman
	kalmanFilter();

	if(lastKnownPositions->size() > MAX_LAST_KNOWN_POSITIONS_SIZE)
		lastKnownPositions->pop_back();
	Matrix<double,3,2> *position = new Matrix<double,3,2>();
	Vector3d pos;
	Vector3d angle;
	pos   << lastKnownPositions->front()->x,   lastKnownPositions->front()->y,     lastKnownPositions->front()->z;
	angle << lastKnownPositions->front()->yaw, lastKnownPositions->front()->pitch, lastKnownPositions->front()->roll;
	*position << pos, angle;
	return position;
}


