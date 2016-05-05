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

Matrix<double,3,2>* PositionEstimation::evaluate(vector<Point2f> &cameraSystemPoints) {

	delete this->cameraSystemPoints;
	this->cameraSystemPoints = new vector<Point2f>(numberOfUsedPoints);
	uchar pointsToEvaluate = this->pointsToEvaluate;
	for(int i = 0, count = 0; i < 8; i++) {
		if((pointsToEvaluate & 0x80) != 0) {
			this->cameraSystemPoints->at(count++) = cameraSystemPoints.at(i);
		}
		pointsToEvaluate = pointsToEvaluate << 1;
	}

	//valuto posizione con Levenberg-Marquardt
	levenbergMarquardt();

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


// sets points to use in the computations using the bit array pointsToEvaluate
// every bit represents the point in the corresponding position in the vector
// a value of 1 means that the point must be used, 0 the point must be excluded
PositionEstimation* PositionEstimation::setPointsToEvaluate(uchar pointsToEvaluate) {

	this->pointsToEvaluate = pointsToEvaluate;
	delete currRealWorldSet;
	currRealWorldSet = new vector<Point3d>(8);
	numberOfUsedPoints = 0;
	for(int i = 0; i < 8; i++) {
		if((pointsToEvaluate & 0x80) != 0) {
			currRealWorldSet->at(numberOfUsedPoints++) = realWorldPoints->at(i);
		}
		pointsToEvaluate = pointsToEvaluate << 1;
	}

	return this;
}
