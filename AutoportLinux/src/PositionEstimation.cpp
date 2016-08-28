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

void PositionEstimation::evaluate(vector<Point2f> &cameraSystemPoints, Matrix<double,3,2> &evaluatedPoints) {

	delete this->cameraSystemPoints;
	this->cameraSystemPoints = new vector<Point2f>();
	uchar pointsToEvaluate = this->pointsToEvaluate;
	for(uint i = 0; i < 8; i++) {
		if((pointsToEvaluate & 0x80) != 0) {
			this->cameraSystemPoints->push_back(cameraSystemPoints.at(i));
			cout << this->cameraSystemPoints->size() << endl;
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
	Vector3d pos;
	Vector3d angle;
	pos   << lastKnownPositions->front()->x,   lastKnownPositions->front()->y,     lastKnownPositions->front()->z;
	angle << lastKnownPositions->front()->yaw, lastKnownPositions->front()->pitch, lastKnownPositions->front()->roll;
	evaluatedPoints << pos, angle;
	return;
}


// sets points to use in the computations using the bit array pointsToEvaluate
// every bit represents the point in the corresponding position in the vector
// a value of 1 means that the point must be used, 0 the point must be excluded
PositionEstimation* PositionEstimation::setPointsToEvaluate(uchar pointsToEvaluate) {

	this->pointsToEvaluate = pointsToEvaluate;
	numberOfUsedPoints = 0;
	for(uint i = 0; i < 8; i++) {
		if((pointsToEvaluate & 0x80) != 0)
			numberOfUsedPoints++;
		pointsToEvaluate = pointsToEvaluate << 1;
	}

	pointsToEvaluate = this->pointsToEvaluate;
	delete currRealWorldSet;
	currRealWorldSet = new vector<Point3d>();
	for(uint i = 0; i < 8; i++) {
		if((pointsToEvaluate & 0x80) != 0) {
			Point3d &p = realWorldPoints->at(i);
			currRealWorldSet->push_back(p);
		}
		pointsToEvaluate = pointsToEvaluate << 1;
	}

	cout << currRealWorldSet->size() << endl;

	delete pinHoleFunctor;
	pinHoleFunctor = new PinHoleEquations(*currRealWorldSet,focalX, focalY, pixelDimension);

	return this;
}
