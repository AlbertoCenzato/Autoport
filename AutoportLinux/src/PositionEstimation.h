/*
 * PositionEstimation.h
 *
 *  Created on: Mar 14, 2016
 *      Author: alberto
 */

#ifndef POSITIONESTIMATION_H_
#define POSITIONESTIMATION_H_

using namespace std;
using namespace cv;
using namespace Eigen;

class PositionEstimation {

	Matrix<float,3,Eigen::Dynamic> realWorldPoints;
	vector<Point2f> lastSeenPoints;
	int tolerance;
	static const int TOL = 5;



public:

	PositionEstimation(Matrix<float,3,Dynamic> realWorldPoints, int tolerance = TOL) {
		this->realWorldPoints = realWorldPoints;
		this->tolerance = tolerance;
	}

	Matrix<float,3,4> evaluate(vector<Point2f>);

	inline PositionEstimation* setTolerance(int tolerance) {
		this->tolerance = tolerance;
		return this;
	}

	inline void clearAll() {

		return;
	}

};




#endif /* POSITIONESTIMATION_H_ */
