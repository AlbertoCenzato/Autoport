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

	static Matrix<float,3,4> realWorldPoints;
	//-50, -50,  30, -30,  //1, 3, 7, 5
	//-30,  20, -20, -10,
	//  0,   0,  20,   0;

	vector<Point2f> *points;
	vector<Point2f> *lastSeenPoints;
	int tolerance;
	static const int TOL = 5;



public:

	PositionEstimation(Matrix<float,3,Dynamic> &ledPoints, int tolerance = TOL) {
		realWorldPoints <<  -50, -50,  30, -30,  //1, 3, 7, 5
							-30,  20, -20, -10,
							  0,   0,  20,   0;

		this->tolerance = tolerance;
	}

	Matrix<float,3,4>* evaluate(vector<Point2f>&);

	inline PositionEstimation* setTolerance(int tolerance) {
		this->tolerance = tolerance;
		return this;
	}

	inline void clearAll() {

		return;
	}

private:

	void positionEstimation() {

		return;
	}

	void lowPassFilter() {
		return;
	}

	void kalmanFilter() {
		return;
	}

};




#endif /* POSITIONESTIMATION_H_ */
