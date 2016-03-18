/*
 * PoseEstimation.h
 *
 *  Created on: Mar 14, 2016
 *      Author: alberto
 */

#ifndef POSEESTIMATION_H_
#define POSEESTIMATION_H_

using namespace std;
using namespace cv;
using namespace Eigen;

class PoseEstimation {

	Matrix<float,3,Eigen::Dynamic> realWorldPoints;
	vector<Point2f> lastSeenPoints;
	int tolerance;
	static const int TOL = 5;



public:

	PoseEstimation(Matrix<float,3,Dynamic> realWorldPoints, int tolerance = TOL) {
		this->realWorldPoints = realWorldPoints;
		this->tolerance = tolerance;
	}
	Matrix<float,3,4> evaluate(vector<Point2f>);
	void setTolerance(int tolerance) { this->tolerance = tolerance; }
	void clearAll();


};




#endif /* POSEESTIMATION_H_ */
