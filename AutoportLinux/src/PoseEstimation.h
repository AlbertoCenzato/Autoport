/*
 * PoseEstimation.h
 *
 *  Created on: Mar 14, 2016
 *      Author: alberto
 */

#ifndef POSEESTIMATION_H_
#define POSEESTIMATION_H_

#include <Eigen/Dense>

class PoseEstimation {

	Matrix<float,3,Dynamic> realWorldPoints;

	PoseEstimation(Matrix<float,3,Dynamic> realWorldPoints) {
		this.realWorldPoints = realWorldPoints;
	}
};




#endif /* POSEESTIMATION_H_ */
