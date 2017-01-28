/*==============================================================================
Software for Autoport project

// Copyright   : Copyright (c) 2016, Alberto Cenzato
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the FreeBSD Project.
//============================================================================ */

#ifndef POSITION_ESTIMATION_HPP_
#define POSITION_ESTIMATION_HPP_

#include "../Utils/global_includes.hpp"

extern ofstream stream;

/**
 * PositionEstimation class computes the rotation matrix R and the translation vector t
 * that transform the input points reference system to the target pattern reference system.
 *
 * TODO: this class should be abstract, leaving the implementation of
 * 		 ransacPnP and kalmanFilter to its concrete classes.
 */
class PositionEstimation {

public:

	PositionEstimation();
	~PositionEstimation();

	/**
	 * Computes the rotation matrix R and the translation vector t that
	 * transform the input points reference system to the target pattern reference system.
	 *
	 * @cameraSystemPoints: input points.
	 * @extrinsicFactors: output 3x4 [R,t] matrix.
	 * @return: true if [R,t] is computed without errors.
	 */
	bool evaluate(const std::vector<LedDescriptor> &cameraSystemPoints, cv::Mat &extrinsicFactors);

	/**
	 * Sets the initial suggested position for the iterative algorithm used
	 * by PositionEstimation::ransacPnP() to its default value.
	 *
	 * @return: always true.
	 */
	bool resetInitialPosition();

private:

	float focalX;
	float focalY;

	// TODO: put these "magic numbers" in configuration file
	float cx = 1.3081e+03;	// optical axis coordinates
	float cy = 	 964.6396;

	float h1 =  0.1768;		// lens distortion coefficients
	float h2 = -0.3365;

	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32FC1);  // output rotation vector
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32FC1);  // output translation vector

	cv::Mat cameraMatrix;

	cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_32FC1);

	/**
	 * Registers detected points descriptors on the expected LED pattern
	 * (loaded from configuration file) using an iterative ransac approach.
	 * See OpenCV documentation about solvePnPRansac.
	 *
	 * @descriptors: input detected points
	 * @extrinsicFactors: output [R,t] matrix
	 */
	void ransacPnP(const std::vector<LedDescriptor> &descriptors, cv::Mat &extrinsicFactors);

	/**
	 * Empty function, does nothing. In future will be used to merge
	 * IMU and camera data to improve accuracy
	 */
	void kalmanFilter();

};

#endif
