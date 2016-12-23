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

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

extern ofstream stream;

class LedDescriptor;

class PositionEstimation {

public:

	PositionEstimation();
	~PositionEstimation();

	bool evaluate(const vector<LedDescriptor> &, Mat &evaluatedPoints);

	bool resetInitialPosition();
	//PositionEstimation* setPointsToEvaluate(uchar pointsToEvaluate);

private:

	float focalX;
	float focalY;
	//float pixelDimension;

	float cx = 1.3081e+03;	// TODO: load the value from config file
	float cy = 	 964.6396;	// TODO: load the value from config file

	float h1 =  0.1768;		// TODO: load the value from config file
	float h2 = -0.3365;		// TODO: load the value from config file

	Mat rvec = Mat::zeros(3, 1, CV_32FC1);  // output rotation vector
	Mat tvec = Mat::zeros(3, 1, CV_32FC1);  // output translation vector

	Mat cameraMatrix;

	Mat distCoeffs = Mat::zeros(4, 1, CV_32FC1);

	void ransacPnP(const vector<LedDescriptor> &ledPoints, Mat &extrinsicFactors);
	void kalmanFilter();

};

#endif
