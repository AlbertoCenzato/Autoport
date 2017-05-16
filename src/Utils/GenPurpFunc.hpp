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

#ifndef GEN_PURP_FUNC_HPP_
#define GEN_PURP_FUNC_HPP_

#include <stdlib.h>
#include <opencv2/opencv.hpp>

class LedDescriptor;

using namespace std;

enum Status {
	LOOKING_FOR_TARGET,
	FIRST_LANDING_PHASE,
	SECOND_LANDING_PHASE,

	HOVERING,
	ERROR,
	MANUAL_CONTROL,

	LANDED
};

enum Result {
	END,
	SUCCESS,
	FAILURE
};

enum LedColor {
	RED,
	BLUE
};

template<typename varType>
struct Interval {

public:
	varType min;
	varType max;

	Interval<varType>() {	}

	Interval<varType>(varType min, varType max) : min(min), max(max) {}

	//Interval<varType>& Interval<varType>::operator=(Interval<varType> interval) { return interval;}

	string toString() {
		return to_string(min) + " " + to_string(max);
	}
};

struct Position_XYZ_YPR {

public:
	double x, y, z, yaw, pitch, roll;

	Position_XYZ_YPR()
		: x(0), y(0), z(0), yaw(0), pitch(0), roll(0) {}

	Position_XYZ_YPR(double x, double y, double z, double yaw, double pitch, double roll)
		: x(x), y(y), z(z), yaw(yaw), pitch(pitch), roll(roll) {}

	string toString() {
		return "x:" + to_string(x) + "; y:" + to_string(y) + "; z:" + to_string(z)
				+ "; yaw:" + to_string(yaw) + "; pitch:" + to_string(pitch) + "; roll:" + to_string(roll);
	}

};


struct Line {

public:
	float m, q;

	Line(cv::Point2f &p1, cv::Point2f &p2) {
		float dx = p1.x - p2.x;
		if(dx == 0) {
			m = numeric_limits<float>::infinity();
			q = nanf("");
		}
		else {
			m = (p1.y - p2.y) / dx;
			q = p1.y - m*(p1.x);
		}
	}
};


namespace GenPurpFunc {

string pointVectorToString(const vector<cv::Point2f> &vect);
string pointVectorToString(const vector<cv::KeyPoint> &vect);
string pointVectorToString(const vector<cv::Point3f> &vect);

cv::Point2f normalize(cv::Point2f &p);
cv::Point2d normalize(cv::Point2d &p);
cv::Point3f normalize(cv::Point3f &p);
cv::Point3d normalize(cv::Point3d &p);

cv::Point2f* findMaxXInVec(vector<cv::Point2f> &vec);
cv::Point2f* findMaxYInVec(vector<cv::Point2f> &vec);
cv::Point2f* findMinXInVec(vector<cv::Point2f> &vec);
cv::Point2f* findMinYInVec(vector<cv::Point2f> &vec);

template<typename T, typename A>
void removeFromVec(int index, vector<T,A> &vec) {
	const int SIZE =  vec.size();
	if(index < SIZE - 1)
		vec[index] = vec[SIZE - 1];
	vec.pop_back();
}

float distPoint2Point(const cv::Point2f &p1,    const cv::Point2f &p2);
float distPoint2Line (const cv::Point2f &point, const Line &line);

cv::Point2f centroid(const vector<cv::Point2f> &points);

void drawDetectedPoints(cv::Mat &image, vector<LedDescriptor> &descriptors, const cv::Scalar &color);
void numberDetectedPoints(cv::Mat &image, vector<LedDescriptor> &descriptors, const cv::Scalar &color);

}

#endif
