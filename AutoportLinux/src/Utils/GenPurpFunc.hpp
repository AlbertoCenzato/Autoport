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

#include <opencv2/opencv.hpp>
#include "LedDescriptor.hpp"

using namespace cv;
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

	Line(Point2f &p1, Point2f &p2) {
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

	inline string pointVectorToStrng(const vector<Point2f> &vect) {
		string str = "";
		uint size = vect.size();
		for(uint i = 0; i < size; i++)
			str += "\nPoint " + to_string(i+1) + ": [" + to_string(vect.at(i).x) + ", " + to_string(vect.at(i).y) + "]";
		return str;
	}
	inline string pointVectorToString(const vector<KeyPoint> &vect) {
		string str = "";
		uint size = vect.size();
		for(uint i = 0; i < size; i++)
			str += "\nPoint " + to_string(i+1) + ": [" + to_string(vect.at(i).pt.x) + ", " + to_string(vect.at(i).pt.y) + "]";
		return str;
	}
	inline string pointVectorToString(const vector<Point3f> &vect) {
		string str = "";
		uint size = vect.size();
		for(uint i = 0; i < size; i++)
			str += "\nPoint " + to_string(i+1) + ": [" + to_string(vect.at(i).x) + ", " + to_string(vect.at(i).y)
				+ ", " + to_string(vect.at(i).z) + "]";
		return str;
	}

	inline Point2f normalize(Point2f &p) {
		double norm = sqrt(pow(p.x, 2) + pow(p.y, 2));
		return Point2f(p.x / norm, p.y / norm);
	}
	inline Point2d normalize(Point2d &p) {
		double norm = sqrt(pow(p.x, 2) + pow(p.y, 2));
		return Point2d(p.x / norm, p.y / norm);
	}
	inline Point3f normalize(Point3f &p) {
		double norm = sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z,2));
		return Point3f(p.x / norm, p.y / norm, p.z/norm);
	}
	inline Point3d normalize(Point3d &p) {
		double norm = sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));
		return Point3d(p.x / norm, p.y / norm, p.z / norm);
	}

	inline Point2f* findMaxXInVec(vector<Point2f> &vec) {
		Point2f *max = &vec[0];
		for (uint i = 1; i < vec.size(); i++)
			if (max->x < vec[i].x)
				max = &vec[i];
		return max;
	}
	inline Point2f* findMaxYInVec(vector<Point2f> &vec) {
		Point2f *max = &vec[0];;
		for (uint i = 1; i < vec.size(); i++)
			if (max->y < vec[i].y)
				max = &vec[i];
		return max;
	}
	inline Point2f* findMinXInVec(vector<Point2f> &vec) {
		Point2f *min = &vec[0];;
		for (uint i = 1; i < vec.size(); i++)
			if (min->x > vec[i].x)
				min = &vec[i];
		return min;
	}
	inline Point2f* findMinYInVec(vector<Point2f> &vec) {
		Point2f *min = &vec[0];;
		for (uint i = 1; i < vec.size(); i++)
			if (min->y > vec[i].y)
				min = &vec[i];
		return min;
	}

	template<typename T, typename A>
	inline void removeFromVec(int index, vector<T,A> &vec) {
		const int SIZE =  vec.size();
		if(index < SIZE - 1)
			vec[index] = vec[SIZE - 1];
		vec.pop_back();
	}

	inline float distPoint2Point(const Point2f &p1, const Point2f &p2) {
		return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
	}

	inline float distPoint2Line(const Point2f &point, const Line &line) {
		// FIXME: if m = INF and q = NAN?
		return abs(point.y - (line.m*(point.x) + line.q)) / sqrt(1 + pow(line.m, 2));
	}

	inline Point2f centroid(const vector<Point2f> &points) {
		float x = 0;
		float y = 0;
		const int SIZE = points.size();
		for (int i = 0; i < SIZE; i++) {
			Point2f p = points.at(i);
			x += p.x;
			y += p.y;
		}
		return Point2f(x/SIZE, y/SIZE);
	}

	inline void drawDetectedPoints(Mat &image, vector<LedDescriptor> &descriptors, const Scalar &color) {
		for(uint i = 0; i < descriptors.size(); ++i)
			circle(image, descriptors[i].position, 30, color, 10);
		return;
	}

	inline void numberDetectedPoints(Mat &image, vector<LedDescriptor> &descriptors, const Scalar &color) {
		for(uint i = 0; i < descriptors.size(); ++i)
			putText(image, to_string(i), descriptors[i].position, FONT_HERSHEY_SIMPLEX, 1, color, 4);
		return;
	}

}

#endif