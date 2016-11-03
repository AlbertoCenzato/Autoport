//Copyright (c) 2016 Alberto Cenzato. All rights reserved.

#pragma once

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;


extern string workingDir;

enum Status {
	LOOKING_FOR_TARGET,
	FIRST_LANDING_PHASE,
	SECOND_LANDING_PHASE,

	HOVERING,
	ERROR,
	MANUAL_CONTROL,

	LANDED
};

template<typename varType> struct Interval {

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

enum LedColor {
	RED,
	BLUE
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

	inline void drawDetectedLed(Mat &image, const Point2f &keyPoint, const string &number) {
		putText(image, number, keyPoint, FONT_HERSHEY_SIMPLEX, 1, Scalar(150, 150, 0),4);
		namedWindow("Detected leds", WINDOW_NORMAL);
		imshow("Detected leds", image);
		waitKey(25);
		imwrite(workingDir + "output/detectedLeds.jpg",image);
		return;
	}

}
