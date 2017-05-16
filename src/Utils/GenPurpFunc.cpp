/*
 * GenPurpFunc.cpp
 *
 *  Created on: Jan 24, 2017
 *      Author: alberto
 */

#include "GenPurpFunc.hpp"
#include "LedDescriptor.hpp"

using namespace std;
using namespace cv;

string GenPurpFunc::pointVectorToString(const vector<Point2f> &vect) {
	string str = "";
	uint size = vect.size();
	for(uint i = 0; i < size; i++)
		str += "\nPoint " + to_string(i+1) + ": [" + to_string(vect.at(i).x) + ", " + to_string(vect.at(i).y) + "]";
	return str;
}
string GenPurpFunc::pointVectorToString(const vector<KeyPoint> &vect) {
	string str = "";
	uint size = vect.size();
	for(uint i = 0; i < size; i++)
		str += "\nPoint " + to_string(i+1) + ": [" + to_string(vect.at(i).pt.x) + ", " + to_string(vect.at(i).pt.y) + "]";
	return str;
}
string GenPurpFunc::pointVectorToString(const vector<Point3f> &vect) {
	string str = "";
	uint size = vect.size();
	for(uint i = 0; i < size; i++)
		str += "\nPoint " + to_string(i+1) + ": [" + to_string(vect.at(i).x) + ", " + to_string(vect.at(i).y)
		+ ", " + to_string(vect.at(i).z) + "]";
	return str;
}

Point2f GenPurpFunc::normalize(Point2f &p) {
	double norm = sqrt(pow(p.x, 2) + pow(p.y, 2));
	return Point2f(p.x / norm, p.y / norm);
}
Point2d GenPurpFunc::normalize(Point2d &p) {
	double norm = sqrt(pow(p.x, 2) + pow(p.y, 2));
	return Point2d(p.x / norm, p.y / norm);
}
Point3f GenPurpFunc::normalize(Point3f &p) {
	double norm = sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z,2));
	return Point3f(p.x / norm, p.y / norm, p.z/norm);
}
Point3d GenPurpFunc::normalize(Point3d &p) {
	double norm = sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));
	return Point3d(p.x / norm, p.y / norm, p.z / norm);
}

Point2f* GenPurpFunc::findMaxXInVec(vector<Point2f> &vec) {
	Point2f *max = &vec[0];
	for (uint i = 1; i < vec.size(); i++)
		if (max->x < vec[i].x)
			max = &vec[i];
	return max;
}
Point2f* GenPurpFunc::findMaxYInVec(vector<Point2f> &vec) {
	Point2f *max = &vec[0];;
	for (uint i = 1; i < vec.size(); i++)
		if (max->y < vec[i].y)
			max = &vec[i];
	return max;
}
Point2f* GenPurpFunc::findMinXInVec(vector<Point2f> &vec) {
	Point2f *min = &vec[0];;
	for (uint i = 1; i < vec.size(); i++)
		if (min->x > vec[i].x)
			min = &vec[i];
	return min;
}
Point2f* GenPurpFunc::findMinYInVec(vector<Point2f> &vec) {
	Point2f *min = &vec[0];;
	for (uint i = 1; i < vec.size(); i++)
		if (min->y > vec[i].y)
			min = &vec[i];
	return min;
}
/*
template<typename T, typename A>
void GenPurpFunc::removeFromVec(int index, vector<T,A> &vec) {
	const int SIZE =  vec.size();
	if(index < SIZE - 1)
		vec[index] = vec[SIZE - 1];
	vec.pop_back();
}
*/

float GenPurpFunc::distPoint2Point(const Point2f &p1, const Point2f &p2) {
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

float GenPurpFunc::distPoint2Line(const Point2f &point, const Line &line) {
	// FIXME: if m = INF and q = NAN?
			return abs(point.y - (line.m*(point.x) + line.q)) / sqrt(1 + pow(line.m, 2));
}

Point2f GenPurpFunc::centroid(const vector<Point2f> &points) {
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

void GenPurpFunc::drawDetectedPoints(Mat &image, vector<LedDescriptor> &descriptors, const Scalar &color) {
	for(uint i = 0; i < descriptors.size(); ++i)
		circle(image, descriptors[i].position, 30, color, 10);
	return;
}

void GenPurpFunc::numberDetectedPoints(Mat &image, vector<LedDescriptor> &descriptors, const Scalar &color) {
	for(uint i = 0; i < descriptors.size(); ++i)
		putText(image, to_string(i), descriptors[i].position, FONT_HERSHEY_SIMPLEX, 1, color, 4);
	return;
}
