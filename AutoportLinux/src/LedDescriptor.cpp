/*
 * LedDescriptor.cpp
 *
 *  Created on: Oct 29, 2016
 *      Author: alberto
 */

#include "LedDescriptor.hpp"

using namespace std;
using namespace cv;

LedDescriptor::LedDescriptor() {
	size = 0;
}

LedDescriptor::LedDescriptor(Point2f &position, Scalar &color, float size) {
	this->position = Point2f(position);
	this->color	   = Scalar(color);
	this->size 	   = size;
}

LedDescriptor::LedDescriptor(float x, float y, float hue, float saturation, float value, float size) {
	position   = Point2f(x,y);
	color 	   = Scalar(hue,saturation,value);
	this->size = size;
}

LedDescriptor::~LedDescriptor() {}

//TODO: maybe it would be better to give higher weight in the sum to led position
//		maybe not
float LedDescriptor::L2Dist(const LedDescriptor &ledDesc) const {
	float sqSum = 0;
	sqSum += pow(position.x - ledDesc.position.x,2);
	sqSum += pow(position.y - ledDesc.position.y,2);
	sqSum += pow(color[0]   - ledDesc.color[0],  2);
	sqSum += pow(color[1]   - ledDesc.color[1],  2);
	sqSum += pow(color[2]   - ledDesc.color[2],  2);
	sqSum += pow(size	    - ledDesc.size,  	 2);
	return sqrt(sqSum);
}

float LedDescriptor::cartDist(const LedDescriptor &ledDesc) const {
	return sqrt(pow(position.x - ledDesc.position.x, 2) + pow(position.y - ledDesc.position.y, 2));
}

bool LedDescriptor::isEmpty() const {
	return position.x == 0 || position.y == 0;
}

Point2f LedDescriptor::centroid(const vector<LedDescriptor> &descriptors) {
	float x = 0;
	float y = 0;
	const int SIZE = descriptors.size();
	for (int i = 0; i < SIZE; i++) {
		x += descriptors[i].position.x;
		y += descriptors[i].position.y;
	}
	return Point2f(x/SIZE, y/SIZE);
}














