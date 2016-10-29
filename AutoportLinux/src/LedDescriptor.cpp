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
	for(int i = 0; i < 6; ++i)
		values[i] = 0;
}

LedDescriptor::LedDescriptor(Point2f &position, Scalar &color, float size) {
	values[0] = position.x;
	values[1] = position.y;
	values[2] = color[0];
	values[3] = color[1];
	values[4] = color[2];
	values[5] = size;
}

LedDescriptor::LedDescriptor(float x, float y, float hue, float saturation, float value, float size) {
	values[0] = x;
	values[1] = y;
	values[2] = hue;
	values[3] = saturation;
	values[4] = value;
	values[5] = size;
}

LedDescriptor::~LedDescriptor() {}

float LedDescriptor::L2Dist(const LedDescriptor &ledDesc) const {
	float sqSum = 0;
	for(int i = 0; i < 6; ++i)
		sqSum += pow(values[i] - ledDesc.values[i],2);
	return sqrt(sqSum);
}

float LedDescriptor::cartDist(const LedDescriptor &ledDesc) const {
	float sqSum = pow(values[0] - ledDesc.values[0],2) + pow(values[1] - ledDesc.values[1],2);
	return sqrt(sqSum);
}

Point2f LedDescriptor::getPosition() const {
	return Point2f(values[0], values[1]);
}

void LedDescriptor::getColor(Scalar &color) const{
	color[0] = values[2];
	color[1] = values[3];
	color[2] = values[4];
}

float LedDescriptor::getSize() const{
	return values[5];
}
















