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

#include "Utils/LedDescriptor.hpp"

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

float LedDescriptor::euclidDist(const LedDescriptor &ledDesc) const {
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
