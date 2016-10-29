/*
 * LedDescriptor.h
 *
 *  Created on: Oct 29, 2016
 *      Author: alberto
 */

#ifndef LEDDESCRIPTOR_HPP_
#define LEDDESCRIPTOR_HPP_

#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

class LedDescriptor {
public:
	LedDescriptor();
	LedDescriptor(Point2f &position, Scalar &color, float size);
	LedDescriptor(float x, float y, float hue, float saturation, float value, float size);
	virtual ~LedDescriptor();

	float L2Dist  (const LedDescriptor &ledDesc) const;
	float cartDist(const LedDescriptor &ledDesc) const;

	Point2f getPosition() const;
	void    getColor(Scalar &color) const;
	float   getSize() const;

	static Point2f centroid(const vector<LedDescriptor> &descriptors) {
		float x = 0;
		float y = 0;
		const int SIZE = descriptors.size();
		for (int i = 0; i < SIZE; i++) {
			x += descriptors[i].values[0];
			y += descriptors[i].values[1];
		}
		return Point2f(x/SIZE, y/SIZE);
	}

private:
	float values[6]; 	// {x, y, hue, saturation, value, size}
};

#endif /* LEDDESCRIPTOR_HPP_ */
