/*
 * IPPAnalysis.hpp
 *
 *  Created on: Oct 12, 2016
 *      Author: alberto
 */

#ifndef IPPANALYSIS_HPP_
#define IPPANALYSIS_HPP_

#include "ImgLoader.hpp"
#include "ImgAnalysis.hpp"
#include "PatternAnalysis.hpp"
#include "PositionEstimation.hpp"

using namespace std;
using namespace cv;

class IPPAnalysis {
public:
	IPPAnalysis(ImgLoader* loader);
	virtual ~IPPAnalysis();

	bool evaluate(Mat& extrinsicFactors);

private:

	ImgLoader* loader;
	ImgAnalysis imageAnalyzer;
	PatternAnalysis patternAnalyzer;
	PositionEstimation positionEstimator;
	int tol = 50;

	void findROI(const vector<LedDescriptor>& descriptors, Rect& roi) {
		const int SIZE = descriptors.size();
		vector<Point2f> points(SIZE);
		for(int i = 0; i < SIZE; ++i)
			points[i] = descriptors[i].getPosition();
		Rect boundBox = boundingRect(points);
		int x = boundBox.x - tol;
		int y = boundBox.y - tol;
		int roiWidth  = boundBox.width  + 2*tol;
		int roiHeight = boundBox.height + 2*tol;
		roi = Rect(x, y, roiWidth, roiHeight);
	}
};

#endif /* IPPANALYSIS_HPP_ */
