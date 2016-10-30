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
	int ROItolerance = 50;
	int sizeTolerance;
	int sizeSupTolerance;
	int colorTol;

	void findROI(const vector<LedDescriptor>& descriptors, Rect& roi) {
		const int SIZE = descriptors.size();
		vector<Point2f> points(SIZE);
		for(int i = 0; i < SIZE; ++i)
			points[i] = descriptors[i].getPosition();
		Rect boundBox = boundingRect(points);
		int x = boundBox.x - ROItolerance;
		int y = boundBox.y - ROItolerance;
		int roiWidth  = boundBox.width  + 2*ROItolerance;
		int roiHeight = boundBox.height + 2*ROItolerance;
		roi = Rect(x, y, roiWidth, roiHeight);
	}

	void updateImgRes(const vector<LedDescriptor> &descriptors) {
		const int SIZE = descriptors.size();
		float meanSize = 0;
		for(int i = 0; i < SIZE; ++i)
			meanSize += descriptors[i].getSize();
		meanSize /= SIZE;

		if(meanSize > sizeSupTolerance) {
			loader->halveRes();
			meanSize /= 4;
		}

		imageAnalyzer.setBlobSizeInterval(Interval<int>(meanSize - sizeTolerance,meanSize + sizeTolerance));
	}

	void updateColor(const vector<LedDescriptor> &descriptors) {
		const int SIZE = descriptors.size();
		int meanHue = 0, meanSat = 0, meanVal = 0;
		for(int i = 0; i < SIZE; ++i) {
			Scalar color = descriptors[i].getColor();
			meanHue += color[0];
			meanSat += color[1];
			meanVal += color[2];
		}
		meanHue /= SIZE;
		meanSat /= SIZE;
		meanVal /= SIZE;

		Scalar minCol(meanHue - colorTol, meanSat - colorTol, meanVal - colorTol);
		Scalar maxCol(meanHue + colorTol, meanSat + colorTol, meanVal + colorTol);
		imageAnalyzer.setColorInterval(Interval<Scalar>(minCol,maxCol));
	}

};

#endif /* IPPANALYSIS_HPP_ */
