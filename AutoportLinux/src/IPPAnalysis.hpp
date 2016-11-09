/*
 * IPPAnalysis.hpp
 *
 *  Created on: Oct 12, 2016
 *      Author: alberto
 */

#ifndef IPPANALYSIS_HPP_
#define IPPANALYSIS_HPP_


#include "GenPurpFunc.hpp"
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

	Result evaluate(Mat& extrinsicFactors);
	bool   reset();

private:

	ImgLoader* loader;
	ImgAnalysis imageAnalyzer;
	PatternAnalysis patternAnalyzer;
	PositionEstimation positionEstimator;
	int ROITol = 50;
	int sizeTol;
	int sizeSupTol;
	int sizeInfTol;
	int colorTol;

	bool updateROI	 (const vector<LedDescriptor> &descriptors);
	bool updateImgRes(const vector<LedDescriptor> &descriptors);
	bool updateColor (const vector<LedDescriptor> &descriptors);
	bool resetROIandRes();
	bool resetColor();
	void convertPointsToCamera(vector<LedDescriptor> &points, Point2f &t, Mat &resampleMat);

};

#endif /* IPPANALYSIS_HPP_ */
