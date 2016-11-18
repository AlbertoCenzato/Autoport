//Copyright (c) 2016 Alberto Cenzato. All rights reserved.

#pragma once

#include "GenPurpFunc.hpp"
#include "Settings.hpp"

using namespace std;
using namespace cv;

class ImgAnalysis {

	Interval<Scalar> colorInterval;
	Interval<Scalar> defColorInterval;

	//int colorTolerance;
	SimpleBlobDetector::Params params;
	int colorConversion;

	Mat hsvImage;

public:

	ImgAnalysis(const Interval<Scalar> &colorInterval, LedColor ledColor);
	ImgAnalysis();

	~ImgAnalysis() {}

	bool evaluate(Mat &image, vector<LedDescriptor> &points);
	//ImgAnalysis* setColorTolerance	(int);
	ImgAnalysis* setSizeTolerance	(int);
	ImgAnalysis* setSizeSupTolerance(int);
	ImgAnalysis* setColorInterval	(const Interval<Scalar> &colorInterval);
	ImgAnalysis* setBlobSizeInterval(const Interval<int> 	&blobSizeInterval);

	void getColorInterval(Interval<Scalar> &colorInterval);
	void resetColorInterval();

private:

	void constructor(const Interval<Scalar> &colorInterval, LedColor ledColor);

	// Processes the input image (in HSV color space) filtering out (setting to black)
	// all colors which are not in the interval [min,max], the others are set to white.
	// @img: the Mat object (HSV color space) containing the image to process.
	// @min: the lower bound specified in the HSV color space.
	// @max: the upper bound specified in the HSV color space.
	// returns: black and white image as a Mat object.
	Mat filterByColor(const Mat &hsvImg);

	// Finds all color blobs that fit the specified parameters. Blobs which distance
	// from the centroid of the blob set is grater than 2*meanDistance are ignored.
	// @img: image to analyze.
	// @blobParam: parameters to fit.
	// returns: a vector of Point2f containing centroids coordinates of detected blobs.
	int findBlobs(const Mat &colorFilteredImg, vector<LedDescriptor>& ledPoints);

};

