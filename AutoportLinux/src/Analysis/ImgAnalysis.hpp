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

#pragma once

#include "../Utils/GenPurpFunc.hpp"
#include "../Utils/Settings.hpp"

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

