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

#ifndef IMGANALYSIS_HPP_
#define IMGANALYSIS_HPP_

#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include "../Utils/GenPurpFunc.hpp"

class LedDescriptor;

/**
 * ImgAnalysis class extracts coordinates, color and size of
 * all the LEDs in an image that match given criteria.
 *
 * WARNING!! At the moment LEDs can be filtered only according
 * to their color; size and shape are disabled.
 *
 * TODO: add other filtering criteria.
 */
class ImgAnalysis {

public:

	/**
	 * Default class constructor
	 */
	ImgAnalysis();

	/**
	 * Class constructor.
	 *
	 * @colorInterval: interval of valid colors for ImgAnalysis::filterByColor()
	 * @ledColor: color of the LEDs to look for
	 */
	ImgAnalysis(const Interval<cv::Scalar> &colorInterval, LedColor ledColor);

	~ImgAnalysis() {}

	/**
	 * Given an image in input returns a vector of descriptors representing
	 * 2D coordinates, color and size of the LEDs in the image. Coordinates are in pixels.
	 * Subsequent calls assume the color of LEDs to be similar to the
	 * color of LEDs detected in the last call. To perform a clean execution
	 * (without "feedback") call ImgAnalysis::resetColorInterval()
	 * after every evaluate call.
	 *
	 * @image: image to evaluate.
	 * @descriptors: descriptors representing LEDs in the image.
	 * @return: always true.	// TODO: could be used in a better way...
	 */
	bool evaluate(cv::Mat &image, std::vector<LedDescriptor> &descriptors);

	// These functions will be used when size "feedback" is ready
	//
	// ImgAnalysis* setSizeTolerance   (int);
	// ImgAnalysis* setSizeSupTolerance(int);

	/**
	 * Sets interval of valid colors for ImgAnalysis::filterByColor()
	 *
	 * @colorInterval: interval of valid colors
	 * @return: a pointer to this object
	 */
	ImgAnalysis* setColorInterval	(const Interval<cv::Scalar> &colorInterval);

	/**
	 * Sets interval of valid blob sizes for ImgAnalysis::findBlobs()
	 *
	 * @blobSizeInterval: interval of valid blob sizes
	 * @return: a pointer to this object
	 */
	ImgAnalysis* setBlobSizeInterval(const Interval<int> &blobSizeInterval);

	/**
	 * Returns the current colorInterval used by ImgAnalysis::filterByColor()
	 */
	void getColorInterval(Interval<cv::Scalar> &colorInterval);

	/**
	 * Sets colorInterval used by ImgAnalysis::filterByColor() to its default value
	 */
	void resetColorInterval();

private:

	Interval<cv::Scalar> colorInterval;
	Interval<cv::Scalar> defColorInterval;

	cv::SimpleBlobDetector::Params params;
	int colorConversion;

	cv::Mat hsvImage;

	/**
	 * Private constructor used by the two public constructors
	 */
	void constructor(const Interval<cv::Scalar> &colorInterval, LedColor ledColor);

	/**
	 * Processes the input image (in HSV color space) filtering out (setting to black)
	 * all colors which are not in the interval [min,max], the others are set to white.
	 *
	 * @hsvImg: the Mat object (HSV color space) containing the image to process.
	 * @return: black and white image as a Mat object.
	 */
	cv::Mat filterByColor(const cv::Mat &hsvImg);

	/*
	 * Finds all color blobs that fit the specified parameters. Blobs which distance
	 * from the centroid of the blob set is grater than 2*meanDistance are ignored.
	 *
	 * @img: image to analyze.
	 * @blobParam: parameters to fit.
	 * @return: a vector of Point2f containing centroids coordinates of detected blobs.
	 */
	int findBlobs(const cv::Mat &bwImage, std::vector<LedDescriptor>& descriptors);

};

#endif  // IMGANALYSIS_HPP_
