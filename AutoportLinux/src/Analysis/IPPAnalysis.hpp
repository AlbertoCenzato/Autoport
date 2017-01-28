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

#ifndef IPPANALYSIS_HPP_
#define IPPANALYSIS_HPP_

#include <opencv2/opencv.hpp>
#include "../Utils/GenPurpFunc.hpp"
#include "ImgAnalysis.hpp"
#include "PatternAnalysis.hpp"
#include "PositionEstimation.hpp"

class LedDescriptor;
class ImgLoader;	  // forward declaration

/**
 * Image, Pattern and Position Analysis class. Performs this pipeline:
 * load image -> extract led positions -> pattern matching -> compute [R,t]
 * This is the main computation unit of the software.
 * How the image is loaded depends on the ImgLoader object given to the
 * class constructor.
 */
class IPPAnalysis {
public:

	// class constructor, receives the ImgLoader object
	// that should be used to load images
	IPPAnalysis(ImgLoader* loader);

	// class destructor, it doesn't delete *loader
    virtual ~IPPAnalysis();

    /**
     * Performs this pipeline:
     * load image -> extract led positions -> pattern matching -> compute [R,t]
     * @extrinsicFactors: input-output 3x4 [R,t] matrix
     *
     * @return: Result enum stating if the evaluation was successful
     * 			see GenPurpFunc.hpp for Result definition
     */
	Result evaluate(cv::Mat &extrinsicFactors);

	/**
	 * Resets all class members to their default values
	 */
	bool reset();

private:

	// objects used by evaluate() to compute [R,t]
	ImgLoader 		  *loader;
	ImgAnalysis 	   imageAnalyzer;
	PatternAnalysis    patternAnalyzer;
	PositionEstimation positionEstimator;

	// feedback tolerances
	int ROITol = 50;
	int sizeTol;
	int sizeSupTol;
	int sizeInfTol;
	int colorTol;

	/**
	 * Sets "loader" Region Of Interest (ROI) according to points bounding box.
	 * The new ROI is computed adding "ROITol" pixels of tolerance
	 * to the descriptors bounding box.
	 *
	 * @descriptors: points to use
	 * @return: true if "loader" successfully updates its ROI
	 */
	bool updateROI	 (const std::vector<LedDescriptor> &descriptors);

	/**
	 * Sets "loader" resolution according to points size, i.e.:
	 * halves the resolution if size > sizeSupTol,
	 * doubles the resolution if size < sizeInfTol.
	 * Size is computed as the mean size of all the points
	 *
	 * @descriptors: points to use
	 * @returns: true if "loader" successfully updates its resolution
	 */
	bool updateImgRes(const std::vector<LedDescriptor> &descriptors);

	/**
	 * Sets "imageAnalyzer" color filter interval according to points color.
	 * The new filter interval is [averageColor - colorTol, averageColor + colorTol]
	 * for each of the three color channels. The average is computed on all the points.
	 *
	 * @descriptors: points to use
	 * @return: always true
	 */
	bool updateColor (const std::vector<LedDescriptor> &descriptors);

	/**
	 * Sets "loader" resolution and ROI to their default value.
	 *
	 * @return: true if "loader" successfully updates its resolution and ROI
	 */
	bool resetResAndROI();

	/**
	 * Sets "imageAnalyzer" color filter interval to its default value.
	 *
	 * @return: always true
	 */
	bool resetColorInterval();

	/**
	 * Converts points from "loader" reference system (RS) to camera RS.
	 * "loader"->getNextFrame() returns an image which pixels are in a
	 * downscaled and translated RS wrt camera RS. Before points positions can
	 * be used by "positionEstimator" they must be transformed back into camera RS.
	 *
	 * @points: points to transform
	 * @t: translation vector from "loader" RS to camera RS
	 * @resampleMat: upscaling matrix from "loader" RS to camera RS
	 */
	void convertPointsToCamera(std::vector<LedDescriptor> &points, cv::Point2f &t, cv::Mat &resampleMat);

};

#endif /* IPPANALYSIS_HPP_ */
