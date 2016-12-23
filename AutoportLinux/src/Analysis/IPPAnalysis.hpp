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

#include <opencv2/core.hpp>

#include <fstream>
#include "ImgAnalysis.hpp"
#include "PatternAnalysis.hpp"
#include "PositionEstimation.hpp"

using namespace std;
using namespace cv;

class LedDescriptor;
class ImgLoader;

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
