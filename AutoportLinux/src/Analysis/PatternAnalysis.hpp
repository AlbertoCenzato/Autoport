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

#ifndef PATTERN_ANALYSIS_HPP_
#define PATTERN_ANALYSIS_HPP_

#include "assert.h"

#include "../Utils/GenPurpFunc.hpp"
#include "../Utils/Settings.hpp"

using namespace std;
using namespace cv;
using namespace flann;
using namespace cvflann;
using namespace GenPurpFunc;

extern string workingDir;

/*
 * This class decides if a given vector of 2D points represents
 * the led pattern or not and, if yes, reorders the points properly
 * as described by the pattern documentation (see Dropbox > AUTOPORT >
 * SENSORI MATERIALE > pattern.docx, "ottava geometria")
 */

class PatternAnalysis {

public:
	PatternAnalysis();
	~PatternAnalysis();

	/*
	 * The only one public method. Receives the points detected from the image
	 * and states if they represent a pattern or not. Decides which
	 * matching algorithm to use based on the moment: if t = 0
	 * uses firstPhase, otherwise uses nearestPoints.
	 *
	 * @ledPoints: points to evaluate
	 * @tolerance: temporarly unused parameter
	 *
	 * @return: true if a match is found, false otherwise
	 */
	bool evaluate(vector<LedDescriptor> &ledPoints);

private:
	vector<LedDescriptor> oldPoints;	// points at time t-1
	vector<Point2f> pattern;	// model of the pattern, do not modify
	float maxDistance = 75;
	int minNumOfMatch = 4;

	/*
	 * WARNING! NOT WORKING by the moment. It was written for another led pattern!
	 * Receives the points detected from the image
	 * and states if they represent a pattern or not.
	 * Used at time t > 0.
	 *
	 * @ledPoints: points to evaluate
	 * @tolerance: temporarly unused parameter
	 *
	 * @return: the number of matched leds
	 */
	int nearestPoints(vector<LedDescriptor> &ledPoints);

	/*
	 * Receives the points detected from the image
	 * and states if they represent a pattern or not.
	 * Used at time t = 0 (the very first frame) or when the drone
	 * has lost track of the leds.
	 *
	 * @ledPoints: points to evaluate
	 * @tolerance: temporarly unused parameter
	 *
	 * @return: true if a match is found, false otherwise
	 */
	bool firstPhase(vector<LedDescriptor> &ledPoints);
	int findNearestPoint(const LedDescriptor &point, const vector<LedDescriptor> &vec);


};

#endif
