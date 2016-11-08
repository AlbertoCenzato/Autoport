/*
 * PatternAnalysis.h
 *
 *  Created on: Apr 6, 2016
 *      Author: alberto
 */

#pragma once

#include "assert.h"

#include "GenPurpFunc.hpp"
#include "Settings.hpp"
#include "LedDescriptor.hpp"

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
