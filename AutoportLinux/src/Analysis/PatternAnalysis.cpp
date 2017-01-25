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

#include "PatternAnalysis.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <stdlib.h>

extern Status status;

PatternAnalysis::PatternAnalysis() {
	Settings *settings = Settings::getInstance();
	const int SIZE = settings->realWorldPoints.size();
	oldPoints = vector<LedDescriptor>(SIZE);

	// TODO: check if this assumption is correct
	// here the z-component is neglected because it is very small

	pattern = vector<Point2f>(SIZE);
	for(int i = 0; i < SIZE; ++i) {
		pattern[i] = Point2f(settings->realWorldPoints[i].x,
							 settings->realWorldPoints[i].y);
	}
}

PatternAnalysis::~PatternAnalysis() {}

// Led recognition algorithm. Gives a number to every led using the numbering convention
// in patterns' file (see "Sensori" folder in dropbox).
// @points: vector of Point2f of the leds' blob centroid.
// @img: image used to visualize identified leds' position and number.
// @tolerance: tolerance in the alignement (in pixels).
//returns: vector of Point2f ordered with the numbering convention

bool PatternAnalysis::evaluate(vector<LedDescriptor> &descriptors) {

	int blobNumber = descriptors.size();
	bool matchFound = false;

	if(status == Status::LOOKING_FOR_TARGET) {
		if(blobNumber == 5) {
			matchFound = firstPhase(descriptors);
		}
		else {
			cout << "Too much or not enough descriptors!" << endl;
		}
	}
	else if(status == Status::FIRST_LANDING_PHASE) {
		int matched = nearestPoints(descriptors);
		if(matched >= minNumOfMatch)
			matchFound = true;
	}

	return matchFound;
}

// --- private members ---

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
int PatternAnalysis::nearestPoints(vector<LedDescriptor> &ledPoints) {

	const int SIZE = ledPoints.size();
	vector<LedDescriptor> matchedLeds(SIZE);
	int matched = 0;

	for(int i = 0; i < SIZE; ++i) {
		int index = findNearestPoint(oldPoints[i], ledPoints);
		if(index > -1) {
			matchedLeds[i] = ledPoints[index];
			oldPoints[i] = ledPoints[index];
			GenPurpFunc::removeFromVec(index,ledPoints);
			++matched;
		}
	}

	ledPoints.clear();
	ledPoints = matchedLeds;

	return matched;
}

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
bool PatternAnalysis::firstPhase(vector<LedDescriptor> &ledPoints) {

	const int SIZE = ledPoints.size();

	int minIndex1 = 0;
	int minIndex2 = 1;
	float minDist = 100000;
	for(int i = 0; i < SIZE; ++i) {
		for(int j = 0; j < SIZE; j++) {
			if(i != j) {
				float distance = ledPoints[i].cartDist(ledPoints[j]);
				if(distance < minDist) {
					minDist = distance;
					minIndex1 = i;
					minIndex2 = j;
				}
			}
		}
	}

	LedDescriptor ledA = ledPoints[minIndex1];
	LedDescriptor ledB = ledPoints[minIndex2];

	if(minIndex1 == SIZE-1) {
		GenPurpFunc::removeFromVec(minIndex1,ledPoints);
		GenPurpFunc::removeFromVec(minIndex2,ledPoints);
	}
	else {
		GenPurpFunc::removeFromVec(minIndex2,ledPoints);
		GenPurpFunc::removeFromVec(minIndex1,ledPoints);
	}

	Point2f posA = ledA.position;
	Point2f posB = ledB.position;
	Line line(posA,posB);
	int minIndex = 0;
	minDist = 100000;
	for(int i = 0; i < SIZE-2; ++i) {
		float distance = GenPurpFunc::distPoint2Line(ledPoints[i].position,line);
		if(distance < minDist) {
			minDist = distance;
			minIndex = i;
		}
	}

	vector<LedDescriptor> sorted(SIZE);
	if(ledPoints[minIndex].cartDist(ledA) < ledPoints[minIndex].cartDist(ledB)) {
		sorted[2] = ledA;
		sorted[3] = ledB;
	}
	else {
		sorted[2] = ledB;
		sorted[3] = ledA;
	}

	sorted[1] = ledPoints[minIndex];
	GenPurpFunc::removeFromVec(minIndex,ledPoints);

	if(ledPoints[0].cartDist(sorted[1]) < ledPoints[0].cartDist(sorted[3])) {
		sorted[0] = ledPoints[0];
		sorted[4] = ledPoints[1];
	}
	else {
		sorted[0] = ledPoints[1];
		sorted[4] = ledPoints[0];
	}

	ledPoints = sorted;
	oldPoints = sorted;

	return true;
}

int PatternAnalysis::findNearestPoint(const LedDescriptor &point, const vector<LedDescriptor> &vec) {
	int minIndex = -1;
	float minDist  = FLT_MAX;
	const int SIZE = vec.size();
	for(int i = 0; i < SIZE; ++i) {
		float distance = point.L2Dist(vec[i]);
		if(distance < minDist) {
			minDist  = distance;
			minIndex = i;
		}
	}

	if(minDist > maxDistance)
		return -1;

	return minIndex;
}
