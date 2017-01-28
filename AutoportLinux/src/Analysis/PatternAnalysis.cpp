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

#include <opencv2/opencv.hpp>
#include "PatternAnalysis.hpp"
#include "../Utils/Settings.hpp"
#include "../Utils/LedDescriptor.hpp"
#include "../Utils/GenPurpFunc.hpp"

using namespace std;
using namespace cv;

extern Status status;

// ----- public members -----

PatternAnalysis::PatternAnalysis() {
	Settings *settings = Settings::getInstance();
	const int SIZE = settings->realWorldPoints.size();
	oldDescriptors = vector<LedDescriptor>(SIZE);

	pattern = vector<Point2f>(SIZE);							// pattern description loaded
	for(int i = 0; i < SIZE; ++i) {								// from config file is never
		pattern[i] = Point2f(settings->realWorldPoints[i].x,	// used by PatternAnalysis for
							 settings->realWorldPoints[i].y);	// the moment
	}
}

PatternAnalysis::~PatternAnalysis() {}

bool PatternAnalysis::evaluate(vector<LedDescriptor> &descriptors) {

	int blobNumber = descriptors.size();
	bool matchFound = false;

	// if "oldDescriptors" is empty or outdated perform a complete match...
	if(status == Status::LOOKING_FOR_TARGET) {
		// if descriptors aren't the same number of the pattern points
		// skip all evaluation and return false
		//
		// FIXME: this isn't robust, it's too specific
		if(blobNumber == 5) {		// TODO: put the "magic number" somewhere else
			matchFound = firstPhase(descriptors);
		}
		else {
			cout << "Too much or not enough descriptors!" << endl;
		}
	}
	// ... otherwise perform a quick nearest neighbor match
	else if(status == Status::FIRST_LANDING_PHASE) {
		int matched = secondPhase(descriptors);
		if(matched >= minNumOfMatch)
			matchFound = true;
	}

	return matchFound;
}

// ----- private members -----

//  ----------------------------------------------------------------
// | TODO: This algorithm is stupid, too specific, bad looking crap |
//  ----------------------------------------------------------------
bool PatternAnalysis::firstPhase(vector<LedDescriptor> &descriptors) {

	const int SIZE = descriptors.size();

	int minIndex1 = 0;
	int minIndex2 = 1;
	float minDist = 100000;
	for(int i = 0; i < SIZE; ++i) {
		for(int j = 0; j < SIZE; j++) {
			if(i != j) {
				float distance = descriptors[i].cartDist(descriptors[j]);
				if(distance < minDist) {
					minDist = distance;
					minIndex1 = i;
					minIndex2 = j;
				}
			}
		}
	}

	LedDescriptor ledA = descriptors[minIndex1];
	LedDescriptor ledB = descriptors[minIndex2];

	if(minIndex1 == SIZE-1) {
		GenPurpFunc::removeFromVec(minIndex1,descriptors);
		GenPurpFunc::removeFromVec(minIndex2,descriptors);
	}
	else {
		GenPurpFunc::removeFromVec(minIndex2,descriptors);
		GenPurpFunc::removeFromVec(minIndex1,descriptors);
	}

	Point2f posA = ledA.position;
	Point2f posB = ledB.position;
	Line line(posA,posB);
	int minIndex = 0;
	minDist = 100000;
	for(int i = 0; i < SIZE-2; ++i) {
		float distance = GenPurpFunc::distPoint2Line(descriptors[i].position,line);
		if(distance < minDist) {
			minDist = distance;
			minIndex = i;
		}
	}

	vector<LedDescriptor> sorted(SIZE);
	if(descriptors[minIndex].cartDist(ledA) < descriptors[minIndex].cartDist(ledB)) {
		sorted[2] = ledA;
		sorted[3] = ledB;
	}
	else {
		sorted[2] = ledB;
		sorted[3] = ledA;
	}

	sorted[1] = descriptors[minIndex];
	GenPurpFunc::removeFromVec(minIndex,descriptors);

	if(descriptors[0].cartDist(sorted[1]) < descriptors[0].cartDist(sorted[3])) {
		sorted[0] = descriptors[0];
		sorted[4] = descriptors[1];
	}
	else {
		sorted[0] = descriptors[1];
		sorted[4] = descriptors[0];
	}

	descriptors = sorted;
	oldDescriptors = sorted;

	return true;
}


int PatternAnalysis::secondPhase(vector<LedDescriptor> &descriptors) {

	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// FIXME: WHAT IF descriptors.size() != oldDescriptors.size() ??????????
	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	const int SIZE = descriptors.size();
	vector<LedDescriptor> matchedLeds(SIZE);
	int matched = 0;

	// for each old descriptor...
	for(int i = 0; i < SIZE; ++i) {
		// ... look for the nearest in the new descriptors set
		int index = findNearestPoint(oldDescriptors[i], descriptors);
		if(index > -1) {
			matchedLeds[i] = descriptors[index];
			oldDescriptors[i] = descriptors[index];
			GenPurpFunc::removeFromVec(index,descriptors);
			++matched;
		}

		// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// FIXME: else if index = -1 oldDescriptors[i] should be emptied
		// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	}

	descriptors.clear();
	descriptors = matchedLeds;	// TODO: assign by ref instead of copying everything!

	return matched;
}


int PatternAnalysis::findNearestPoint(const LedDescriptor &oldDescriptor, const vector<LedDescriptor> &descriptors) {
	int minIndex = -1;
	float minDist  = FLT_MAX;
	const int SIZE = descriptors.size();

	// for each descriptor in the set...
	for(int i = 0; i < SIZE; ++i) {

		// ... compute its L2 distance from oldDescriptor...
		float distance = oldDescriptor.L2Dist(descriptors[i]);
		if(distance < minDist) { // ... and keep the minimum distance
			minDist  = distance;
			minIndex = i;
		}
	}

	if(minDist > maxDistance) // check if minDist is beneath the tolerance value
		return -1;

	return minIndex;
}
