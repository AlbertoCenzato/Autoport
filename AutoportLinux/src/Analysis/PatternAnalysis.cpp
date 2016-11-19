/*
 * PatternAnalysis.cpp
 *
 *  Created on: Apr 6, 2016
 *      Author: alberto
 */

#include "PatternAnalysis.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;
using namespace cv;

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
		removeFromVec(minIndex1,ledPoints);
		removeFromVec(minIndex2,ledPoints);
	}
	else {
		removeFromVec(minIndex2,ledPoints);
		removeFromVec(minIndex1,ledPoints);
	}

	Point2f posA = ledA.position;
	Point2f posB = ledB.position;
	Line line(posA,posB);
	int minIndex = 0;
	minDist = 100000;
	for(int i = 0; i < SIZE-2; ++i) {
		float distance = distPoint2Line(ledPoints[i].position,line);
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
	removeFromVec(minIndex,ledPoints);

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
