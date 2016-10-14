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

using namespace std;
using namespace cv;
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
	PatternAnalysis() {
		oldPoints = vector<Point2f>();
		Settings& settings = Settings::getInstance();
		const int SIZE = settings.realWorldPoints.size();
		pattern = vector<Point2f>(SIZE);

		// TODO: check if this assumption is correct
		// here the z-component is neglected because it is very small
		for(int i = 0; i < SIZE; ++i)
			pattern[i] = Point2f(settings.realWorldPoints[i].x, settings.realWorldPoints[i].y);
	}

	~PatternAnalysis() {}

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
	bool evaluate(vector<Point2f> &ledPoints, int tolerance);

private:
	// TODO: change with a KD-tree
	vector<Point2f> oldPoints;	// points at time t-1
	vector<Point2f> pattern;	// model of the pattern, do not modify

	// TODO: use a KD-tree and simplify the code
	/*
	 * WARNING! NOT WORKING by the moment. It was written for another led pattern!
	 * Receives the points detected from the image
	 * and states if they represent a pattern or not.
	 * Used at time t > 0.
	 *
	 * @ledPoints: points to evaluate
	 * @tolerance: temporarly unused parameter
	 *
	 * @return: true if a match is found, false otherwise
	 */
	bool nearestPoints(vector<Point2f> &ledPoints, Mat &img, int tolerance) {

		vector<Point2f> orderedVector(8);

		//looking for led 6
		Point2f point = oldPoints[6];

		float minDist = GenPurpFunc::distPoint2Point(point,ledPoints[0]);
		int minIndex = 0;
		for (int i = 1; i < 8; i++) {
			Point2f keyPoint = ledPoints[i];
			float distance = GenPurpFunc::distPoint2Point(point,keyPoint);
			if(distance < minDist) {
				minDist = distance;
				minIndex = i;
			}
		}

		orderedVector[6] = ledPoints[minIndex];
		ledPoints[minIndex] = ledPoints[7];
		ledPoints.pop_back();

		//looking for led 7
		point = oldPoints[7];

		minDist = GenPurpFunc::distPoint2Point(point,ledPoints[0]);
		minIndex = 0;
		for (int i = 1; i < 7; i++) {
			Point2f keyPoint = ledPoints[i];
			float distance = GenPurpFunc::distPoint2Point(point, keyPoint);
			if(distance < minDist) {
				minDist = distance;
				minIndex = i;
			}
		}

		orderedVector[7] = ledPoints[minIndex];
		ledPoints[minIndex] = ledPoints[6];
		ledPoints.pop_back();
	}

	/*
	 * Receives the points detected from the image
	 * and states if they represent a pattern or not.
	 * Used at time t = 0 (the very first frame)
	 *
	 * @ledPoints: points to evaluate
	 * @tolerance: temporarly unused parameter
	 *
	 * @return: true if a match is found, false otherwise
	 */
	bool firstPhase(vector<Point2f> &ledPoints, int tolerance) {

		const uint SIZE = ledPoints.size();
		assert(SIZE == pattern.size() &&
			"PatternAnalysis error! RANSAC ledPoints vector and pattern vector don't have the same size");

		// find the homography that transforms ledPoints points in pattern points
		Mat_<float> H = findHomography(pattern, ledPoints, RANSAC);

		if(H.empty()) {
			cout << "homography not found!" << endl;
			return false;
		}

		// apply the homography to each point
		vector<Point2f> tmp(SIZE);
		for(uint i = 0; i < SIZE; ++i) {
			Vec3f vec(ledPoints[i].x, ledPoints[i].y, 1);
			Mat_<float> p = H*Mat_<float>(vec);
			Vec3f reversedPoint(p);
			tmp[i] = Point2f(reversedPoint.val[0],reversedPoint.val[1]);;
		}

		// find the nearest point
		// TODO: use a KD-tree
		int *flag = new int[SIZE];
		for(uint i = 0; i < SIZE; ++i)
			flag[i] = -1;

		for(uint i = 0; i < SIZE; ++i) {
			int index = GenPurpFunc::findNearestPoint(tmp[i],pattern);
			if(flag[i] == 1) {
				cout << "ERROR!! overlapping points" << endl;
				return false;
			}
			flag[i] = 1;
			ledPoints[index] = tmp[i];
		}

		delete [] flag;

		return true;
	}


};
