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
	PatternAnalysis() {
		oldPoints = vector<Point2f>();
		Settings& settings = Settings::getInstance();
		const int SIZE = settings.realWorldPoints.size();

		// TODO: check if this assumption is correct
		// here the z-component is neglected because it is very small

		pattern = vector<Point2f>(SIZE);
		for(int i = 0; i < SIZE; ++i) {
			pattern[i] = Point2f(settings.realWorldPoints[i].x,settings.realWorldPoints[i].y);
		}

		//FIXME: "pattern" must be a Mat object
		//kdTree = new flann::GenericIndex<cvflann::L2<float>>(pattern, params);
	}

	~PatternAnalysis() {
		//delete kdTree;
	}

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
	//flann::GenericIndex<cvflann::L2<float>>* kdTree;
	cvflann::LinearIndexParams params;

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

		/*
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
		*/

		vector<Point2f> prevLedPoints;

		auto kdTree = flann::GenericIndex<cvflann::L2<float>>(Mat(prevLedPoints),params);



		//FIXME: can't always return true!
		return true;
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
	bool firstPhase(vector<Point2f> &ledPoints) {

		const int SIZE = ledPoints.size();
		//assert(SIZE == pattern.size() && "PatternAnalysis error! RANSAC ledPoints vector and pattern vector don't have the same size");

		/*
		// find the homography that transforms ledPoints points in pattern points

		//FIXME: can't find the homography! The difference between ledpoints and pattern
		//		 is not a simple plane projection?

		cout << "ledPoints" << ledPoints << endl;
		Point2f ledCenter = GenPurpFunc::centroid(ledPoints);
		Point2f patCenter = GenPurpFunc::centroid(pattern);
		cout << "ledCenter " << ledCenter << endl;
		cout << "patCenter " << patCenter << endl;

		Point2f dist = ledCenter - patCenter;
		for(int i = 0; i < SIZE; ++i)
			ledPoints[i] = ledPoints[i] - dist;
		cout << "ledPoints" << ledPoints << endl;

		Mat_<float> H = findHomography(pattern, ledPoints, RANSAC);

		if(H.empty()) {
			cout << "homography not found!" << endl;
			return false;
		}

		cout << "Homography matrix: " << H << endl;

		cout << "LedPoints: " << ledPoints << endl;
		vector<Point3f> homogLedPoints(SIZE);
		for(int i = 0; i < SIZE; ++i)
			homogLedPoints[i] = Point3f(ledPoints[i].x, ledPoints[i].y, 1);

		// apply the homography to each point
		vector<Point3f> out(SIZE);
		cv::transform(homogLedPoints,out,H);
		//Point2f transPoints = H*ledPoints[0];

		cout << "Transformed points: " << out << endl;
		cout << "Pattern points: " 	   << pattern << endl;

		// find the nearest point
		// TODO: use a KD-tree

		/*
		int *flag = new int[SIZE];
		for(int i = 0; i < SIZE; ++i)
			flag[i] = -1;

		for(int i = 0; i < SIZE; ++i) {
			int index = GenPurpFunc::findNearestPoint(transPoints.col(i),*kdTree);
			if(flag[i] == 1) {
				cout << "ERROR!! overlapping points" << endl;
				return false;
			}
			flag[i] = 1;
			//ledPoints[index] = tmp[i];
		}

		delete [] flag;
		*/

		int minIndex1 = 0;
		int minIndex2 = 1;
		float minDist = 100000;
		for(int i = 0; i < SIZE; ++i) {
			for(int j = 0; j < SIZE; j++) {
				if(i != j) {
					float distance = distPoint2Point(ledPoints[i],ledPoints[j]);
					if(distance < minDist) {
						minDist = distance;
						minIndex1 = i;
						minIndex2 = j;
					}
				}
			}
		}

		Point2f ledA = ledPoints[minIndex1];
		Point2f ledB = ledPoints[minIndex2];

		if(minIndex1 == SIZE-1) {
			removeFromVec(minIndex1,ledPoints);
			removeFromVec(minIndex2,ledPoints);
		}
		else {
			removeFromVec(minIndex2,ledPoints);
			removeFromVec(minIndex1,ledPoints);
		}

		Line line(ledA,ledB);
		int minIndex = 0;
		minDist = 100000;
		for(int i = 0; i < SIZE-2; ++i) {
			float distance = distPoint2Line(ledPoints[i],line);
			if(distance < minDist) {
				minDist = distance;
				minIndex = i;
			}
		}

		vector<Point2f> sorted(SIZE);
		if(distPoint2Point(ledPoints[minIndex], ledA) < distPoint2Point(ledPoints[minIndex], ledB)) {
			sorted[2] = ledA;
			sorted[3] = ledB;
		}
		else {
			sorted[2] = ledB;
			sorted[3] = ledA;
		}

		sorted[1] = ledPoints[minIndex];
		removeFromVec(minIndex,ledPoints);

		if(distPoint2Point(ledPoints[0], sorted[1]) < distPoint2Point(ledPoints[0], sorted[3])) {
			sorted[0] = ledPoints[0];
			sorted[4] = ledPoints[1];
		}
		else {
			sorted[0] = ledPoints[1];
			sorted[4] = ledPoints[0];
		}

		ledPoints = sorted;

		return true;
	}


};
