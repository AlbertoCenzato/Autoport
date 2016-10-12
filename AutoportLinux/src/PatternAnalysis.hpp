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

	bool evaluate(vector<Point2f> &ledPoints, int tolerance);

private:
	vector<Point2f> oldPoints;
	vector<Point2f> pattern;

	// TODO: use a KD-tree and simplify the code
	void nearestPoints(vector<Point2f> &ledPoints, Mat &img, int tolerance) {

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

	// TODO: remove this method
	// TODO: rename this shitty-named method
	bool firstPhase(vector<Point2f> &ledPoints, int tolerance) {

		auto size = ledPoints.size();
		if(size != 5)
			return false;

		Point2f minPoint1;
		Point2f minPoint2;
		float minDist = FLT_MAX;

		vector<Point2f> sortedVector(size);

		for(int i = 0; i < size; i++) {
			for(int j = 0; j < size; j++) {
				if(i != j) {
					float distance = distPoint2Point(ledPoints[i],ledPoints[j]);
					if(distance < minDist) {
						minDist = distance;
						minPoint1 = ledPoints[i];
						minPoint2 = ledPoints[j];
					}
				}
			}
		}

		Line line(minPoint1,minPoint2);

		Point2f patternLed3(-1,-1);
		for(uint i = 0; i < size; i++) {
			Point2f point = ledPoints[i];
			if(&point != &minPoint1 && &point != &minPoint2) {
				float distance = distPoint2Line(point, line);
				if (distance < tolerance) {
					patternLed3 = point;
					break;
				}
			}
		}

		if(patternLed3.x == -1)
			return false;	//ERRORE!

		sortedVector[3] = patternLed3;
		float d1 = distPoint2Point(minPoint1,patternLed3);
		float d2 = distPoint2Point(minPoint2,patternLed3);
		if(d1 > d2) {
			sortedVector[1] = minPoint1;
			sortedVector[2] = minPoint2;
		}
		else {
			sortedVector[1] = minPoint2;
			sortedVector[2] = minPoint1;
		}

		Point2f led[2];
		for(uint i = 0, count = 0; i < size; i++) {
			Point2f point = ledPoints[i];
			if(&point != &minPoint1 && &point != &minPoint2 && &point != &patternLed3) {
				led[count++] = point;
			}
		}

		d1 = distPoint2Point(sortedVector[1],led[0]);
		d2 = distPoint2Point(sortedVector[1],led[1]);
		if(d1 < d2) {
			sortedVector[0] = led[0];
			sortedVector[4] = led[1];
		}
		else {
			sortedVector[0] = led[1];
			sortedVector[4] = led[0];
		}

		return true;
	}

	bool ransac(vector<Point2f> &input) {
		const uint SIZE = input.size();
		assert(SIZE == pattern.size() &&
			"PatternAnalysis error! RANSAC input vector and pattern vector don't have the same size");

		// find the homography that transforms input points in pattern points
		Mat_<float> H = findHomography(pattern, input, RANSAC);

		if(H.empty()) {
			cout << "homography not found!" << endl;
			return false;
		}

		// apply the homography to each point
		vector<Point2f> tmp(SIZE);
		for(uint i = 0; i < SIZE; ++i) {
			Vec3f vec(input[i].x, input[i].y, 1);
			Mat_<float> p = H*Mat_<float>(vec);
			Vec3f reversedPoint(p);
			tmp[i] = Point2f(reversedPoint.val[0],reversedPoint.val[1]);;
		}

		// find the nearest point
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
			input[index] = tmp[i];
		}

		delete [] flag;

		return true;
	}


};
