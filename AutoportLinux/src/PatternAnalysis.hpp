/*
 * PatternAnalysis.h
 *
 *  Created on: Apr 6, 2016
 *      Author: alberto
 */

#pragma once

#include "GenPurpFunc.hpp"

using namespace std;
using namespace cv;
using namespace GenPurpFunc;

extern string workingDir;

class PatternAnalysis {

public:
	PatternAnalysis() {
		oldPoints = vector<Point2f>();
	}

	~PatternAnalysis() {}

	void evaluate(vector<Point2f>&, Mat&, int);

private:
	vector<Point2f> oldPoints;

	void patternMirko(vector<Point2f> &points, Mat &img, int tolerance) {

		int numOfPoints = points.size();
		int setNumber = 0;			//number of aligned sets found;

		vector<Point2f> *alignedPoints = new vector<Point2f>[4];	//TODO: use an array of vectors of POINTERS to Point2f
		long alignedPointsHash[4] = {0L,0L,0L,0L};

		//look for the 4 sets of 3 aligned points
		for (int i = 0; i < numOfPoints; i++) {
			Point2f *p1 = &(points.at(i));
			//for each couple of points...
			for (int j = 0; j < numOfPoints; j++) {
				Point2f *p2 = &(points.at(j));
				if (p1 != p2) {
					//... compute the equation of the line laying on p1 and p2...
					float dx = p1->x - p2->x;
					float m = (p1->y - p2->y) / dx;
					float q = p1->y - m*(p1->x);

					//... and look for another point that satisfies the equation
					for (int k = 0; k < numOfPoints; k++) {
						Point2f *p3 = &(points.at(k));
						if (p3 != p1 && p3 != p2) {		//TODO: manage strange cases like +INF, -INF, NAN
							float distance = abs(p3->y - (m*(p3->x) + q)) / sqrt(1 + pow(m, 2));
							if (distance < tolerance) {

								line(img, *p1, *p2, Scalar(150, 150, 0));
								namedWindow("Detected lines", WINDOW_NORMAL);
								imshow("Detected lines", img);
								waitKey(1);
								imwrite(workingDir + "output/detectedLines.jpg",img);
								//std::cout << "\nvalid set";

								//check if the set {p1, p2, p3} has been already found
								bool alreadyFound = false;
								long hash = (long)p1 + (long)p2 + (long)p3;
								for (int h = setNumber - 1; h >= 0; h--) {
									if (hash == alignedPointsHash[h])
										alreadyFound = true;
								}
								if (!alreadyFound) {
									alignedPointsHash[setNumber] = hash;
									alignedPoints[setNumber].push_back(*p1);
									alignedPoints[setNumber].push_back(*p2);
									alignedPoints[setNumber++].push_back(*p3);
									//std::cout << "\nAligned set " << setNumber - 1 << ": p1[" << p1->x << "," << p1->y << "]"
											//<< " p2["  << p2->x << "," << p2->y << "]"
											//<< " p3["  << p3->x << "," << p3->y << "]";
								}
							}
						}
					}
				}
			}
		}

		//compute the mass center for every set
		Point2f massCenter[4];
		for (int i = 0; i < setNumber-1; i++) {
			massCenter[i] = GenPurpFunc::centroid(alignedPoints[i]);
			//std::cout << "\nMass center " << i << ": " << massCenter[i];
		}

		//order lines: 0 extern vertical, 1 intern vertical, 2 upper horizontal, 3 lower horizontal
		int maxMinCouples[2][2];
		int secondMinDist[2];
		float maxDist = 0, minDist = INT_MAX;
		//find couples of lines with min distance, second min distance and max distance
		for (int j = 0; j < 4; j++)
			for (int k = 0; k < 4; k++)
				if (k != j) {
					float dist = GenPurpFunc::distPoint2Point(massCenter[j],massCenter[k]);
					if ( dist < minDist) {
						secondMinDist[0] = maxMinCouples[0][0];
						secondMinDist[1] = maxMinCouples[0][1];
						minDist = dist;
						maxMinCouples[0][0] = j;
						maxMinCouples[0][1] = k;
					}
					if (dist > maxDist) {
						maxDist = dist;
						maxMinCouples[1][0] = j;
						maxMinCouples[1][1] = k;
					}
				}
		//std::cout << "\nMin distance: (" << maxMinCouples[0][0] << "," << maxMinCouples[0][1] << ")";
		//std::cout << "\nMax distance: (" << maxMinCouples[1][0] << "," << maxMinCouples[1][1] << ")";

		int lines[4];
		//the line that has both min distance and second min distance is the internal vertical line, the others are assigned consequently
		if (secondMinDist[0] == maxMinCouples[0][0]) {
			//secondMinDist[0] internal vertical line
			lines[1] = secondMinDist[0];
			lines[0] = maxMinCouples[0][1];
			lines[2] = secondMinDist[1];
			lines[3] = maxMinCouples[1][0] != secondMinDist[1] ? maxMinCouples[1][0] : maxMinCouples[1][1];
		}
		else if (secondMinDist[0] == maxMinCouples[0][1]) {
			lines[1] = secondMinDist[0];
			lines[0] = maxMinCouples[0][0];
			lines[2] = secondMinDist[1];
			lines[3] = maxMinCouples[1][0] != secondMinDist[1] ? maxMinCouples[1][0] : maxMinCouples[1][1];
		}
		else if (secondMinDist[1] == maxMinCouples[0][0]) {
			lines[1] = secondMinDist[1];
			lines[0] = maxMinCouples[0][1];
			lines[2] = secondMinDist[0];
			lines[3] = maxMinCouples[1][0] != secondMinDist[0] ? maxMinCouples[1][0] : maxMinCouples[1][1];
		}
		else {
			lines[1] = secondMinDist[1];
			lines[0] = maxMinCouples[0][0];
			lines[2] = secondMinDist[0];
			lines[3] = maxMinCouples[1][0] != secondMinDist[0] ? maxMinCouples[1][0] : maxMinCouples[1][1];
		}
		std::cout << "\nLine 0: " << lines[0];
		std::cout << "\nLine 1: " << lines[1];
		std::cout << "\nLine 2: " << lines[2];
		std::cout << "\nLine 3: " << lines[3];



		vector<Point2f> ledPattern = vector<Point2f>(8);
		int count = 0;
		for (int i = 0; i < 2; i++) {
			double minDist = INT_MAX, maxDist = 0;
			int minIndx[2], maxIndx[2];
			vector<Point2f> alignedSet = alignedPoints[lines[i]];
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					if (i != j) {
						double dist = GenPurpFunc::distPoint2Point(alignedSet.at(i), alignedSet.at(j));
						if (dist < minDist) {
							minDist = dist;
							minIndx[0] = i;
							minIndx[1] = j;
						}
						if (dist > maxDist) {
							maxDist = dist;
							maxIndx[0] = i;
							maxIndx[1] = j;
						}
					}
				}
			}
			if (minIndx[0] == maxIndx[0]) {
				ledPattern.at(count++) = alignedSet[minIndx[0]];
				ledPattern.at(count++) = alignedSet[minIndx[1]];
				ledPattern.at(count++) = alignedSet[maxIndx[1]];
			}
			else if (minIndx[0] == maxIndx[1]) {
				ledPattern.at(count++) = alignedSet[minIndx[0]];
				ledPattern.at(count++) = alignedSet[minIndx[1]];
				ledPattern.at(count++) = alignedSet[maxIndx[0]];
			}
			else if (minIndx[1] == maxIndx[0]) {
				ledPattern.at(count++) = alignedSet[minIndx[1]];
				ledPattern.at(count++) = alignedSet[minIndx[0]];
				ledPattern.at(count++) = alignedSet[maxIndx[1]];
			}
			else {
				ledPattern.at(count++) = alignedSet[minIndx[1]];
				ledPattern.at(count++) = alignedSet[minIndx[0]];
				ledPattern.at(count++) = alignedSet[maxIndx[0]];
			}
		}

		for (int i = 0; i < 3; i++) {
			Point2f *p = &(alignedPoints[lines[2]][i]);
			if (p->x != ledPattern.at(0).x && p->y != ledPattern.at(0).y && p->x != ledPattern.at(3).x && p->y != ledPattern.at(3).y) {
				ledPattern.at(6) = *p;
				break;
			}
		}
		for (int i = 0; i < 3; i++) {
			Point2f *p = &(alignedPoints[lines[3]][i]);
			if (p->x != ledPattern.at(2).x && p->y != ledPattern.at(2).y && p->x != ledPattern.at(5).x && p->y != ledPattern.at(5).y) {
				ledPattern.at(7) = *p;
				break;
			}
		}

		for (int i = 0; i < 8; i++) {
			ostringstream convert;
			convert << i;
			string s = convert.str();
			GenPurpFunc::drawDetectedLed(img, ledPattern.at(i), s);
		}
		waitKey(1);

		points.assign(ledPattern.begin(),ledPattern.end());

		delete [] alignedPoints;

		return; //ledPattern;
	}

	void nearerPoints(vector<Point2f> &ledPoints, Mat &img, int tolerance) {

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

	// TODO: check this method
	// TODO: rename this shitty-named method
	bool firstPhase(vector<Point2f> &ledPoints, int tolerance) {

		uint size = ledPoints.size();
		if(size != 5)
			return false;

		Point2f minPoint1;
		Point2f minPoint2;
		float minDist = FLT_MAX;

		vector<Point2f> sortedVector(size);

		for(uint i = 0; i < size; i++) {
			for(uint j = 0; j < size; j++) {
				if(i != j) {
					Point2f p1 = ledPoints[i];
					Point2f p2 = ledPoints[j];
					float distance = distPoint2Point(p1,p2);
					if(distance < minDist) {
						minDist = distance;
						minPoint1 = p1;
						minPoint2 = p2;
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


};
