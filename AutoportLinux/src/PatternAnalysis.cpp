/*
 * PatternAnalysis.cpp
 *
 *  Created on: Apr 6, 2016
 *      Author: alberto
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "GenPurpFunc.h"
#include "PatternAnalysis.h"

using namespace std;
using namespace cv;

// Led recognition algorithm. Gives a number to every led using the numbering convention
// in patterns' file (see "Sensori" folder in dropbox).
// @points: vector of Point2f of the leds' blob centroid.
// @img: image used to visualize identified leds' position and number.
// @tolerance: tolerance in the alignement (in pixels).
//returns: vector of Point2f ordered with the numbering convention

/// TODO: use a quadtree data structure to drastically improve performances
/// TODO: throw an exception if there are problems
void PatternAnalysis::patternMirko(vector<KeyPoint> *points, Mat &img, int tolerance) {

	int numOfPoints = points->size();
	int setNumber = 0;			//number of aligned sets found;

	vector<KeyPoint> *alignedPoints = new vector<KeyPoint>[4];	//TODO: use an array of vectors of POINTERS to Point2f
	long alignedPointsHash[4] = {0L,0L,0L,0L};

	//look for the 4 sets of 3 aligned points
	for (int i = 0; i < numOfPoints; i++) {
		KeyPoint *p1 = &(points->at(i));
		//for each couple of points...
		for (int j = 0; j < numOfPoints; j++) {
			KeyPoint *p2 = &(points->at(j));
			if (p1 != p2) {
				//... compute the equation of the line laying on p1 and p2...
				float dx = p1->pt.x - p2->pt.x;
				float m = (p1->pt.y - p2->pt.y) / dx;
				float q = p1->pt.y - m*(p1->pt.x);

				//... and look for another point that satisfies the equation
				for (int k = 0; k < numOfPoints; k++) {
					KeyPoint *p3 = &(points->at(k));
					if (p3 != p1 && p3 != p2) {		//TODO: manage strange cases like +INF, -INF, NAN
						float distance = abs(p3->pt.y - (m*(p3->pt.x) + q)) / sqrt(1 + pow(m, 2));
						if (distance < tolerance) {

							line(img, p1->pt, p2->pt, Scalar(0, 0, 255));
							imshow("Thresholded Image", img);
							waitKey(1);
							std::cout << "\nvalid set";

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
								std::cout << "\nAligned set " << setNumber - 1 << ": p1[" << p1->pt.x << "," << p1->pt.y << "]"
										  	  	  	  	  	  	  	  	  	   << " p2["  << p2->pt.x << "," << p2->pt.y << "]"
																			   << " p3["  << p3->pt.x << "," << p3->pt.y << "]";
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
		std::cout << "\nMass center " << i << ": " << massCenter[i];
	}

	//order lines: 0 extern vertical, 1 intern vertical, 2 upper horizontal, 3 lower horizontal
	int maxMinCouples[2][2];
	int secondMinDist[2];
	float maxDist = 0, minDist = INT_MAX;
	//find couples of lines with min distance, second min distance and max distance
	for (int j = 0; j < 4; j++)
		for (int k = 0; k < 4; k++)
			if (k != j) {
				float dist = GenPurpFunc::distancePointToPoint(massCenter[j],massCenter[k]);
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
	std::cout << "\nMin distance: (" << maxMinCouples[0][0] << "," << maxMinCouples[0][1] << ")";
	std::cout << "\nMax distance: (" << maxMinCouples[1][0] << "," << maxMinCouples[1][1] << ")";

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



	vector<KeyPoint> *ledPattern = new vector<KeyPoint>(8);
	int count = 0;
	for (int i = 0; i < 2; i++) {
		double minDist = INT_MAX, maxDist = 0;
		int minIndx[2], maxIndx[2];
		vector<KeyPoint> alignedSet = alignedPoints[lines[i]];
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				if (i != j) {
					double dist = GenPurpFunc::distancePointToPoint(alignedSet[i].pt, alignedSet[j].pt);
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
			ledPattern->at(count++) = alignedSet[minIndx[0]];
			ledPattern->at(count++) = alignedSet[minIndx[1]];
			ledPattern->at(count++) = alignedSet[maxIndx[1]];
		}
		else if (minIndx[0] == maxIndx[1]) {
			ledPattern->at(count++) = alignedSet[minIndx[0]];
			ledPattern->at(count++) = alignedSet[minIndx[1]];
			ledPattern->at(count++) = alignedSet[maxIndx[0]];
		}
		else if (minIndx[1] == maxIndx[0]) {
			ledPattern->at(count++) = alignedSet[minIndx[1]];
			ledPattern->at(count++) = alignedSet[minIndx[0]];
			ledPattern->at(count++) = alignedSet[maxIndx[1]];
		}
		else {
			ledPattern->at(count++) = alignedSet[minIndx[1]];
			ledPattern->at(count++) = alignedSet[minIndx[0]];
			ledPattern->at(count++) = alignedSet[maxIndx[0]];
		}
	}

	for (int i = 0; i < 3; i++) {
		KeyPoint *p = &(alignedPoints[lines[2]][i]);
		if (p->pt.x != ledPattern->at(0).pt.x && p->pt.y != ledPattern->at(0).pt.y && p->pt.x != ledPattern->at(3).pt.x && p->pt.y != ledPattern->at(3).pt.y) {
			ledPattern->at(6) = *p;
			break;
		}
	}
	for (int i = 0; i < 3; i++) {
		KeyPoint *p = &(alignedPoints[lines[3]][i]);
		if (p->pt.x != ledPattern->at(2).pt.x && p->pt.y != ledPattern->at(2).pt.y && p->pt.x != ledPattern->at(5).pt.x && p->pt.y != ledPattern->at(5).pt.y) {
			ledPattern->at(7) = *p;
			break;
		}
	}

	for (int i = 0; i < 8; i++) {
		ostringstream convert;
		convert << i;
		string s = convert.str();
		GenPurpFunc::drawDetectedLed(img, ledPattern->at(i).pt, s);
	}
	waitKey(1);

	delete [] alignedPoints;

	return; //ledPattern;
}



