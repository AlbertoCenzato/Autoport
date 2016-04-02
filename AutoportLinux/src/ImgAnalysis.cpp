//Copyright (c) 2016 Alberto Cenzato. All rights reserved.

#include <iostream>
#include <set>
#include <chrono>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "ImgAnalysis.h"
#include "GenPurpFunc.h"

using namespace cv;
using namespace std;

//--- Structs ---

struct Distance {
	float dist = 0;
	Point2f *point1;
	//KeyPoint *keyPoint2;
	int point2;
};

struct lessDist : binary_function <Distance, Distance, bool> {
	bool operator() (const Distance &d1, const Distance &d2) const { return d1.dist < d2.dist; }
};

struct orderByX : binary_function <Point2f, Point2f, bool> {
	bool operator() (const Point2f &p1, const Point2f &p2) const { return p1.x < p2.x; }
};

//--- Functions ---


//TODO: make the function accept a pointer to a pattern analysis function
vector<Point2f>* ImgAnalysis::evaluate(Mat &img) {

	// Crop the full image according to the region of interest
	// Note that this doesn't copy the data
	if(regionOfInterest != NULL)
		img = img(*regionOfInterest);

	tempImg = new Mat(img.rows,img.cols,img.depth());

	//change color space: from BGR to HSV;
	auto begin = std::chrono::high_resolution_clock::now();
	if(ledColor == LedColor::RED)	{
		int test = 0;
		cvtColor(img,*tempImg,COLOR_RGB2HSV);
	}
	else {
		int test = 1;
		cvtColor(img,*tempImg,COLOR_BGR2HSV);
	}

	auto end = std::chrono::high_resolution_clock::now();
	cout << "\nConvert color: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;
	namedWindow("Cropped image", WINDOW_NORMAL);
	imshow("Cropped image", *tempImg);
	waitKey(1);

	//filter the color according to this->low and this->high tolerances
	begin = std::chrono::high_resolution_clock::now();
	filterByColor();
	end = std::chrono::high_resolution_clock::now();
	cout << "\nFilter color: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;
	namedWindow("Filtered image", WINDOW_NORMAL);
	imshow("Filtered image", *tempImg);
	waitKey(1000);
	imwrite("/home/alberto/Pictures/foto/output/filterByColor.jpg",*tempImg);

	delete points;
	//put in this->points detected blobs that satisfy this->params tolerance
	begin = std::chrono::high_resolution_clock::now();
	findBlobs();
	end = std::chrono::high_resolution_clock::now();
	cout << "\nFind blobs: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;
	imwrite("/home/alberto/Pictures/foto/output/findBlobs.jpg",*tempImg);

	//order this->points accordingly to the led pattern numbering
	begin = std::chrono::high_resolution_clock::now();
	patternMirko(points, *tempImg, 10);
	end = std::chrono::high_resolution_clock::now();
	cout << "\nPattern: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;
	imwrite("/home/alberto/Pictures/foto/output/patternMirko.jpg",*tempImg);

	int ledPointsLength = points->size();
	for (int i = 0; i < ledPointsLength; i++) {
		Point2f point = points->at(i);
		std::cout << "\nPoint " << i + 1 << ": x[" << point.x << "] y[" << point.y << "]";
	}

	delete tempImg;

	int maxH = 0;
	int maxS = 0;
	int maxV = 0;
	int minH = 255;
	int minS = 255;
	int minV = 255;

	for (int i = 0; i < ledPointsLength; i++) {
		Point2f p = points->at(i);
		Vec3b color = img.at<Vec3b>(p);
		if (color[0] > maxH)	maxH = color[0];
		if (color[1] > maxS)	maxS = color[1];
		if (color[2] > maxV)	maxV = color[2];
		if (color[0] < minH)	minH = color[0];
		if (color[1] < minS)	minS = color[1];
		if (color[2] < minV)	minV = color[2];
	}
	low  = Scalar(minH - colorTolerance, minS - colorTolerance, minV - colorTolerance);
	high = Scalar(maxH + colorTolerance, maxS + colorTolerance, maxV + colorTolerance);

	Point2f *maxX = GenPurpFunc::findMaxXInVec(*points);
	Point2f *maxY = GenPurpFunc::findMaxYInVec(*points);
	Point2f *minX = GenPurpFunc::findMinXInVec(*points);
	Point2f *minY = GenPurpFunc::findMinYInVec(*points);
	if(regionOfInterest != NULL) delete regionOfInterest;
	regionOfInterest = new Rect(minX->x - ROItolerance, minY->y - ROItolerance, maxX->x - minX->x + 2*ROItolerance, maxY->y - minY->y + 2*ROItolerance);

	return points;
}


// Led recognition algorithm. Gives a number to every led using the numbering convention
// in patterns' file (see "Sensori" folder in dropbox).
// @points: vector of Point2f of the leds' blob centroid.
// @img: image used to visualize identified leds' position and number
//returns: vector of Point2f ordered with the numbering convention

/*
vector<Point2f> ImgAnalysis::pattern1(vector<Point2f> &points, Mat &img) {

	//compute the distances between points
	set<Distance, lessDist> distances[12];	//TODO: dynamic allocation using the free store
	Distance d;
	for (int i = 0; i < 12; i++)	{
		for (int j = 0; j < 12; j++) {
			if (i != j) {
				d.point1 = &points[i];
				d.point2 = j;
				d.dist = distancePointToPoint(*d.point1, points[j]);
				distances[i].insert(d);
			}
		}
	}

	//print distances
	for (int j = 0; j < 12; j++) {
		std::cout << "\nPoint " << j+1 << ":";
		int i = 1;
		for (std::set<Distance, lessDist>::iterator it = distances[j].begin(); it != distances[j].end(); ++it) {
			Distance d = *it;
			std::cout << "\n   Distance " << i++ << ": " << d.dist;
		}
	}

	int minIndx = 0;
	int maxIndx = 0;
	for (int i = 0; i < 12; i++){
		Distance newMinDist = *(distances[i].begin());
		Distance newMaxDist = *(distances[i].rbegin());
		if (newMinDist.dist < (*(distances[minIndx].begin())).dist)
			minIndx = i;
		if (newMaxDist.dist > (*(distances[maxIndx].rbegin())).dist)
			maxIndx = i;
	}

	std::cout << "\nMin distance: " << (*(distances[minIndx].begin())).dist;
	std::cout << "\nMax distance: " << (*(++distances[maxIndx].rend())).dist;

	set<Distance, lessDist> patternPoints[12];

	int minKP1 = minIndx;
	int minKP2 = (*(distances[minIndx].begin())).point2;
	int maxKP1 = maxIndx;
	int maxKP2 = (*(distances[maxIndx].rbegin())).point2;

	int prevPos[12];

	//LED 12, 11, 9
	if (minKP1 == maxKP1) {
		patternPoints[11] = distances[minIndx];  //CHECK: retruns by value or by reference??
		prevPos[11] = minIndx;
		distances[minIndx] = set<Distance,lessDist>();
		patternPoints[8] = distances[maxKP2];
		prevPos[8] = maxKP2;
		distances[maxKP2] = set<Distance, lessDist>();
		patternPoints[10] = distances[minKP2];
		prevPos[10] = minKP2;
		distances[minKP2] = set<Distance, lessDist>();
	}
	else if (minKP1 == maxKP2) {
		patternPoints[11] = distances[minKP1];
		prevPos[11] = minKP1;
		distances[minKP1] = set<Distance, lessDist>();
		patternPoints[8]  = distances[maxKP1];
		prevPos[8] = maxKP1;
		distances[maxKP1] = set<Distance, lessDist>();
		patternPoints[10] = distances[minKP2];
		prevPos[10] = minKP2;
		distances[minKP2] = set<Distance, lessDist>();
	}
	else if (minKP2 == maxKP1) {
		patternPoints[11] = distances[minKP2];
		prevPos[11] = minKP2;
		distances[minKP2] = set<Distance, lessDist>();
		patternPoints[8]  = distances[maxKP2];
		prevPos[8] = maxKP2;
		distances[maxKP2] = set<Distance, lessDist>();
		patternPoints[10] = distances[minKP1];
		prevPos[10] = minKP1;
		distances[minKP1] = set<Distance, lessDist>();
	}
	else {
		patternPoints[11] = distances[minKP2];
		prevPos[11] = minKP2;
		distances[minKP2] = set<Distance, lessDist>();
		patternPoints[8]  = distances[maxKP1];
		prevPos[8] = maxKP1;
		distances[maxKP1] = set<Distance, lessDist>();
		patternPoints[10] = distances[minKP1];
		prevPos[10] = minKP1;
		distances[minKP1] = set<Distance, lessDist>();
	}

	string s = "9";
	drawDetectedLed(img, *(*patternPoints[8].begin()).point1, s);
	s = "11";
	drawDetectedLed(img, *(*patternPoints[10].begin()).point1, s);
	s = "12";
	drawDetectedLed(img, *(*patternPoints[11].begin()).point1, s);

	//LED 10, 6
	set<Distance,lessDist>::reverse_iterator riter = patternPoints[10].rbegin();
	int kp1 = (*riter).point2;
	if (kp1 == prevPos[8])
		kp1 = (*++riter).point2;
	patternPoints[9] = distances[kp1];
	distances[kp1] = set<Distance, lessDist>();
	s = "10";
	drawDetectedLed(img, *(*patternPoints[9].begin()).point1, s);
	kp1 = (*++riter).point2;
	if (kp1 == prevPos[8] || kp1 == prevPos[9])
		kp1 = (*++riter).point2;
	patternPoints[5] = distances[kp1];
	distances[kp1] = set<Distance, lessDist>();
	s = "6";
	drawDetectedLed(img, *(*patternPoints[5].begin()).point1, s);

	//LED 1 e 2
	set<Distance, lessDist>::iterator iter1 = patternPoints[5].begin();
	kp1 = (*iter1).point2;
	int kp2 = (*++iter1).point2;
	for (set<Distance, lessDist>::iterator iter2 = patternPoints[8].begin(); iter2 != patternPoints[8].end();) {
		int kp = (*iter2).point2;
		iter2++;
		if (kp == kp1) {
			patternPoints[1] = distances[kp1];
			prevPos[1] = kp1;
			distances[kp1] = set<Distance, lessDist>();
			patternPoints[0] = distances[kp2];
			prevPos[0] = kp2;
			distances[kp2] = set<Distance, lessDist>();
			iter2 = patternPoints[8].end();
		}
		else if (kp == kp2) {
			patternPoints[1] = distances[kp2];
			prevPos[1] = kp2;
			distances[kp2] = set<Distance, lessDist>();
			patternPoints[0] = distances[kp1];
			prevPos[0] = kp1;
			distances[kp1] = set<Distance, lessDist>();
			iter2 = patternPoints[8].end();
		}
	}
	s = "1";
	drawDetectedLed(img, *(*patternPoints[0].begin()).point1, s);
	s = "2";
	drawDetectedLed(img, *(*patternPoints[1].begin()).point1, s);

	//led 3
	set<Distance, lessDist>::iterator iter = patternPoints[1].begin();
	kp1 = (*iter).point2;
	while (kp1 == prevPos[0] || kp1 == prevPos[5] || kp1 == prevPos[8])
		kp1 = (*++iter).point2;
	patternPoints[2] = distances[kp1];
	prevPos[2] = kp1;
	distances[kp1] = set<Distance, lessDist>();
	s = "3";
	drawDetectedLed(img, *(*patternPoints[2].begin()).point1, s);

	//led 7
	iter = patternPoints[9].begin();
	kp1 = (*iter).point2;
	if (kp1 == prevPos[2])
		kp1 = (*++iter).point2;
	patternPoints[6] = distances[kp1];
	prevPos[6] = kp1;
	distances[kp1] = set<Distance, lessDist>();
	s = "7";
	drawDetectedLed(img, *(*patternPoints[6].begin()).point1, s);

	//led 4
	iter = patternPoints[6].begin();
	kp1 = (*iter).point2;
	if (kp1 == prevPos[2])
		kp1 = (*++iter).point2;
	patternPoints[3] = distances[kp1];
	prevPos[3] = kp1;
	distances[kp1] = set<Distance, lessDist>();
	s = "4";
	drawDetectedLed(img, *(*patternPoints[3].begin()).point1, s);

	//led 5 e 8
	int fiveAndEight[2];
	int j = 0;
	for (int i = 0; i < 12, j < 2; i++) {
		if (!distances[i].empty()) {
			fiveAndEight[j++] = i;
		}
	}

	vector<Point2f> finalKeyPoints = vector<Point2f>(12);
	for (int i = 0; i < 12; i++) {
		if (!patternPoints[i].empty())
			finalKeyPoints[i] = *((*(patternPoints[i].begin())).point1);
	}

	for (iter = patternPoints[3].begin(); iter != patternPoints[3].end(); ) {
		kp1 = (*(iter++)).point2;
		if (kp1 == fiveAndEight[0]) {
			finalKeyPoints[4] = *(*(distances[fiveAndEight[0]].begin())).point1;
			finalKeyPoints[7] = *(*(distances[fiveAndEight[1]].begin())).point1;
			iter = patternPoints[3].end();
		}
		else if (kp1 == fiveAndEight[1]) {
			finalKeyPoints[4] = *(*(distances[fiveAndEight[1]].begin())).point1;
			finalKeyPoints[7] = *(*(distances[fiveAndEight[0]].begin())).point1;
			iter = patternPoints[3].end();
		}
	}
	s = "5";
	drawDetectedLed(img, finalKeyPoints[4], s);
	s = "8";
	drawDetectedLed(img, finalKeyPoints[7], s);

	return finalKeyPoints;
}
*/

// Led recognition algorithm. Gives a number to every led using the numbering convention
// in patterns' file (see "Sensori" folder in dropbox).
// @points: vector of Point2f of the leds' blob centroid.
// @img: image used to visualize identified leds' position and number
//returns: vector of Point2f ordered with the numbering convention

//THE LED PATTERN HAS AN ERROR!
/*
vector<Point2f> ImgAnalysis::pattern3(vector<Point2f> &points, Mat &image) {

	//compute the distances between points
	set<Distance, lessDist> distances[12];
	Distance d;
	for (int i = 0; i < 12; i++)	{
		for (int j = 0; j < 12; j++) {
			if (j != i) {
				d.point1 = &points[i];
				d.point2 = j;
				d.dist = distancePointToPoint(*d.point1, points[j]);
				distances[i].insert(d);
			}
		}
	}

	//print distances
	for (int j = 0; j < 12; j++) {
		std::cout << "\nPoint " << j + 1 << ":";
		int i = 1;
		for (std::set<Distance, lessDist>::iterator it = distances[j].begin(); it != distances[j].end(); ++it) {
			Distance d = *it;
			std::cout << "\n   Distance " << i++ << ": " << d.dist;
		}
	}

	//find the max and min distance
	int minIndx = 0;
	int maxIndx = 0;
	for (int i = 0; i < 12; i++){
		float newMinDist = (*(distances[i].begin())).dist;
		float newMaxDist = (*(distances[i].rbegin())).dist;
		if (newMinDist < (*(distances[minIndx].begin())).dist)
			minIndx = i;
		if (newMaxDist >(*(distances[maxIndx].rbegin())).dist)
			maxIndx = i;
	}

	std::cout << "\nMin distance: " << (*(distances[minIndx].begin())).dist;
	std::cout << "\nMax distance: " << (*(++distances[maxIndx].rend())).dist;

	set<Distance, lessDist> patternPoints[12];


	//	KeyPoint *minKP1 = (*(distances[minIndx].begin())).keyPoint1;
	//	KeyPoint *minKP2 = (*(distances[minIndx].begin())).keyPoint2;
	//	KeyPoint *maxKP1 = (*(++(distances[maxIndx].rend()))).keyPoint1;
	//	KeyPoint *maxKP2 = (*(++(distances[maxIndx].rend()))).keyPoint2;

	int minKP1 = minIndx;
	int minKP2 = (*(distances[minIndx].begin())).point2;
	int maxKP1 = maxIndx;
	int maxKP2 = (*(distances[maxIndx].rbegin())).point2;

	int prevPos[12];

	//LED 12, 11, 6
	if (minKP1 == maxKP1) {
		patternPoints[11] = distances[minKP1];  //CHECK: retruns by value or by reference??
		prevPos[11] = minKP1;
		distances[minKP1] = set<Distance, lessDist>();
		patternPoints[8] = distances[maxKP2];
		prevPos[8] = maxKP2;
		distances[maxKP2] = set<Distance, lessDist>();
		patternPoints[5] = distances[minKP2];
		prevPos[5] = minKP2;
		distances[minKP2] = set<Distance, lessDist>();
	}
	else if (minKP1 == maxKP2) {
		patternPoints[11] = distances[minKP1];
		prevPos[11] = minKP1;
		distances[minKP1] = set<Distance, lessDist>();
		patternPoints[8] = distances[maxKP1];
		prevPos[8] = maxKP1;
		distances[maxKP1] = set<Distance, lessDist>();
		patternPoints[5] = distances[minKP2];
		prevPos[5] = minKP2;
		distances[minKP2] = set<Distance, lessDist>();
	}
	else if (minKP2 == maxKP1) {
		patternPoints[11] = distances[minKP2];
		prevPos[11] = minKP2;
		distances[minKP2] = set<Distance, lessDist>();
		patternPoints[8] = distances[maxKP2];
		prevPos[8] = maxKP2;
		distances[maxKP2] = set<Distance, lessDist>();
		patternPoints[5] = distances[minKP1];
		prevPos[5] = minKP1;
		distances[minKP1] = set<Distance, lessDist>();
	}
	else {
		patternPoints[11] = distances[minKP2];
		prevPos[11] = minKP2;
		distances[minKP2] = set<Distance, lessDist>();
		patternPoints[8] = distances[maxKP1];
		prevPos[8] = maxKP1;
		distances[maxKP1] = set<Distance, lessDist>();
		patternPoints[5] = distances[minKP1];
		prevPos[5] = minKP1;
		distances[minKP1] = set<Distance, lessDist>();
	}

	string s = "9";
	drawDetectedLed(image, *(*patternPoints[8].begin()).point1, s);
	s = "6";
	drawDetectedLed(image, *(*patternPoints[5].begin()).point1, s);
	s = "12";
	drawDetectedLed(image, *(*patternPoints[11].begin()).point1, s);

	//led 2
	set<Distance, lessDist>::iterator iter = patternPoints[5].begin();
	int kp1 = (*iter).point2;
	while (kp1 == prevPos[11] || kp1 == prevPos[8])
		kp1 = (*++iter).point2;
	patternPoints[1] = distances[kp1];
	prevPos[1] = kp1;
	distances[kp1] = set<Distance, lessDist>();
	s = "2";
	drawDetectedLed(image, *(*patternPoints[1].begin()).point1, s);

	//led 7
	iter = patternPoints[11].begin();
	kp1 = (*iter).point2;
	while (kp1 == prevPos[1] || kp1 == prevPos[5] || kp1 == prevPos[8])
		kp1 = (*++iter).point2;
	patternPoints[6] = distances[kp1];
	prevPos[6] = kp1;
	distances[kp1] = set<Distance, lessDist>();
	s = "7";
	drawDetectedLed(image, *(*patternPoints[6].begin()).point1, s);

	//led 3
	iter = patternPoints[6].begin();
	kp1 = (*iter).point2;
	while (kp1 == prevPos[1] || kp1 == prevPos[5] || kp1 == prevPos[8] || kp1 == prevPos[11])
		kp1 = (*++iter).point2;
	patternPoints[2] = distances[kp1];
	prevPos[2] = kp1;
	distances[kp1] = set<Distance, lessDist>();
	s = "3";
	drawDetectedLed(image, *(*patternPoints[2].begin()).point1, s);

	//led 1
	iter = patternPoints[1].begin();
	kp1 = (*iter).point2;
	while (kp1 == prevPos[2] || kp1 == prevPos[5] || kp1 == prevPos[6] || kp1 == prevPos[8] || kp1 == prevPos[11])
		kp1 = (*++iter).point2;
	patternPoints[0] = distances[kp1];
	prevPos[0] = kp1;
	distances[kp1] = set<Distance, lessDist>();
	s = "1";
	drawDetectedLed(image, *(*patternPoints[0].begin()).point1, s);

	//led 4
	iter = patternPoints[2].begin();
	kp1 = (*iter).point2;
	while (kp1 == prevPos[0] || kp1 == prevPos[1] || kp1 == prevPos[5] || kp1 == prevPos[6] || kp1 == prevPos[8] || kp1 == prevPos[11])
		kp1 = (*++iter).point2;
	patternPoints[3] = distances[kp1];
	prevPos[3] = kp1;
	distances[kp1] = set<Distance, lessDist>();
	s = "4";
	drawDetectedLed(image, *(*patternPoints[3].begin()).point1, s);

	//led 5
	iter = patternPoints[0].begin();
	kp1 = (*iter).point2;
	while (kp1 == prevPos[1] || kp1 == prevPos[2] || kp1 == prevPos[3] || kp1 == prevPos[5] || kp1 == prevPos[6] || kp1 == prevPos[8] || kp1 == prevPos[11])
		kp1 = (*++iter).point2;
	patternPoints[4] = distances[kp1];
	prevPos[4] = kp1;
	distances[kp1] = set<Distance, lessDist>();
	s = "5";
	drawDetectedLed(image, *(*patternPoints[4].begin()).point1, s);

	//led 8
	iter = patternPoints[2].begin();
	kp1 = (*iter).point2;
	while (kp1 == prevPos[0] || kp1 == prevPos[1] || kp1 == prevPos[3] || kp1 == prevPos[4] || kp1 == prevPos[5] || kp1 == prevPos[6] || kp1 == prevPos[8] || kp1 == prevPos[11])
		kp1 = (*++iter).point2;
	patternPoints[7] = distances[kp1];
	prevPos[7] = kp1;
	distances[kp1] = set<Distance, lessDist>();
	s = "8";
	drawDetectedLed(image, *(*patternPoints[7].begin()).point1, s);


	vector<Point2f> finalKeyPoints = vector<Point2f>(12);
	for (int i = 0; i < 12; i++) {
		if (!patternPoints[i].empty())
			finalKeyPoints[i] = *((*(patternPoints[i].begin())).point1);
	}

	//led 10 e 11
	int i = 9;
	for (iter = patternPoints[8].begin(); iter != patternPoints[9].end(), i < 11;) {
		kp1 = (*(iter++)).point2;
		if (!distances[kp1].empty()) {
			finalKeyPoints[i++] = *(*(distances[kp1].begin())).point1;
		}
	}
	s = "10";
	drawDetectedLed(image, finalKeyPoints[9], s);
	s = "11";
	drawDetectedLed(image, finalKeyPoints[10], s);

	return finalKeyPoints;
}
*/

// Led recognition algorithm. Gives a number to every led using the numbering convention
// in patterns' file (see "Sensori" folder in dropbox).
// @points: vector of Point2f of the leds' blob centroid.
// @img: image used to visualize identified leds' position and number.
// @tolerance: tolerance in the alignement (in pixels).
//returns: vector of Point2f ordered with the numbering convention

/// TODO: use a quadtree data structure to drastically improve performances
/// TODO: throw an exception if there are problems
vector<Point2f>* ImgAnalysis::patternMirko(vector<Point2f> *points, Mat &img, int tolerance) {

	int numOfPoints = points->size();
	int setNumber = 0;			//number of aligned sets found;

	vector<Point2f> *alignedPoints = new vector<Point2f>[4];	//TODO: use an array of vectors of POINTERS to Point2f
	long alignedPointsHash[4] = {0L,0L,0L,0L};

	//look for the 4 sets of 3 aligned points
	for (int i = 0; i < numOfPoints; i++) {
		Point2f *p1 = &(points->at(i));
		//for each couple of points...
		for (int j = 0; j < numOfPoints; j++) {
			Point2f *p2 = &(points->at(j));
			if (p1 != p2) {
				//... compute the equation of the line laying on p1 and p2...
				float dx = p1->x - p2->x;
				float m = (p1->y - p2->y) / dx;
				float q = p1->y - m*(p1->x);

				//... and look for another point that satisfies the equation
				for (int k = 0; k < numOfPoints; k++) {
					Point2f *p3 = &(points->at(k));
					if (p3 != p1 && p3 != p2) {		//TODO: manage strange cases like +INF, -INF, NAN
						float distance = abs(p3->y - (m*(p3->x) + q)) / sqrt(1 + pow(m, 2));
						if (distance < tolerance) {

							line(img, *p1, *p2, Scalar(0, 0, 255));
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
								std::cout << "\nAligned set " << setNumber - 1 << ": p1[" << p1->x << "," << p1->y << "]"
										  	  	  	  	  	  	  	  	  	   << " p2["  << p2->x << "," << p2->y << "]"
																			   << " p3["  << p3->x << "," << p3->y << "]";
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
		massCenter[i] = centroid(alignedPoints[i]);
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
				float dist = distancePointToPoint(massCenter[j],massCenter[k]);
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



	vector<Point2f> *ledPattern = new vector<Point2f>(8);
	int count = 0;
	for (int i = 0; i < 2; i++) {
		double minDist = INT_MAX, maxDist = 0;
		int minIndx[2], maxIndx[2];
		vector<Point2f> alignedSet = alignedPoints[lines[i]];
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				if (i != j) {
					double dist = distancePointToPoint(alignedSet[i], alignedSet[j]);
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
		Point2f *p = &alignedPoints[lines[2]][i];
		if (p->x != ledPattern->at(0).x && p->y != ledPattern->at(0).y && p->x != ledPattern->at(3).x && p->y != ledPattern->at(3).y) {
			ledPattern->at(6) = *p;
			break;
		}
	}
	for (int i = 0; i < 3; i++) {
		Point2f *p = &alignedPoints[lines[3]][i];
		if (p->x != ledPattern->at(2).x && p->y != ledPattern->at(2).y && p->x != ledPattern->at(5).x && p->y != ledPattern->at(5).y) {
			ledPattern->at(7) = *p;
			break;
		}
	}

	for (int i = 0; i < 8; i++) {
		ostringstream convert;
		convert << i;
		string s = convert.str();
		drawDetectedLed(img, ledPattern->at(i), s);
	}
	waitKey(1);

	delete [] alignedPoints;

	return ledPattern;
}
