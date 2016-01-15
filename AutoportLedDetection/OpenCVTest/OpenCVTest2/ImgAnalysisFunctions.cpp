#include "stdafx.h"
#include <iostream>
#include <set>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\features2d\features2d.hpp>

using namespace cv;
using namespace std;

//---Private structs---
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

//---Function declaration---
inline void tbColorCallback(int state, void* userdata);
inline void tbBlobCallback(int state, void* userdata);
inline float myDistance(cv::Point2f &, cv::Point2f &);
inline void drawDetectedLed(Mat &, Point2f &, string &);

//---Function definition---

//simple image analysis, with color filtering
vector<Point2f> imgLedDetection(string &imgName,Mat &imgThresholded)
{
	//---Color filtering----
	int iLowH = 80;
	int iHighH = 110;

	int iLowS = 0;
	int iHighS = 255;

	int iLowV = 0;
	int iHighV = 255;

	int64 start = getTickCount();

	Mat imgOriginal = imread(imgName, cv::ImreadModes::IMREAD_COLOR); // read a new frame from video

	//Convert the captured frame from BGR to HSV
	Mat imgHSV;
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); 

	std::cout << "\nTime elapsed in RGB->HSV convertion: " << (getTickCount() - start)/getTickFrequency() << "s    MA SIAMO SICURI CHE SIA IN SECONDI???";

	//Threshold the image
	//Mat imgThresholded;
	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); 

	//morphological opening (remove small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//morphological closing (fill small holes in the foreground)
	dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	double totalTime = (((double)getTickCount()) - start)/getTickFrequency();

	//---Blob detection---

	SimpleBlobDetector::Params params;
	params.filterByColor = true;
	params.blobColor = 255;
	params.filterByInertia = true;
	params.minInertiaRatio = 0.5;
	params.maxInertiaRatio = 1;
	params.filterByArea = true;
	params.minArea = 100;
	params.maxArea = 10000;
	params.filterByConvexity = true;
	params.minConvexity = 0.5;
	params.maxConvexity = 1;
	params.filterByCircularity = true;
	params.minCircularity = 0.5;
	params.maxCircularity = 1;

	Ptr<SimpleBlobDetector> featureDetector = SimpleBlobDetector::create(params);
	vector<KeyPoint> keypoints;
	featureDetector->detect(imgThresholded, keypoints);
	// Draw detected blobs as red circles.
	// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
	drawKeypoints(imgThresholded, keypoints, imgThresholded, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

	vector<Point2f> points;
	for (uint i = 0; i < keypoints.size(); i++) {
		Point2f pt = keypoints[i].pt;
		points.push_back(pt);
		std::cout << "\nPoint " << i+1 << ": x[" << pt.x << "] y[" << pt.y << "]";
	}

	namedWindow("Thresholded Image", WINDOW_NORMAL);
	namedWindow("Original", WINDOW_NORMAL);

	imshow("Thresholded Image", imgThresholded); //show the thresholded image
	imshow("Original", imgOriginal); //show the original image

	waitKey(25);

	return points;
}

//image analysis on video, RETURNS ONLY THE KEYPOINTS DETECTED IN THE LAST FRAME (WELL...
//IT SHOULD, BUT IT THROWS AN EXCEPTION REACHING THE EOF
vector<KeyPoint> vidLedDetection(string &vidName)
{
	int iLowH = 0;
	int iHighH = 180;
	int iLowS = 0;
	int iHighS = 25;
	int iLowV = 225;
	int iHighV = 255;

	//settings trackbar color filtering
	string settWinColor = "Color filtering settings";
	namedWindow(settWinColor, WINDOW_NORMAL);

	cv::TrackbarCallback onChange = tbColorCallback;

	int trckbrLowH = createTrackbar("HueLow", settWinColor, &iLowH, 180, onChange, &iLowH);
	int trckbrHighH = createTrackbar("HueHigh", settWinColor, &iHighH, 180, onChange, &iHighH);

	int trckbrLowS = createTrackbar("SatLow", settWinColor, &iLowS, 255, onChange, &iLowS);
	int trckbrHighS = createTrackbar("SatHigh", settWinColor, &iHighS, 255, onChange, &iHighS);

	int trckbrLowV = createTrackbar("ValLow", settWinColor, &iLowV, 255, onChange, &iLowV);
	int trckbrHighV = createTrackbar("ValHigh", settWinColor, &iHighV, 255, onChange, &iHighV);

	//Mat imgOriginal = imread("image.jpg", cv::ImreadModes::IMREAD_COLOR); // read a new frame from video
	VideoCapture videoCapture = VideoCapture(vidName);

	if (!videoCapture.isOpened())
		return vector<KeyPoint>();

	Mat imgOriginal;
	videoCapture.read(imgOriginal);
	
	// Set up the detector with default parameters.
	SimpleBlobDetector::Params params;
	params.filterByColor = true;
	params.blobColor = 255;
	params.filterByInertia = true;
	int minIn = 500;	params.minInertiaRatio = float(minIn) / 1000;
	int maxIn = 1000;	params.maxInertiaRatio = float(maxIn) / 1000;
	params.filterByArea = true;
	int minA = 0;		params.minArea = float(minA) / 1000;
	int maxA = 100000;	params.maxArea = float(maxA) / 1000;
	params.filterByConvexity = true;
	int minCon = 500;	params.minConvexity = float(minCon) / 1000;
	int maxCon = 1000;	params.maxConvexity = float(maxCon) / 1000;
	params.filterByCircularity = true;
	int minCir = 500;	params.minCircularity = float(minCir) / 1000;
	int maxCir = 1000;	params.maxCircularity = float(maxCir) / 1000;

	string settWinBlob = "Blob filtering settings";
	namedWindow(settWinBlob, WINDOW_NORMAL);
	onChange = tbBlobCallback;

	int minIner = createTrackbar("Min Inertia", settWinBlob, &minIn, 1000, onChange, &params.minInertiaRatio);
	int maxIner = createTrackbar("Max Inertia", settWinBlob, &maxIn, 1000, onChange, &params.maxInertiaRatio);

	int minAr = createTrackbar("Min Area", settWinBlob, &minA, 100000,    onChange, &params.minArea);
	int maxAr = createTrackbar("Max Area", settWinBlob, &maxA, 100000, onChange, &params.maxArea);

	int minConv = createTrackbar("Min Convexity", settWinBlob, &minCon, 1000, onChange, &params.minConvexity);
	int maxConv = createTrackbar("Max Convexity", settWinBlob, &maxCon, 1000, onChange, &params.maxConvexity);

	int minCirc = createTrackbar("Min Circularity", settWinBlob, &minCir, 1000, onChange, &params.minCircularity);
	int maxCirc = createTrackbar("Max Circularity", settWinBlob, &maxCir, 1000, onChange, &params.maxCircularity);
	
	namedWindow("Original", WINDOW_NORMAL);
	namedWindow("Blob detection", WINDOW_NORMAL);

	Mat im_with_keypoints;
	Mat imgThresholded;
	Mat imgHSV;

	vector<KeyPoint> keypoints;
	bool done = false;
	while (!done) {

		videoCapture.read(imgOriginal);
			//break;
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
		//printf("Min %d\nMax %d\n",min,max);

		//morphological opening (remove small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (fill small holes in the foreground)
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		Ptr<SimpleBlobDetector> featureDetector = SimpleBlobDetector::create(params);
		featureDetector->detect(imgThresholded, keypoints);
		// Draw detected blobs as red circles.
		// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
		drawKeypoints(imgThresholded, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

		for (uint i = 0; i < keypoints.size(); i++) {
			Point pt = keypoints[i].pt;
			printf("Point %d - x:%d      y:%d\n", i, pt.x, pt.y);
		}

		// Show blobs
		imshow("Blob detection", im_with_keypoints);

		//namedWindow("Thresholded Image", WINDOW_NORMAL);
		//imshow("Thresholded Image", imgThresholded); //show the thresholded image

		imshow("Original", imgOriginal); //show the original image
		waitKey(25);
		//Sleep(100);
		//videoCapture.set(CAP_PROP_FORMAT,)
	}
	return keypoints;
}

//led recognition algorithm
vector<Point2f> pattern1(vector<Point2f> &keyPoints, Mat &image) {
	
	//compute the distances between points
	set<Distance, lessDist> distances[12];
	Distance d;
	for (uint i = 0; i < 12; i++)	{
		for (int j = 0; j < 12; j++) {
			if (i != j) {
				d.point1 = &keyPoints[i];
				d.point2 = j;
				d.dist = myDistance(*d.point1, keyPoints[j]);
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
	
	/*
	KeyPoint *minKP1 = (*(distances[minIndx].begin())).keyPoint1;
	KeyPoint *minKP2 = (*(distances[minIndx].begin())).keyPoint2;
	KeyPoint *maxKP1 = (*(++(distances[maxIndx].rend()))).keyPoint1;
	KeyPoint *maxKP2 = (*(++(distances[maxIndx].rend()))).keyPoint2;
	*/
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
	drawDetectedLed(image, *(*patternPoints[8].begin()).point1, s);
	s = "11";
	drawDetectedLed(image, *(*patternPoints[10].begin()).point1, s);
	s = "12";
	drawDetectedLed(image, *(*patternPoints[11].begin()).point1, s);

	//LED 10, 6
	set<Distance,lessDist>::reverse_iterator riter = patternPoints[10].rbegin();
	int kp1 = (*riter).point2;
	if (kp1 == prevPos[8])
		kp1 = (*++riter).point2;
	patternPoints[9] = distances[kp1];
	distances[kp1] = set<Distance, lessDist>();
	s = "10";
	drawDetectedLed(image, *(*patternPoints[9].begin()).point1, s);
	kp1 = (*++riter).point2;
	if (kp1 == prevPos[8] || kp1 == prevPos[9])
		kp1 = (*++riter).point2;
	patternPoints[5] = distances[kp1];
	distances[kp1] = set<Distance, lessDist>();
	s = "6";
	drawDetectedLed(image, *(*patternPoints[5].begin()).point1, s);

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
	drawDetectedLed(image, *(*patternPoints[0].begin()).point1, s);
	s = "2";											
	drawDetectedLed(image, *(*patternPoints[1].begin()).point1, s);

	//led 3
	set<Distance, lessDist>::iterator iter = patternPoints[1].begin();
	kp1 = (*iter).point2;
	while (kp1 == prevPos[0] || kp1 == prevPos[5] || kp1 == prevPos[8])
		kp1 = (*++iter).point2;
	patternPoints[2] = distances[kp1];
	prevPos[2] = kp1;
	distances[kp1] = set<Distance, lessDist>();
	s = "3";
	drawDetectedLed(image, *(*patternPoints[2].begin()).point1, s);

	//led 7
	iter = patternPoints[9].begin();
	kp1 = (*iter).point2;
	if (kp1 == prevPos[2])
		kp1 = (*++iter).point2;
	patternPoints[6] = distances[kp1];
	prevPos[6] = kp1;
	distances[kp1] = set<Distance, lessDist>();
	s = "7";
	drawDetectedLed(image, *(*patternPoints[6].begin()).point1, s);

	//led 4
	iter = patternPoints[6].begin();
	kp1 = (*iter).point2;
	if (kp1 == prevPos[2])
		kp1 = (*++iter).point2;
	patternPoints[3] = distances[kp1];
	prevPos[3] = kp1;
	distances[kp1] = set<Distance, lessDist>();
	s = "4";
	drawDetectedLed(image, *(*patternPoints[3].begin()).point1, s);

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
	drawDetectedLed(image, finalKeyPoints[4], s);
	s = "8";
	drawDetectedLed(image, finalKeyPoints[7], s);

	return finalKeyPoints;
}

//led recognition algorithm
//THE LED PATTERN HAS AN ERROR!
vector<Point2f> pattern3(vector<Point2f> &keyPoints, Mat &image) {

	//compute the distances between points
	set<Distance, lessDist> distances[12];
	Distance d;
	for (uint i = 0; i < 12; i++)	{
		for (int j = 0; j < 12; j++) {
			if (j != i) {
				d.point1 = &keyPoints[i];
				d.point2 = j;
				d.dist = myDistance(*d.point1, keyPoints[j]);
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

	/*
	KeyPoint *minKP1 = (*(distances[minIndx].begin())).keyPoint1;
	KeyPoint *minKP2 = (*(distances[minIndx].begin())).keyPoint2;
	KeyPoint *maxKP1 = (*(++(distances[maxIndx].rend()))).keyPoint1;
	KeyPoint *maxKP2 = (*(++(distances[maxIndx].rend()))).keyPoint2;
	*/
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

//Interesting algorithm, for the moment works only if all leds are detected 
//TODO: use a quadtree data structure to drastically improve performances
//@keyPoints: vector containig identified leds in the image
//@image: Mat containing thresholded image with identified leds
//@tolerance: tolerance in the alignement pixels
vector<Point2f> patternMirko(vector<Point2f> &keyPoints, Mat &image, int tolerance) {
	
	//vector<Point2f> alignedPoints[4];
	//set<Point2f, orderByX> alPoints[4];
	int numOfPoints = keyPoints.size();
	double angle = 0;	//angle of the line
	double m;			//angolar coefficent of the line
	int setNumber = 0;			//number of aligned sets found;

	vector<Point2f> alignedPoints[4];	//TODO: use an array of vectors of POINTERS to Point2f

	long alignedPointsHash[4];
	//look for the 4 sets of 3 aligned points
	for (int i = 0; i < numOfPoints; i++) {
		Point2f *p1 = &keyPoints[i];
		//for each couple of points...
		for (int j = 0; j < numOfPoints; j++) {
			Point2f *p2 = &keyPoints[j];
			if (p1 != p2) {
				//... compute the equation of the line laying on p1 and p2...
				float dx = p1->x - p2->x;
				float q;
				float m;
				if (dx != 0) {
					m = (p1->y - p2->y) / dx;
					q = p1->y - m*p1->x;
				} 
				else {
					q = NAN;	//CHECK: probably a division by zero returns NAN anyway
				}
				//... and look for another point that satisfies the equation
				for (int k = 0; k < numOfPoints; k++) {
					Point2f *p3 = &keyPoints[k];
					bool validSet = false;
					if (p3 != p1 && p3 != p2) {
						if (q != NAN) {
							if (p3->y - m*(p3->x) < q + tolerance && p3->y - m*(p3->x) > q - tolerance) {
								validSet = true;
							}
						}
						else if (p3->x - p1->x == 0) {
							validSet = true;
						}
					}
					if (validSet) {
						line(image, *p1, *p2, Scalar(0, 0, 255));
						imshow("Thresholded Image", image);
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
															  <<  " p2[" << p2->x << "," << p2->y << "]"
															  <<  " p3[" << p3->x << "," << p3->y << "]";
						}
					}
				}
			}
		}
	}

	//compute the mass center for every set
	Point2f massCenter[4];
	for (int i = 0; i < 4; i++) {
		int x = 0, y = 0;
		for each(Point2f p in alignedPoints[i]) {
			x += p.x;
			y += p.y;
		}
		massCenter[i] = Point2f(x / 3, y / 3);
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
				float dist = myDistance(massCenter[j], massCenter[k]);
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


	vector<Point2f> ledPattern = vector<Point2f>(8);
	int count = 0;
	for (int i = 0; i < 2; i++) {
		double minDist = INT_MAX, maxDist = 0;
		int minIndx[2], maxIndx[2];
		vector<Point2f> alignedSet = alignedPoints[lines[0]];
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				if (i != j) {
					double dist = myDistance(alignedSet[i], alignedSet[j]);
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
			ledPattern[count++] = alignedSet[minIndx[0]];
			ledPattern[count++] = alignedSet[minIndx[1]];
			ledPattern[count++] = alignedSet[maxIndx[1]];
		}
		else if (minIndx[0] == maxIndx[1]) {
			ledPattern[count++] = alignedSet[minIndx[0]];
			ledPattern[count++] = alignedSet[minIndx[1]];
			ledPattern[count++] = alignedSet[maxIndx[0]];
		}
		else if (minIndx[1] == maxIndx[0]) {
			ledPattern[count++] = alignedSet[minIndx[1]];
			ledPattern[count++] = alignedSet[minIndx[0]];
			ledPattern[count++] = alignedSet[maxIndx[1]];
		}
		else {
			ledPattern[count++] = alignedSet[minIndx[1]];
			ledPattern[count++] = alignedSet[minIndx[0]];
			ledPattern[count++] = alignedSet[maxIndx[0]];
		}
	}

	for each (Point2f p in alignedPoints[2]) {
		if (&p != &ledPattern[0] && &p != &ledPattern[3]) {
			ledPattern[6] = p;
			break;
		}
	}
	for each (Point2f p in alignedPoints[3]) {
		if (&p != &ledPattern[2] && &p != &ledPattern[5]) {
			ledPattern[7] = p;
			break;
		}
	}

	return ledPattern;
}


//---Private functions---

float myDistance(Point2f &point1, Point2f &point2) {
	return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
}

void drawDetectedLed(Mat &image, Point2f &keyPoint, string &number) {
	putText(image, number, keyPoint, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255),4);
	imshow("Thresholded Image", image);
	waitKey(25);
}

//---Callback functions for sliders in GUI---

inline void tbColorCallback(int state, void* userdata) {
	int *ptr = (int*)userdata;
	*ptr = state;
	return;
}

inline void tbBlobCallback(int state, void* userdata) {
	float *ptr = (float*)userdata;
	*ptr = float(state) / 1000;
	return;
}



