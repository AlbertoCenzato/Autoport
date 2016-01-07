#include "stdafx.h"
#include <iostream>
#include <set>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\features2d\features2d.hpp>

using namespace cv;
using namespace std;

//---Private structs---
struct Distance {
	float dist = 0;
	KeyPoint *keyPoint1;
	//KeyPoint *keyPoint2;
	int keyPoint2;
};

struct lessDist : binary_function <Distance, Distance, bool> {
	bool operator() (const Distance &d1, const Distance &d2) const { return d1.dist < d2.dist; }
};

//---Function declaration---
inline void tbColorCallback(int state, void* userdata);
inline void tbBlobCallback(int state, void* userdata);
inline float myDistance(cv::KeyPoint*, cv::KeyPoint*);

//---Function definition---

//simple image analysis, with color filtering
vector<KeyPoint> imgLedDetection(string &imgName)
{

	//---Color filtering----

	int iLowH = 150;
	int iHighH = 180;

	int iLowS = 100;
	int iHighS = 200;

	int iLowV = 100;
	int iHighV = 255;

	int64 start = getTickCount();

	Mat imgOriginal = imread(imgName, cv::ImreadModes::IMREAD_COLOR); // read a new frame from video

	//Convert the captured frame from BGR to HSV
	Mat imgHSV;
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); 

	//Threshold the image
	Mat imgThresholded;
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
	params.filterByArea = false;
	params.minArea = 0;
	params.maxArea = 100;
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

	for (uint i = 0; i < keypoints.size(); i++) {
		Point pt = keypoints[i].pt;
		printf("Point %d - x:%d      y:%d\n", i+1, pt.x, pt.y);
	}

	namedWindow("Thresholded Image", WINDOW_NORMAL);
	namedWindow("Original", WINDOW_NORMAL);

	imshow("Thresholded Image", imgThresholded); //show the thresholded image
	imshow("Original", imgOriginal); //show the original image

	waitKey(25);

	return keypoints;
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
vector<KeyPoint> pattern1(vector<KeyPoint> &keyPoints) {
	
	//compute the distances between points
	
	set<Distance, lessDist> distances[12];
	//set<Distance, lessDist>	distances;	//TODO: preallocate the exact vector size if possible...
	Distance d;
	int k = 0;
	for (uint i = 0; i < 12; i++)	{
		for (int j = 0; j < 12; j++) {
			if (i != j) {
				d.keyPoint1 = &keyPoints[i];
				d.keyPoint2 = j;
				d.dist = myDistance(d.keyPoint1, &(keyPoints[j]));
				distances[i].insert(d);
			}
		}
	}

	//print distances
	for (int j = 0; j < 12; j++) {
		cout << "\nPoint " << j+1 << ":";
		int i = 1;
		for (std::set<Distance, lessDist>::iterator it = distances[j].begin(); it != distances[j].end(); ++it) {
			Distance d = *it;
			cout << "\n   Distance " << i++ << ": " << d.dist;
		}
	}

	int minIndx = 0;
	int maxIndx = 0;
	for (int i = 0; i < 12; i++){
		Distance newMinDist = *(distances[i].begin());
		Distance newMaxDist = *(++distances[i].rend());
		if (newMinDist.dist < (*(distances[minIndx].begin())).dist)
			minIndx = i;
		if (newMaxDist.dist > (*(++distances[maxIndx].rend())).dist)
			maxIndx = i;
	}

	cout << "\nMin distance: " << (*(distances[minIndx].begin())).dist;
	cout << "\nMax distance: " << (*(++distances[maxIndx].rend())).dist;
	
	set<Distance, lessDist> patternPoints[12];
	
	/*
	KeyPoint *minKP1 = (*(distances[minIndx].begin())).keyPoint1;
	KeyPoint *minKP2 = (*(distances[minIndx].begin())).keyPoint2;
	KeyPoint *maxKP1 = (*(++(distances[maxIndx].rend()))).keyPoint1;
	KeyPoint *maxKP2 = (*(++(distances[maxIndx].rend()))).keyPoint2;
	*/
	int minKP1 = minIndx;
	int minKP2 = (*(distances[minIndx].begin())).keyPoint2;
	int maxKP1 = maxIndx;
	int maxKP2 = (*(++distances[maxIndx].rend())).keyPoint2;

	if (minKP1 == maxKP1) {
		patternPoints[11] = distances[minIndx];  //CHECK: retruns by value or by reference??
		distances[minIndx] = set<Distance,lessDist>();
		patternPoints[8] = distances[maxKP2];
		distances[maxKP2] = set<Distance, lessDist>();
		patternPoints[10] = distances[minKP2];
		distances[minKP2] = set<Distance, lessDist>();
	} 
	else if (minKP1 == maxKP2) {
		patternPoints[11] = distances[minKP1];
		distances[minKP1] = set<Distance, lessDist>();
		patternPoints[8]  = distances[maxKP1];
		distances[maxKP1] = set<Distance, lessDist>();
		patternPoints[10] = distances[minKP2];
		distances[minKP2] = set<Distance, lessDist>();
	}
	else if (minKP2 == maxKP1) {
		patternPoints[11] = distances[minKP2];
		distances[minKP2] = set<Distance, lessDist>();
		patternPoints[8]  = distances[maxKP2];
		distances[maxKP2] = set<Distance, lessDist>();
		patternPoints[10] = distances[minKP1];
		distances[minKP1] = set<Distance, lessDist>();
	}
	else {
		patternPoints[11] = distances[minKP2];
		distances[minKP2] = set<Distance, lessDist>();
		patternPoints[8]  = distances[maxKP1];
		distances[maxKP1] = set<Distance, lessDist>();
		patternPoints[10] = distances[minKP1];
		distances[minKP1] = set<Distance, lessDist>();
	}

	set<Distance,lessDist>::reverse_iterator riter = patternPoints[10].rend();
	riter++;
	riter++;
	int kp1 = (*riter).keyPoint2;
	patternPoints[9] = distances[kp1];
	distances[kp1] = set<Distance, lessDist>();
	riter++;
	kp1 = (*riter).keyPoint2;
	patternPoints[5] = distances[kp1];
	distances[kp1] = set<Distance, lessDist>();

	set<Distance, lessDist>::iterator iter1 = patternPoints[5].begin();
	kp1 = (*iter1).keyPoint2;
	iter1++;
	int kp2 = (*iter1).keyPoint2;
	int firstFound = 0;
	for (set<Distance, lessDist>::iterator iter2 = patternPoints[8].begin(); iter2 != patternPoints[8].end();) {
		int kp = (*iter2).keyPoint2;
		iter2++;
		if (kp == kp1) {
			patternPoints[1] = distances[kp1];
			distances[kp1] = set<Distance, lessDist>();
			patternPoints[0] = distances[kp2];
			distances[kp2] = set<Distance, lessDist>();
			iter2 = patternPoints[8].end();
		}
		else if (kp == kp2) {
			patternPoints[1] = distances[kp2];
			distances[kp2] = set<Distance, lessDist>();
			patternPoints[0] = distances[kp1];
			distances[kp1] = set<Distance, lessDist>();
			iter2 = patternPoints[8].end();
		}
	}

	//led 3
	set<Distance, lessDist>::iterator iter = patternPoints[1].begin();
	iter++;
	iter++;
	iter++;
	kp1 = (*iter).keyPoint2;
	patternPoints[2] = distances[kp1];
	distances[kp1] = set<Distance, lessDist>();

	//led 7
	iter = patternPoints[9].begin();
	iter++;
	kp1 = (*iter).keyPoint2;
	patternPoints[6] = distances[kp1];
	distances[kp1] = set<Distance, lessDist>();

	//led 4
	iter = patternPoints[6].begin();
	iter++;
	kp1 = (*iter).keyPoint2;
	patternPoints[3] = distances[kp1];
	distances[kp1] = set<Distance, lessDist>();

	//led 5 e 8
	int fiveAndEight[2];
	int j = 0;
	for (int i = 0; i < 12, j < 2; i++) {
		if (!distances[i].empty()) {
			fiveAndEight[j++] = i;
		}
	}
	/*
	int keyPointFound = 0;
	int fiveAndEight[2];
	for (int i = 0; i < 12 && keyPointFound < 2; i++) {
		kp1 = i;
		bool found = false;
		for (int j = 0; j < 12; j++) {
			if (!patternPoints[j].empty()) {
				if (kp1 == (*(patternPoints[j].begin())).keyPoint1) {
					found = true;
					j = 12;
				}
			}
		}
		if (!found) {
			fiveAndEight[keyPointFound++] = kp1;
		}
	}
	*/

	vector<KeyPoint> finalKeyPoints = vector<KeyPoint>(12);
	for (int i = 0; i < 12; i++) {
		if (!patternPoints[i].empty())
			finalKeyPoints[i] = *((*(patternPoints[i].begin())).keyPoint1);
	}

	for (iter = patternPoints[3].begin(); iter != patternPoints[3].end(); ) {
		kp1 = (*iter).keyPoint2;
		iter++;
		if (fiveAndEight[0] == kp1) {
			finalKeyPoints[4] = *(*(distances[fiveAndEight[0]].begin())).keyPoint1;
			finalKeyPoints[7] = *(*(distances[fiveAndEight[1]].begin())).keyPoint1;
			iter == patternPoints[3].end();
		}
		else if(fiveAndEight[1] == kp1) {
			finalKeyPoints[4] = *(*(distances[fiveAndEight[1]].begin())).keyPoint1;
			finalKeyPoints[7] = *(*(distances[fiveAndEight[0]].begin())).keyPoint1;
			iter == patternPoints[3].end();
		}
	}
	return finalKeyPoints;
}


//---Private functions---

float myDistance(cv::KeyPoint *point1, cv::KeyPoint *point2) {
	return sqrt(pow((*point1).pt.x - (*point2).pt.x, 2) + pow((*point1).pt.y - (*point2).pt.y, 2));
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



