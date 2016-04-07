//Copyright (c) 2016 Alberto Cenzato. All rights reserved.

#include <iostream>
#include <set>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "GenPurpFunc.h"

using namespace std;
using namespace cv;

#ifndef IMGANALYSIS_H
#define IMGANALISIS_H

enum LedColor {
	RED,
	BLUE
};

class ImgAnalysis {

	Rect *regionOfInterest;
	Scalar low;
	Scalar high;
	static const int COLOR_TOLERANCE = 20;
	int colorTolerance;
	static const int ROI_TOLERANCE = 100;
	int ROItolerance;	//region of interest cropping tolerance [px]
	static const int SIZE_TOLERANCE = 20;
	int sizeTolerance;
	static const int SIZE_SUP_TOLERANCE = 128;
	int sizeSupTolerance;
	vector<KeyPoint> *keyPoints;
	SimpleBlobDetector::Params *params;
	int colorConversion;
	function<void(vector<KeyPoint>*, Mat&, int)> patternAnalysis;

public:

	ImgAnalysis(const Scalar &low, const Scalar &high, LedColor ledColor, function<void(vector<KeyPoint>*, Mat&, int)> patternAnalysis, Rect *regionOfInterest = NULL) {
		this->regionOfInterest = regionOfInterest;
		this->low  = low;
		this->high = high;

		params = new SimpleBlobDetector::Params();
		params->filterByColor = true;
		params->blobColor = 255;
		params->filterByInertia = false;
		params->minInertiaRatio = 0.3;
		params->maxInertiaRatio = 1;
		params->filterByArea = true;
		params->minArea = 30;
		params->maxArea = 1000;
		params->filterByConvexity = false;
		params->minConvexity = 0.2;
		params->maxConvexity = 1;
		params->filterByCircularity = false;
		params->minCircularity = 0.2;
		params->maxCircularity = 1;

		if(ledColor == LedColor::RED) colorConversion = COLOR_RGB2HSV;
		else						  colorConversion = COLOR_BGR2HSV;

		colorTolerance   = COLOR_TOLERANCE;
		ROItolerance     = ROI_TOLERANCE;
		sizeTolerance    = SIZE_TOLERANCE;
		sizeSupTolerance = SIZE_SUP_TOLERANCE;

		this->patternAnalysis = patternAnalysis;

		keyPoints = NULL;
	}

	~ImgAnalysis() {
		delete keyPoints;
		delete params;
	}

	bool evaluate(Mat &, vector<Point2f> *, float);
	ImgAnalysis* setROItolerance(int);
	ImgAnalysis* setColorTolerance(int);
	ImgAnalysis* setSizeTolerance(int);
	ImgAnalysis* setSizeSupTolerance(int);

	static vector<Point2f> pattern1(vector<Point2f> &, Mat &);
	static vector<Point2f> pattern3(vector<Point2f> &, Mat &);

private:

	// Processes the input image (in HSV color space) filtering out (setting to black)
	// all colors which are not in the interval [min,max], the others are set to white.
	// @img: the Mat object (HSV color space) containing the image to process.
	// @min: the lower bound specified in the HSV color space.
	// @max: the upper bound specified in the HSV color space.
	// returns: black and white image as a Mat object.
	void filterByColor(Mat *hsvImg, Mat *colorFilteredImg) {

		// Sets to white all colors in the threshold interval [min,max] and to black the others
		inRange(*hsvImg, low, high, *colorFilteredImg);

		//morphological opening (remove small objects from the foreground)
		erode (*colorFilteredImg, *colorFilteredImg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(*colorFilteredImg, *colorFilteredImg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (fill small holes in the foreground)
		dilate(*colorFilteredImg, *colorFilteredImg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode (*colorFilteredImg, *colorFilteredImg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		return;
	}

	// Finds all color blobs that fit the specified parameters. Blobs which distance
	// from the centroid of the blob set is grater than 2*meanDistance are ignored.
	// @img: image to analyze.
	// @blobParam: parameters to fit.
	// returns: a vector of Point2f containing centroids coordinates of detected blobs.
	void findBlobs(Mat *colorFilteredImg) {

		//TODO: check this way of computing valid led sizes interval: it can lead to
		//a degeneration of the interval amplitude continuously increasing it in presence
		//of noise similar to leds, maybe it wold be better to use the medium value of led sizes
		int length = 10;
		if(keyPoints != NULL) {
			length = keyPoints->size();
			int minSize = INT_MAX;
			int maxSize = INT_MIN;
			for(int i = 0; i < length; i++) {
				int size = keyPoints->at(i).size;
				if(size < minSize) minSize = size;
				if(size > maxSize) maxSize = size;
			}
			params->maxArea = maxSize + sizeTolerance;
			minSize = minSize - sizeTolerance;
			params->minArea = minSize > 0 ? minSize : 0;
			delete keyPoints;
		}
		keyPoints = new vector<KeyPoint>(2*length);

		//finds the centroids of blobs

		Ptr<SimpleBlobDetector> featureDetector = SimpleBlobDetector::create(*params);
		featureDetector->detect(*colorFilteredImg, *keyPoints);

		//delete ledPoints;
		//ledPoints = new vector<Point2f>(keyPoints->size());
		
		// Draw detected blobs as red circles.
		drawKeypoints(*colorFilteredImg, *keyPoints, *colorFilteredImg, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		namedWindow("Thresholded Image", WINDOW_NORMAL);
		imshow("Thresholded Image", *colorFilteredImg); //show the thresholded img

		//KeyPoint::convert(*keyPoints, *ledPoints);
		// Remove points too far from the centroid of the detected points set
		// compute the mean distance from the centroid
		Point2f centr = GenPurpFunc::centroid(*keyPoints);
		float meanDist = 0;
		uint size = keyPoints->size();
		float *distances = new float[size];
		for (uint i = 0; i < size; i++) {
			float dist = GenPurpFunc::distancePointToPoint(centr, keyPoints->at(i).pt);
			meanDist += dist;
			distances[i] = dist;
		}
		meanDist = meanDist / size;

		// remove points
		//size = ledPoints->size();
		for (uint i = 0; i < size; ) {
			if (distances[i] > 2 * meanDist) {
				keyPoints->at(i) = keyPoints->at(size - 1);
				distances[i] = distances[size - 1];
				keyPoints->erase(--keyPoints->end());
			}
			else i++;
			size = keyPoints->size();
		}
		delete[] distances;

		//size = ledPoints->size();
		//draws detected points
		for (uint i = 0; i < size; i++) {
			Point2f p = keyPoints->at(i).pt;
			circle(*colorFilteredImg, p, 10, Scalar(0, 255, 0), 3);
		}

		return;
	}


};

#endif
