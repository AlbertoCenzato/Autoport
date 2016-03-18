//Copyright (c) 2016 Alberto Cenzato. All rights reserved.

#include <iostream>
#include <set>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;
using namespace cv;

#ifndef IMGANALYSIS_H
#define IMGANALISIS_H

class ImgAnalysis {

	Mat *tempImg;
	Rect *regionOfInterest;
	Scalar startingLow;
	Scalar startingHigh;
	Scalar low;
	Scalar high;
	SimpleBlobDetector::Params params;
	SimpleBlobDetector::Params startingParams;
	static const int TOL = 20;
	int tolerance;
	vector<Point2f> *points;

public:

	ImgAnalysis(Scalar startingLow, Scalar startingHigh, SimpleBlobDetector::Params startingParameters, int tol = TOL, Rect *regionOfInterest = NULL) {
		this->regionOfInterest = regionOfInterest;
		this->startingLow  = startingLow;
		this->startingHigh = startingHigh;
		low  = startingLow;
		high = startingHigh;
		startingParams = startingParameters;
		params = startingParameters;
		tolerance = tol;
		points = NULL;
	}

	~ImgAnalysis() {}

	vector<Point2f>* evaluate(Mat &img);
	inline void setTolerance(int tol) {
		tolerance = tol;
	}
	inline void clearAll() {
		regionOfInterest = NULL;
		tolerance = TOL;
		low = startingLow;
		high = startingHigh;
		params = startingParams;
	}
	static vector<Point2f> pattern1(vector<Point2f> &, Mat &);
	static vector<Point2f> pattern3(vector<Point2f> &, Mat &);
	static vector<Point2f>* patternMirko(vector<Point2f> *, Mat &, int);

private:

	// Processes the input image (in HSV color space) filtering out (setting to black)
	// all colors which are not in the inteval [min,max], the others are set to white.
	// @img: the Mat object (HSV color space) containing the image to process.
	// @min: the lower bound specified in the HSV color space.
	// @max: the upper bound specified in the HSV color space.
	// returns: black and white image as a Mat object.
	void filterByColor() {

		// Sets to white all colors in the threshold inteval [min,max] and to black the others
		inRange(*tempImg, low, high, *tempImg);

		//morphological opening (remove small objects from the foreground)
		erode (*tempImg, *tempImg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(*tempImg, *tempImg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (fill small holes in the foreground)
		dilate(*tempImg, *tempImg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode (*tempImg, *tempImg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		return;
	}

	// Finds all color blobs that fit the specified paramethers. Blobs which distance
	// from the centroid of the blob set is grater than 2*meanDistance are ignored.
	// @img: image to analyze.
	// @blobParam: paramethers to fit.
	// returns: a vector of Point2f containing centroids cohordinates of detected blobs.
	void findBlobs() {

		Ptr<SimpleBlobDetector> featureDetector = SimpleBlobDetector::create(params);
		vector<KeyPoint> *keyPoints = new vector<KeyPoint>();

		//finds the centroids of blobs
		featureDetector->detect(*tempImg, *keyPoints);  //TODO: use a mask (see detect method description) to improve performances

		points = new vector<Point2f>(keyPoints->size());
		// Draw detected blobs as red circles.
		// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
		drawKeypoints(*tempImg, *keyPoints, *tempImg, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		namedWindow("Thresholded Image", WINDOW_NORMAL);
		imshow("Thresholded Image", *tempImg); //show the thresholded img

		KeyPoint::convert(*keyPoints, *points);
		delete keyPoints;

		// Removes points too far from the centroid of the detected points set
		// computes the mean distance from the centroid
		Point2f centr = centroid(*points);
		float meanDist = 0;
		float distances[20];
		uint size = points->size();
		for (uint i = 0; i < size; i++) {
			float dist = distancePointToPoint(centr, points->at(i));
			meanDist += dist;
			distances[i] = dist;
		}
		meanDist = meanDist / size;

		// removes points
		for (uint i = 0; i < size; i++) {
			if (distances[i] > 2 * meanDist) {
				points->at(i) = points->at(size - 1);
				points->erase(--points->end());
			}
		}

		//draws detected points
		for (uint i = 0; i < size; i++) {
			Point2f p = points->at(i);
			circle(*tempImg, p, 10, Scalar(0, 255, 0), 3);
		}

		return;
	}

	static inline float distancePointToPoint(Point2f &p1, Point2f &p2) {
		return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
	}

	static inline void drawDetectedLed(Mat &image, const Point2f &keyPoint, const string &number) {
		putText(image, number, keyPoint, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255),4);
		imshow("Thresholded Image", image);
		waitKey(25);
	}

	static inline Point2f centroid(const vector<Point2f> &points) {
		float x = 0;
		float y = 0;
		uint size = points.size();
		for (uint i = 0; i < size; i++) {
			Point2f p = points.at(i);
			x += p.x;
			y += p.y;
		}
		return Point2f(x/size, y/size);
	}

};

#endif
