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
	vector<KeyPoint> *keyPoints;
	vector<Point2f>  *ledPoints;
	Ptr<SimpleBlobDetector> featureDetector;
	int colorConversion;

public:

	ImgAnalysis(Scalar &low, Scalar &high, const SimpleBlobDetector::Params &params, LedColor ledColor,
				int colorTolerance = COLOR_TOLERANCE, int ROItolerance = ROI_TOLERANCE, Rect *regionOfInterest = NULL) {
		this->regionOfInterest = regionOfInterest;
		this->low  = low;
		this->high = high;
		this->featureDetector = SimpleBlobDetector::create(params);
		if(ledColor == LedColor::RED) colorConversion = COLOR_RGB2HSV;
		else						  colorConversion = COLOR_BGR2HSV;
		this->colorTolerance = colorTolerance;
		this->ROItolerance = ROItolerance;
		keyPoints = NULL;
	}

	~ImgAnalysis() {
		delete keyPoints;
		//TODO: delete also featureDetector??
	}

	vector<Point2f>* evaluate(Mat &img);

	static vector<Point2f> pattern1(vector<Point2f> &, Mat &);
	static vector<Point2f> pattern3(vector<Point2f> &, Mat &);
	static vector<KeyPoint>* patternMirko(vector<KeyPoint> *, Mat &, int);

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

		int oldSize = keyPoints->size();
		delete keyPoints;
		keyPoints = new vector<KeyPoint>(2*oldSize);

		//finds the centroids of blobs
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
		Point2f centr = centroid(*keyPoints);
		float meanDist = 0;
		uint size = keyPoints->size();
		float *distances = new float[size];
		for (uint i = 0; i < size; i++) {
			float dist = distancePointToPoint(centr, keyPoints->at(i).pt);
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

	static inline float distancePointToPoint(Point2f &p1, Point2f &p2) {
		return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
	}

	static inline void drawDetectedLed(Mat &image, const Point2f &keyPoint, const string &number) {
		putText(image, number, keyPoint, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255),4);
		imshow("Thresholded Image", image);
		waitKey(25);
	}

	static inline Point2f centroid(const vector<KeyPoint> &points) {
		float x = 0;
		float y = 0;
		uint size = points.size();
		for (uint i = 0; i < size; i++) {
			Point2f p = points.at(i).pt;
			x += p.x;
			y += p.y;
		}
		return Point2f(x/size, y/size);
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
