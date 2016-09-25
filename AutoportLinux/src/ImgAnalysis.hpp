//Copyright (c) 2016 Alberto Cenzato. All rights reserved.

#pragma once

#include "PatternAnalysis.hpp"
#include "Settings.hpp"

using namespace std;
using namespace cv;

enum LedColor {
	RED,
	BLUE
};


class ImgAnalysis {

	Rect regionOfInterest;
	Interval<Scalar> colorInterval;

	int colorTolerance;
	int ROItolerance;	//region of interest cropping tolerance [px]
	int sizeTolerance;
	int sizeSupTolerance;
	vector<Point2f> ledPoints;
	SimpleBlobDetector::Params params;
	int colorConversion;
	PatternAnalysis patternAnalysis;

	Interval<float> *oldKeyPointSizeInterval;

public:

	ImgAnalysis(const Interval<Scalar> &colorInterval, LedColor ledColor, PatternAnalysis &patternAnalysis, const Rect &regionOfInterest = Rect(0,0,0,0)) {
		constructor(colorInterval, ledColor, patternAnalysis, regionOfInterest);
	}

	ImgAnalysis(LedColor ledColor) {

		Scalar low = Scalar(Settings::hue.low,Settings::saturation.low,Settings::value.low);
		Scalar high = Scalar(Settings::hue.high,Settings::saturation.high,Settings::value.high);

		auto patternAnalysis = PatternAnalysis();
		constructor(Interval<Scalar>(low,high), ledColor, patternAnalysis);
	}

	~ImgAnalysis() {}

	bool evaluate(Mat &, vector<Point2f> &, float);
	ImgAnalysis* setROItolerance(int);
	ImgAnalysis* setColorTolerance(int);
	ImgAnalysis* setSizeTolerance(int);
	ImgAnalysis* setSizeSupTolerance(int);

private:

	void constructor(const Interval<Scalar> &colorInterval, LedColor ledColor, PatternAnalysis &patternAnalysis, const Rect &regionOfInterest = Rect(0,0,0,0)) {
		this->colorInterval = colorInterval; //FIXME: what happens here? is it coping the object? Probably yes

		if(ledColor == LedColor::RED) colorConversion = COLOR_RGB2HSV;
		else						  colorConversion = COLOR_BGR2HSV;

		this->patternAnalysis = patternAnalysis;
		this->regionOfInterest = regionOfInterest;

		params = SimpleBlobDetector::Params();
		params.filterByColor = true;
		params.blobColor = 255;
		params.filterByArea = true;
		params.minArea = 1000;
		params.maxArea = 5000;
		params.filterByInertia = false;
		params.filterByConvexity = false;
		params.filterByCircularity = false;

		colorTolerance   = Settings::colorTolerance;
		ROItolerance     = Settings::ROITolerance;
		sizeTolerance    = Settings::sizeTolerance;
		sizeSupTolerance = Settings::sizeSupTolerance;

		oldKeyPointSizeInterval = nullptr;
	}

	// Processes the input image (in HSV color space) filtering out (setting to black)
	// all colors which are not in the interval [min,max], the others are set to white.
	// @img: the Mat object (HSV color space) containing the image to process.
	// @min: the lower bound specified in the HSV color space.
	// @max: the upper bound specified in the HSV color space.
	// returns: black and white image as a Mat object.
	void filterByColor(const Mat &hsvImg, Mat &colorFilteredImg) {

		// Sets to white all colors in the threshold interval [min,max] and to black the others
		inRange(hsvImg, colorInterval.low, colorInterval.high, colorFilteredImg);

		//morphological opening (remove small objects from the foreground)
		erode (colorFilteredImg, colorFilteredImg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(colorFilteredImg, colorFilteredImg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (fill small holes in the foreground)
		dilate(colorFilteredImg, colorFilteredImg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode (colorFilteredImg, colorFilteredImg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		return;
	}

	// Finds all color blobs that fit the specified parameters. Blobs which distance
	// from the centroid of the blob set is grater than 2*meanDistance are ignored.
	// @img: image to analyze.
	// @blobParam: parameters to fit.
	// returns: a vector of Point2f containing centroids coordinates of detected blobs.
	void findBlobs(Mat &colorFilteredImg, float downscalingFactor) {

		//TODO: check this way of computing valid led sizes interval: it can lead to
		//a degeneration of the interval amplitude continuously increasing it in presence
		//of noise similar to leds, maybe it would be better to use the medium value of led sizes

		if(oldKeyPointSizeInterval != NULL) {
			params.maxArea = oldKeyPointSizeInterval->high;
			params.minArea = oldKeyPointSizeInterval->low;
		}
		else {
			oldKeyPointSizeInterval = new Interval<float>();
		}

		vector<KeyPoint> keyPoints = vector<KeyPoint>();

		Ptr<SimpleBlobDetector> featureDetector = SimpleBlobDetector::create(params);
		featureDetector->detect(colorFilteredImg, keyPoints);

		ledPoints = vector<Point2f>(keyPoints.size());
		KeyPoint::convert(keyPoints, ledPoints);
		
		// Remove points too far from the centroid of the detected points set
		// compute the mean distance from the centroid
		Point2f centr = GenPurpFunc::centroid(ledPoints);
		float meanDist = 0;
		uint size = ledPoints.size();
		float *distances = new float[size];	// TODO: try to use another way, dangerous pointer
		for (uint i = 0; i < size; i++) {
			float dist = GenPurpFunc::distancePointToPoint(centr, ledPoints.at(i));
			meanDist += dist;
			distances[i] = dist;
		}
		meanDist = meanDist / size;

		// remove points
		for (uint i = 0; i < size; ) {
			if (distances[i] > 2 * meanDist) {
				ledPoints.at(i) = ledPoints.at(size - 1);
				distances[i] = distances[size - 1];
				ledPoints.erase(--ledPoints.end());
			}
			else i++;
			size = ledPoints.size();
		}
		delete[] distances;

		cout << "ledPoints length: " << size << endl;

		//draws detected points
		for (uint i = 0; i < size; i++) {
			Point2f p = ledPoints.at(i);
			Scalar color = Scalar(150, 150, 0);
			circle(colorFilteredImg, p, 30, color, 10);
		}

		float min = keyPoints.at(0).size;
		float max = min;
		for(uint i = 1; i < keyPoints.size(); i++) {
			float newSize = keyPoints.at(i).size;
			if(newSize < min)	min = newSize;
			if(newSize > max)	max = newSize;
		}
		oldKeyPointSizeInterval->low  = min;
		oldKeyPointSizeInterval->high = max;


		return;
	}


	//Finds the Region Of Interest that surrounds the pattern
	void findROI() {

		Point2f *point = &ledPoints.at(0);
		Interval<float> x = Interval<float>();
		x.low  = point->x;
		x.high = point->x;
		Interval<float> y = Interval<float>();
		y.low  = point->y;
		y.high = point->y;
		uint size = ledPoints.size();
		for (uint i = 1; i < size; i++) {
			point = &ledPoints.at(i);
			if (point->x < x.low)	x.low  = point->x;
			if (point->x > x.high)	x.high = point->x;
			if (point->y < y.low)	y.low  = point->y;
			if (point->y < y.high)	y.high = point->y;
		}
		regionOfInterest = Rect(x.low - ROItolerance, y.low - ROItolerance, x.high - x.low + 2*ROItolerance, y.high - y.low + 2*ROItolerance);

		return;
	}

};

