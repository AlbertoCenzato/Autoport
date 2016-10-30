//Copyright (c) 2016 Alberto Cenzato. All rights reserved.

#pragma once

#include "PatternAnalysis.hpp"

using namespace std;
using namespace cv;

class ImgAnalysis {

	Interval<Scalar> colorInterval;

	//int colorTolerance;
	SimpleBlobDetector::Params params;
	int colorConversion;

	Mat originalImage;

public:

	ImgAnalysis(const Interval<Scalar> &colorInterval, LedColor ledColor) {
		constructor(colorInterval, ledColor);
	}

	ImgAnalysis() {

		Settings& settings = Settings::getInstance();
		Scalar low  = Scalar(settings.hue.min, settings.saturation.min, settings.value.min);
		Scalar high = Scalar(settings.hue.max, settings.saturation.max, settings.value.max);

		//auto patternAnalysis = PatternAnalysis();
		constructor(Interval<Scalar>(low,high), settings.patternColor);
	}

	~ImgAnalysis() {}

	bool evaluate(Mat &image, vector<LedDescriptor> &points, float downscalingFactor);
	//ImgAnalysis* setColorTolerance	(int);
	ImgAnalysis* setSizeTolerance	(int);
	ImgAnalysis* setSizeSupTolerance(int);
	ImgAnalysis* setColorInterval	(const Interval<Scalar> &colorInterval);
	ImgAnalysis* setBlobSizeInterval(const Interval<int> 	&blobSizeInterval);

	void getColorInterval(Interval<Scalar> &colorInterval);

private:

	void constructor(const Interval<Scalar> &colorInterval, LedColor ledColor) {
		this->colorInterval = colorInterval; //FIXME: what happens here? is it coping the object? Probably yes

		if(ledColor == LedColor::RED) colorConversion = COLOR_RGB2HSV;
		else						  colorConversion = COLOR_BGR2HSV;

		params = SimpleBlobDetector::Params();

		params.filterByColor = true;
		params.blobColor = 255;
		params.filterByArea = false;	//FIXME: set this to true and find default params
		//params.minArea = 0;
		//params.maxArea = 10000;
		params.filterByInertia = false;
		params.filterByConvexity = false;
		params.filterByCircularity = false;
		//params.minCircularity = 0.5;
		//params.maxCircularity = 1;

		Settings& settings = Settings::getInstance();
		//colorTolerance   = settings.colorTolerance;
	}

	// Processes the input image (in HSV color space) filtering out (setting to black)
	// all colors which are not in the interval [min,max], the others are set to white.
	// @img: the Mat object (HSV color space) containing the image to process.
	// @min: the lower bound specified in the HSV color space.
	// @max: the upper bound specified in the HSV color space.
	// returns: black and white image as a Mat object.
	void filterByColor(const Mat &hsvImg, Mat &colorFilteredImg) {

		// Sets to white all colors in the threshold interval [min,max] and to black the others
		inRange(hsvImg, colorInterval.min, colorInterval.max, colorFilteredImg);

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
	int findBlobs(const Mat &colorFilteredImg, vector<LedDescriptor>& ledPoints, float downscalingFactor) {

		//TODO: check this way of computing valid led sizes interval: it can lead to
		//a degeneration of the interval amplitude continuously increasing it in presence
		//of noise similar to leds, maybe it would be better to use the medium value of led sizes

		ledPoints.clear();
		vector<KeyPoint> keyPoints(10);

		Ptr<SimpleBlobDetector> featureDetector = SimpleBlobDetector::create(params);
		featureDetector->detect(colorFilteredImg, keyPoints);

		const int SIZE = keyPoints.size();
		cout << "KeyPoints SIZE: " << SIZE << endl;
		if(SIZE > 0) {

			for(int i = 0; i < SIZE; ++i) {
				Point2f position = keyPoints[i].pt;
				Scalar  color 	 = originalImage.at<Vec3b>(position);
				ledPoints.push_back(LedDescriptor(position,color,keyPoints[i].size));
			}

			// Remove points too far from the centroid of the detected points set
			// compute the mean distance from the centroid
			Point2f centr = LedDescriptor::centroid(ledPoints);
			float meanDist = 0;
			float *distances = new float[SIZE];	// TODO: try to use another way, dangerous pointer
			for (int i = 0; i < SIZE; i++) {
				float dist = GenPurpFunc::distPoint2Point(centr, ledPoints[i].getPosition());
				meanDist += dist;
				distances[i] = dist;
			}
			meanDist = meanDist / SIZE;

			// remove points too far away points
			int size = SIZE;
			for (int i = 0; i < size; ) {
				if (distances[i] > 2 * meanDist) {
					ledPoints[i] = ledPoints[size - 1];
					distances[i] = distances[size - 1];
					ledPoints.pop_back();
				}
				else i++;
				size = ledPoints.size();
			}

			delete [] distances;
		}

		return ledPoints.size();
	}




};

