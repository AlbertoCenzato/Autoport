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

extern string resourcesPath;

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
bool ImgAnalysis::evaluate(Mat &image, vector<Point2f> *points, float downscalingFactor) {

	// Crop the full image according to the region of interest
	// Note that this doesn't copy the data
	if(regionOfInterest != NULL)
		image = image(*regionOfInterest);

	Mat *hsvImg = new Mat(image.rows,image.cols,image.depth());
	Mat *colorFilteredImg = new Mat(image.rows,image.cols,image.depth());

	//change color space: from BGR to HSV;
	auto begin = std::chrono::high_resolution_clock::now();
	cvtColor(image,*hsvImg,colorConversion);

	auto end = std::chrono::high_resolution_clock::now();
	cout << "\nConvert color: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;
	namedWindow("Cropped image", WINDOW_NORMAL);
	imshow("Cropped image", *hsvImg);
	waitKey(1);

	//filter the color according to this->low and this->high tolerances
	begin = std::chrono::high_resolution_clock::now();
	filterByColor(hsvImg,colorFilteredImg);
	end = std::chrono::high_resolution_clock::now();
	cout << "\nFilter color: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;
	namedWindow("Filtered image", WINDOW_NORMAL);
	imshow("Filtered image", *colorFilteredImg);
	waitKey(1000);
	imwrite(resourcesPath + "output/filterByColor.jpg", *colorFilteredImg);

	//put in this->points detected blobs that satisfy this->params tolerance
	begin = std::chrono::high_resolution_clock::now();
	findBlobs(colorFilteredImg, downscalingFactor);
	end = std::chrono::high_resolution_clock::now();
	cout << "\nFind blobs: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;
	namedWindow("Blobs found", WINDOW_NORMAL);
	imshow("Blobs found", *colorFilteredImg);
	waitKey(100000);
	imwrite(resourcesPath + "output/findBlobs.jpg",*colorFilteredImg);

	//order this->points accordingly to the led pattern numbering
	begin = std::chrono::high_resolution_clock::now();
	patternAnalysis->evaluate(ledPoints, *colorFilteredImg, 10);
	end = std::chrono::high_resolution_clock::now();
	cout << "\nPattern: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;
	imwrite(resourcesPath + "output/patternMirko.jpg",*colorFilteredImg);

	delete colorFilteredImg;

	GenPurpFunc::printPointVector(*ledPoints);

	int maxH = 0, 	maxS = 0, 	maxV = 0;
	int minH = 255, minS = 255, minV = 255;

	int ledPointsLength = ledPoints->size();
	for (int i = 0; i < ledPointsLength; i++) {
		Point2f p = ledPoints->at(i);
		//Vec3b color = hsvImg->at<Vec3b>(p);
		Vec3b color = hsvImg->at<Vec3b>(p);
		if (color[0] > maxH)	maxH = color[0];
		if (color[1] > maxS)	maxS = color[1];
		if (color[2] > maxV)	maxV = color[2];
		if (color[0] < minH)	minH = color[0];
		if (color[1] < minS)	minS = color[1];
		if (color[2] < minV)	minV = color[2];
	}
	colorInterval->low  = Scalar(minH - colorTolerance, minS - colorTolerance, minV - colorTolerance);
	colorInterval->high = Scalar(maxH + colorTolerance, maxS + colorTolerance, maxV + colorTolerance);
	delete hsvImg;

	Point2f *maxX = GenPurpFunc::findMaxXInVec(*ledPoints);
	Point2f *maxY = GenPurpFunc::findMaxYInVec(*ledPoints);
	Point2f *minX = GenPurpFunc::findMinXInVec(*ledPoints);
	Point2f *minY = GenPurpFunc::findMinYInVec(*ledPoints);
	delete regionOfInterest;
	regionOfInterest = new Rect(minX->x - ROItolerance, minY->y - ROItolerance, maxX->x - minX->x + 2*ROItolerance, maxY->y - minY->y + 2*ROItolerance);

	delete points;
	points = ledPoints;

	int averageSize = (oldKeyPointSizeInterval->low + oldKeyPointSizeInterval->high)/2;

	return averageSize > sizeSupTolerance;
}

ImgAnalysis* ImgAnalysis::setROItolerance(int ROItolerance) {
	this->ROItolerance = ROItolerance;
	return this;
}
ImgAnalysis* ImgAnalysis::setColorTolerance(int colorTolerance) {
	this->colorTolerance = colorTolerance;
	return this;
}
ImgAnalysis* ImgAnalysis::setSizeTolerance(int sizeTolerance) {
	this->sizeTolerance = sizeTolerance;
	return this;
}
ImgAnalysis* ImgAnalysis::setSizeSupTolerance(int sizeSupTolerance) {
	this->sizeSupTolerance = sizeSupTolerance;
	return this;
}


