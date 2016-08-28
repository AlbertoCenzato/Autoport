//Copyright (c) 2016 Alberto Cenzato. All rights reserved.

#include "ImgAnalysis.hpp"

#include <iostream>
#include <set>
#include <chrono>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "GenPurpFunc.hpp"

using namespace cv;
using namespace std;

//--- Structs ---

extern string workingDir;

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
bool ImgAnalysis::evaluate(Mat &image, vector<Point2f> &points, float downscalingFactor) {

	// Crop the full image according to the region of interest
	// Note that this doesn't copy the data
	if(regionOfInterest != NULL)
		image = image(*regionOfInterest);

	Mat *hsvImg = new Mat(image.rows,image.cols,image.depth());
	Mat *colorFilteredImg = new Mat(image.rows,image.cols,image.depth());

	//change color space: from BGR to HSV;
    //TODO: color conversion and filterByColor can be performed with a shader
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
	imwrite(workingDir + "output/filterByColor.jpg", *colorFilteredImg);

	//put in this->points detected blobs that satisfy this->params tolerance
	begin = std::chrono::high_resolution_clock::now();
	findBlobs(colorFilteredImg, downscalingFactor);
	end = std::chrono::high_resolution_clock::now();
	cout << "\nFind blobs: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;
	namedWindow("Blobs found", WINDOW_NORMAL);
	imshow("Blobs found", *colorFilteredImg);
	waitKey(100000);
	imwrite(workingDir + "output/findBlobs.jpg",*colorFilteredImg);

	//order this->points accordingly to the led pattern numbering
	begin = std::chrono::high_resolution_clock::now();
	patternAnalysis->evaluate(ledPoints, *colorFilteredImg, 10);
	end = std::chrono::high_resolution_clock::now();
	cout << "\nPattern: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;
	imwrite(workingDir + "output/patternMirko.jpg",*colorFilteredImg);

	delete colorFilteredImg;

	GenPurpFunc::printPointVector(*ledPoints);

	Interval<int> hue = Interval<int>();
	hue.low  = 0;
	hue.high = 255;
	Interval<int> sat = Interval<int>();
	sat.low  = 0;
	sat.high = 255;
	Interval<int> val = Interval<int>();
	val.low  = 0;
	sat.high = 255;

	int ledPointsLength = ledPoints->size();
	for (int i = 0; i < ledPointsLength; i++) {
		Point2f p = ledPoints->at(i);
		Vec3b color = hsvImg->at<Vec3b>(p);
		if (color[0] > hue.high) hue.high = color[0];
		if (color[1] > sat.high) sat.high = color[1];
		if (color[2] > val.high) val.high = color[2];
		if (color[0] < hue.low)	 hue.low  = color[0];
		if (color[1] < sat.low)	 sat.low  = color[1];
		if (color[2] < val.low)	 val.low  = color[2];
	}
	colorInterval->low  = Scalar(hue.low  - colorTolerance, sat.low  - colorTolerance, val.low  - colorTolerance);
	colorInterval->high = Scalar(hue.high + colorTolerance, sat.high + colorTolerance, val.high + colorTolerance);
	delete hsvImg;

	findROI();

	//delete points;
	points = *ledPoints;

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


