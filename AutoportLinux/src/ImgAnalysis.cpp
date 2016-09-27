//Copyright (c) 2016 Alberto Cenzato. All rights reserved.

#include "ImgAnalysis.hpp"
#include <chrono>

using namespace cv;
using namespace std;

extern string workingDir;

//--- Functions ---

//TODO: make the function accept a pointer to a pattern analysis function
bool ImgAnalysis::evaluate(Mat &image, vector<Point2f> &points, float downscalingFactor) {

	// Crop the full image according to the region of interest
	// Note that this doesn't copy the data
	if(regionOfInterest.height != 0 && regionOfInterest.width != 0)
		image = image(regionOfInterest);

	Mat hsvImg(image.rows,image.cols,image.depth());
	Mat colorFilteredImg(image.rows,image.cols,image.depth());

	//change color space: from BGR to HSV;
    //TODO: color conversion and filterByColor can be performed with a shader (?)
	auto begin = chrono::high_resolution_clock::now();
	cvtColor(image,image,colorConversion);
	auto end = chrono::high_resolution_clock::now();
	cout << "\nConvert color: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;
	/*
	namedWindow("Cropped image", WINDOW_NORMAL);
	imshow("Cropped image", hsvImg);
	waitKey(1);
	*/

	//filter the color according to this->low and this->high tolerances
	begin = chrono::high_resolution_clock::now();
	filterByColor(image,image);
	end = chrono::high_resolution_clock::now();
	cout << "\nFilter color: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;
	/*
	namedWindow("Filtered image", WINDOW_NORMAL);
	imshow("Filtered image", colorFilteredImg);
	waitKey(1);
	imwrite(workingDir + "output/filterByColor.jpg", colorFilteredImg);
	*/

	//put in this->points detected blobs that satisfy this->params tolerance
	begin = chrono::high_resolution_clock::now();
	int blobNumber = findBlobs(image, downscalingFactor);
	end = chrono::high_resolution_clock::now();
	cout << "\nFound " << blobNumber << " blobs: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;
	/*
	namedWindow("Blobs found", WINDOW_NORMAL);
	imshow("Blobs found", colorFilteredImg);
	waitKey(1);
	imwrite(workingDir + "output/findBlobs.jpg",colorFilteredImg);
	*/

	if(blobNumber == 5) {
		//order this->points accordingly to the led pattern numbering
		begin = chrono::high_resolution_clock::now();
		patternAnalysis.evaluate(ledPoints, image, 10);
		end = chrono::high_resolution_clock::now();
		cout << "\nPattern: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;
		//imwrite(workingDir + "output/patternMirko.jpg",colorFilteredImg);

		GenPurpFunc::pointVectorToStrng(ledPoints);

		Interval<int> hue(0,255);
		Interval<int> sat(0,255);
		Interval<int> val(0,255);

		int ledPointsLength = ledPoints.size();
		for (int i = 0; i < ledPointsLength; i++) {
			Point2f p = ledPoints[i];
			Vec3b color = hsvImg.at<Vec3b>(p);
			if (color[0] > hue.high) hue.high = color[0];
			if (color[1] > sat.high) sat.high = color[1];
			if (color[2] > val.high) val.high = color[2];
			if (color[0] < hue.low)	 hue.low  = color[0];
			if (color[1] < sat.low)	 sat.low  = color[1];
			if (color[2] < val.low)	 val.low  = color[2];
		}
		colorInterval.low  = Scalar(hue.low  - colorTolerance, sat.low  - colorTolerance, val.low  - colorTolerance);
		colorInterval.high = Scalar(hue.high + colorTolerance, sat.high + colorTolerance, val.high + colorTolerance);

		findROI();

		points = ledPoints;
	}

	int averageSize = (oldKeyPointSizeInterval.low + oldKeyPointSizeInterval.high)/2;

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


