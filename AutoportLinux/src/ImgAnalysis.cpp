//Copyright (c) 2016 Alberto Cenzato. All rights reserved.

#include "ImgAnalysis.hpp"
#include <chrono>

using namespace cv;
using namespace std;


bool ImgAnalysis::evaluate(Mat &image, vector<Point2f> &points, float downscalingFactor) {

	Mat hsvImg(image.rows,image.cols,image.depth());
	Mat colorFilteredImg(image.rows,image.cols,image.depth());

	//change color space: from BGR to HSV;
    //TODO: color conversion and filterByColor can be performed with a shader (?)
	auto begin = chrono::high_resolution_clock::now();
	cvtColor(image,hsvImg,colorConversion);
	auto end = chrono::high_resolution_clock::now();
	cout << "\nConvert color: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;

	//filter the color according to this->low and this->high tolerances
	begin = chrono::high_resolution_clock::now();
	filterByColor(hsvImg,colorFilteredImg);
	end = chrono::high_resolution_clock::now();
	cout << "\nFilter color: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;

	//put in this->points detected blobs that satisfy this->params tolerance
	begin = chrono::high_resolution_clock::now();
	int blobNumber = findBlobs(colorFilteredImg, image, points, downscalingFactor);
	end = chrono::high_resolution_clock::now();
	cout << "\nFound " << blobNumber << " blobs: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;

	namedWindow("Blobs found", WINDOW_NORMAL);
	imshow("Blobs found", colorFilteredImg);
	waitKey(1);

	return true;
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
ImgAnalysis* ImgAnalysis::setColorInterval(Interval<Scalar> &colorInterval) {
	this->colorInterval.high[0] = colorInterval.high[0];
	this->colorInterval.high[1] = colorInterval.high[1];
	this->colorInterval.high[2] = colorInterval.high[2];

	this->colorInterval.low[0] = colorInterval.low[0];
	this->colorInterval.low[1] = colorInterval.low[1];
	this->colorInterval.low[2] = colorInterval.low[2];

	return this;
}

void ImgAnalysis::getColorInterval(Interval<Scalar> &colorInterval) {
	colorInterval.high[0] = this->colorInterval.high[0];
	colorInterval.high[1] = this->colorInterval.high[1];
	colorInterval.high[2] = this->colorInterval.high[2];

	colorInterval.low[0] = this->colorInterval.low[0];
	colorInterval.low[1] = this->colorInterval.low[1];
	colorInterval.low[2] = this->colorInterval.low[2];

}

