//Copyright (c) 2016 Alberto Cenzato. All rights reserved.

#include "ImgAnalysis.hpp"
#include <chrono>

using namespace cv;
using namespace std;


bool ImgAnalysis::evaluate(Mat &image, vector<LedDescriptor> &points, float downscalingFactor) {


	Mat hsvImg;
	Mat colorFilteredImg;

	namedWindow("Original image", WINDOW_NORMAL);
	imshow("Original image", image);

	//change color space: from BGR to HSV;
    //TODO: color conversion and filterByColor can be performed with a shader (?)
	auto begin = chrono::high_resolution_clock::now();
	cvtColor(image,hsvImg,colorConversion);
	auto end = chrono::high_resolution_clock::now();
	cout << "\nConvert color: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;

	hsvImage = hsvImg;


	//filter the color according to this->low and this->high tolerances
	begin = chrono::high_resolution_clock::now();
	filterByColor(hsvImg,colorFilteredImg);
	end = chrono::high_resolution_clock::now();
	cout << "\nFilter color: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;

	namedWindow("Filtered image", WINDOW_NORMAL);
	imshow("Filtered image", colorFilteredImg);

	//put in this->points detected blobs that satisfy this->params tolerance
	begin = chrono::high_resolution_clock::now();
	int blobNumber = findBlobs(colorFilteredImg, points, downscalingFactor);
	end = chrono::high_resolution_clock::now();
	cout << "\nFound " << blobNumber << " blobs: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;

	waitKey(1);

	return true;
}

ImgAnalysis* ImgAnalysis::setColorInterval(const Interval<Scalar> &colorInterval) {
	this->colorInterval.max[0] = colorInterval.max[0];
	this->colorInterval.max[1] = colorInterval.max[1];
	this->colorInterval.max[2] = colorInterval.max[2];

	this->colorInterval.min[0] = colorInterval.min[0];
	this->colorInterval.min[1] = colorInterval.min[1];
	this->colorInterval.min[2] = colorInterval.min[2];

	return this;
}

ImgAnalysis* ImgAnalysis::setBlobSizeInterval(const Interval<int> &blobSizeInterval) {
	params.minArea = blobSizeInterval.min;
	params.maxArea = blobSizeInterval.max;
	return this;
}

void ImgAnalysis::getColorInterval(Interval<Scalar> &colorInterval) {
	colorInterval.max[0] = this->colorInterval.max[0];
	colorInterval.max[1] = this->colorInterval.max[1];
	colorInterval.max[2] = this->colorInterval.max[2];

	colorInterval.min[0] = this->colorInterval.min[0];
	colorInterval.min[1] = this->colorInterval.min[1];
	colorInterval.min[2] = this->colorInterval.min[2];

}

