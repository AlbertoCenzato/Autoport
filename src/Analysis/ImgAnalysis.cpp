/*==============================================================================
Software for Autoport project

// Copyright   : Copyright (c) 2016, Alberto Cenzato
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the FreeBSD Project.
//============================================================================ */

#include <chrono>
#include "ImgAnalysis.hpp"
#include "../Utils/Settings.hpp"
#include "../Utils/LedDescriptor.hpp"

using namespace std;
using namespace cv;

ImgAnalysis::ImgAnalysis() {

	// loads default colorInterval values
	Settings *settings = Settings::getInstance();
	Scalar low  = Scalar(settings->hue.min, settings->saturation.min, settings->value.min);
	Scalar high = Scalar(settings->hue.max, settings->saturation.max, settings->value.max);

	constructor(Interval<Scalar>(low,high), settings->patternColor);
}

ImgAnalysis::ImgAnalysis(const Interval<Scalar> &colorInterval, LedColor ledColor) {
	constructor(colorInterval, ledColor);
}

void ImgAnalysis::constructor(const Interval<Scalar> &colorInterval, LedColor ledColor) {
	this->colorInterval = colorInterval;
	defColorInterval    = colorInterval;

	// If looking of red color the channels are loaded in a different order.
	// This results in a 90 degrees rotation of H channel, shifting red
	// where blue should be, thus using the same colorInterval to filter
	// the image both for red LEDs and for blue LEDs
	if(ledColor == LedColor::RED) colorConversion = COLOR_RGB2HSV;
	else						  colorConversion = COLOR_BGR2HSV;

	params = SimpleBlobDetector::Params();

	params.filterByColor 	   = true;
	params.filterByArea 	   = false;	//TODO: set this to true and find default params
	params.filterByInertia 	   = false;
	params.filterByConvexity   = false;
	params.filterByCircularity = true;
	
	params.blobColor 	  = 255;
	params.minCircularity = 0.5;
	params.maxCircularity = 1;
}

bool ImgAnalysis::evaluate(Mat &image, vector<LedDescriptor> &descriptors) {

	//change color space: from BGR to HSV;
	auto begin = chrono::high_resolution_clock::now();
	cvtColor(image,hsvImage,colorConversion);
	auto end = chrono::high_resolution_clock::now();
	cout << "\nConvert color: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;

	//filter the color according to "colorInterval" tolerance
	begin = chrono::high_resolution_clock::now();
	Mat colorFilteredImg = filterByColor(hsvImage);
	end = chrono::high_resolution_clock::now();
	cout << "\nFilter color: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;

	namedWindow("Filtered image", WINDOW_NORMAL);
	imshow("Filtered image", colorFilteredImg);

	//put in this->points detected blobs that satisfy this->params tolerance
	begin = chrono::high_resolution_clock::now();
	int blobNumber = findBlobs(colorFilteredImg, descriptors);
	end = chrono::high_resolution_clock::now();
	cout << "\nFound " << blobNumber << " blobs: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;

	waitKey(1); // necessary to show the image

	return true;
}

ImgAnalysis* ImgAnalysis::setColorInterval(const Interval<Scalar> &colorInterval) {
	
	// FIXME: these 6 lines are the same as:
	//		  this->colorInterval = colorInterval;
	//		  change them
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
	
	// FIXME: see FIXME in ImgAnalysis::setColorInterval()
	colorInterval.max[0] = this->colorInterval.max[0];
	colorInterval.max[1] = this->colorInterval.max[1];
	colorInterval.max[2] = this->colorInterval.max[2];

	colorInterval.min[0] = this->colorInterval.min[0];
	colorInterval.min[1] = this->colorInterval.min[1];
	colorInterval.min[2] = this->colorInterval.min[2];

}

void ImgAnalysis::resetColorInterval() {
	colorInterval = defColorInterval;
}

// --- private members ---

Mat ImgAnalysis::filterByColor(const Mat &hsvImg) {

	Mat bwImage;
	// Sets to white all colors in the threshold interval [min,max] and to black the others
	inRange(hsvImg, colorInterval.min, colorInterval.max, bwImage);

	//morphological opening (remove small objects from the foreground)
	erode (bwImage, bwImage, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(bwImage, bwImage, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//morphological closing (fill small holes in the foreground)
	dilate(bwImage, bwImage, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode (bwImage, bwImage, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	return bwImage;
}

int ImgAnalysis::findBlobs(const Mat &bwImage, vector<LedDescriptor>& descriptors) {

	descriptors.clear();
	vector<KeyPoint> keyPoints(10);

	Ptr<SimpleBlobDetector> featureDetector = SimpleBlobDetector::create(params);
	featureDetector->detect(bwImage, keyPoints);

	const int SIZE = keyPoints.size();
	cout << "KeyPoints SIZE: " << SIZE << endl;
	if(SIZE > 0) {

		for(int i = 0; i < SIZE; ++i) {
			Point2f position = keyPoints[i].pt;
			Scalar  color 	 = hsvImage.at<Vec3b>(position);
			descriptors.push_back(LedDescriptor(position,color,keyPoints[i].size));
		}

		// Remove points too far from the centroid of the detected points set
		// compute the mean distance from the centroid
		Point2f centr = LedDescriptor::centroid(descriptors);
		float meanDist = 0;
		float *distances = new float[SIZE];	// TODO: try to use another way, dangerous pointer
		for (int i = 0; i < SIZE; i++) {
			float dist = GenPurpFunc::distPoint2Point(centr, descriptors[i].position);
			meanDist += dist;
			distances[i] = dist;
		}
		meanDist = meanDist / SIZE;

		// remove points too far away points
		int size = SIZE;
		for (int i = 0; i < size; ) {
			if (distances[i] > 2 * meanDist) {
				descriptors[i] = descriptors[size - 1];
				distances[i] = distances[size - 1];
				descriptors.pop_back();
			}
			else i++;
			size = descriptors.size();
		}

		delete [] distances;
	}

	return descriptors.size();
}

