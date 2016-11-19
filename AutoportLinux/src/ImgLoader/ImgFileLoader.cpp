/*
 * ImgFileLoader.cpp
 *
 *  Created on: Nov 18, 2016
 *      Author: alberto
 */

#include "ImgFileLoader.hpp"

ImgFileLoader::ImgFileLoader() : ImgLoader() { }

ImgFileLoader::ImgFileLoader(const string &source, bool resizeDynamically, const Size &frameSize) : ImgLoader(source) {
	if(!capture.isOpened()) {
		cerr << "Error opening image stream" << endl;
		return;
	}

	cout << "Found input!" << endl;

	resampleMat = Mat::zeros(2,2,CV_32FC1);
	this->resizeDynamically = resizeDynamically;

	int width  = capture.get(CV_CAP_PROP_FRAME_WIDTH);
	int height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
	if(frameSize.height != 0 && frameSize.width != 0) {
		height = frameSize.height;
		width  = frameSize.width;
	}

	defRes = Size(width,height);
	setResolutionWidth (width);
	setResolutionHeight(height);

	roi = Rect(0, 0, res.width, res.height);
}

ImgFileLoader::~ImgFileLoader() { }

bool ImgFileLoader::getNextFrame(Mat &frame) {
	capture >> frame;
	if(frame.empty())
		return false;

	if(resizeDynamically) {
		resize(frame,frame,res,0,0,INTER_LANCZOS4);
	}
	frame = frame(roi);

	return true;
}

Rect ImgFileLoader::getROI() {
	if(roi.width == 0 || roi.height == 0) {
		roi.x = 0;
		roi.y = 0;
		roi.width  = res.width;
		roi.height = res.height;
	}
	return roi;
}

Mat ImgFileLoader::getResampleMat() {
	if(resampleMat.empty()) {
		resampleMat = Mat::zeros(2,2,CV_32FC1);
		resampleMat.at<float>(0,0) = 2592/res.width;	// TODO: use Settings to store "magic numbers"
		resampleMat.at<float>(1,1) = 1944/res.height;	// TODO: use Settings to store "magic numbers"
	}
	return resampleMat;
}

void ImgFileLoader::getCropVector(Point2f &t) {
	t.x = roi.x;
	t.y = roi.y;
}

bool ImgFileLoader::setResolutionWidth (int frameWidth)  {
	res.width = frameWidth;
	resampleMat.at<float>(0,0) = 2592/frameWidth;
	return true;
}

bool ImgFileLoader::setResolutionHeight(int frameHeight) {
	res.height = frameHeight;
	resampleMat.at<float>(1,1) = 1944/frameHeight;
	return true;
}

bool ImgFileLoader::setROI(const Rect& roi) {

	int x = roi.x + this->roi.x;
	int y = roi.y + this->roi.y;
	int roiWidth  = roi.width;
	int roiHeight = roi.height;

	if(x < 0) x = 0;
	if(y < 0) y = 0;
	if(roiWidth  + x > res.width)  roiWidth  = res.width  - x;
	if(roiHeight + y > res.height) roiHeight = res.height - y;
	this->roi = Rect(x, y, roiWidth, roiHeight);

	return true;
}

bool ImgFileLoader::resetRes() {
	setResolutionWidth( defRes.width);
	setResolutionHeight(defRes.height);
	return true;
}

void ImgFileLoader::resetROI() {
	roi = Rect(0, 0, res.width, res.height);
}


