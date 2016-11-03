/*
 * ImageLoader.cpp
 *
 *  Created on: Sep 7, 2016
 *      Author: root
 */

#include "ImgLoader.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

using namespace std;
using namespace cv;

ImgLoader::ImgLoader(const string &source, SourceType type, const Size &frameSize, int fps) {
	if(cleverConstr(source,type,frameSize,fps))
		cerr << "Error setting capture parameters!" << endl;

}

ImgLoader::ImgLoader(const string &source, SourceType type) {

	if(cleverConstr(source, type))
		cerr << "Error setting capture parameters!" << endl;
}

ImgLoader::ImgLoader() {}


bool ImgLoader::getNextFrame(Mat &frame) {
	capture >> frame;
	if(frame.empty())
		return false;

	if(sourceType == sFILE) {
		if(resizeDynamically)
			resize(frame,frame,res,0,0,INTER_LANCZOS4);
		frame = frame(roi);
	}

	return true;
}

int  ImgLoader::getFrameWidth () {
	if(sourceType == sFILE && roi.width != 0 && roi.height != 0)
		return roi.width;
	return capture.get(CV_CAP_PROP_FRAME_WIDTH );
}
int  ImgLoader::getFrameHeight() {
	if(sourceType == sFILE && roi.width != 0 && roi.height != 0)
			return roi.height;
	return capture.get(CV_CAP_PROP_FRAME_HEIGHT);
}

SourceType ImgLoader::getSourceType() {
	return sourceType;
}

void ImgLoader::getResampleMat(Mat &resampleMat) {
	resampleMat = Mat::zeros(2,2,CV_32FC1);
	resampleMat.at<float>(0,0) = 2592/res.width;	// TODO
	resampleMat.at<float>(1,1) = 1944/res.height;	// TODO
}

void ImgLoader::getCropVector(Point2f &t) {
	t.x = roi.x;
	t.y = roi.y;
}

bool ImgLoader::setFrameWidth (int frameWidth)  {
	bool success = false;
	if(sourceType == sFILE) {
		resizeDynamically = true;
		success = true;
	}
	else {
		res.width = frameWidth;
		success = capture.set(CV_CAP_PROP_FRAME_WIDTH,  frameWidth);
	}
	if(success) {
		res.width = frameWidth;
	}
	return success;
}

bool ImgLoader::setFrameHeight(int frameHeight) {
	bool success = false;
	if(sourceType == sFILE) {
		resizeDynamically = true;
		success = true;
	}
	else {
		success = capture.set(CV_CAP_PROP_FRAME_HEIGHT, frameHeight);
	}
	if(success) {
		res.height = frameHeight;
	}
	return success;
}

bool ImgLoader::setROI(const Rect& roi) {

	int x = roi.x + this->roi.x;
	int y = roi.y + this->roi.y;
	int roiWidth  = roi.width;
	int roiHeight = roi.height;

	if(x < 0) x = 0;
	if(y < 0) y = 0;
	if(roiWidth  + x > res.width)  roiWidth  = res.width  - x;
	if(roiHeight + y > res.height) roiHeight = res.height - y;
	this->roi = Rect(x, y, roiWidth, roiHeight);

	bool success = true;
	if(sourceType == SourceType::sDEVICE) {
		// TODO: add the code for change device resolution here
		success = false;
	}
	return success;
}

bool ImgLoader::resetRes() {
	bool success = setFrameWidth( defRes.width);
	success = success && setFrameHeight(defRes.height);
	resizeDynamically = !success;
	return success;
}

void ImgLoader::resetROI() {
	roi = Rect(0, 0, res.width, res.height);
}

bool ImgLoader::halveRes() {
	return setFrameHeight(res.height / 2) && setFrameWidth(res.width / 2);
}

bool ImgLoader::doubleRes() {
	return setFrameHeight(res.height * 2) && setFrameWidth(res.width * 2);
}


