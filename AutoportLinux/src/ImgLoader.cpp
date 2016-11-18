/*
 * ImageLoader.cpp
 *
 *  Created on: Sep 7, 2016
 *      Author: root
 */

#include "ImgLoader.hpp"

using namespace std;
using namespace cv;

ImgLoader::ImgLoader(const string &source, SourceType type, const Size &frameSize, int fps) {
	if(cleverConstr(source,type,frameSize,fps))
		cerr << "Error setting capture parameters!" << endl;

}

ImgLoader::ImgLoader(const string &source, SourceType type) {

	if(!cleverConstr(source, type))
		cerr << "Error setting capture parameters!" << endl;
}

ImgLoader::ImgLoader() {}


bool ImgLoader::constructor(const string &source, SourceType type) {
	switch(type) {
	case sDEVICE:
		capture = VideoCapture(0);
		break;
	case sFILE:
		capture = VideoCapture(source);
		break;
	case sSTREAM:	// WARNING: never tested, may have some bugs...
		capture = VideoCapture(source, CAP_FFMPEG);
		break;
	case sNONE:
		return true;
	}
	sourceType = type;

	if(!capture.isOpened()) {
		opened = false;
		return false;
	}

	int width  = getFrameWidth();
	int height = getFrameHeight();
	res    = Size(width,height);
	defRes = Size(width,height);
	roi = Rect(0, 0, res.width, res.height);

	opened = true;
	return true;
}

bool ImgLoader::cleverConstr(const string &source, SourceType type, const Size &frameSize, int fps) {

	if(!constructor(source, type)) {
		cerr << "Cannot find input" << endl;
		return false;
	}

	cout << "Found input!" << endl;

	bool success = true;
	if(frameSize.height != 0 && frameSize.width != 0) {
		success = 	 	 	 setFrameHeight(frameSize.height);
		success = success && setFrameWidth (frameSize.width);
	}
	//success = success && capture.set(CAP_PROP_FPS, fps);
	//success = success && capture.set(CAP_PROP_BRIGHTNESS, 0.0001);

	// if an error occurs fall back to default capture parameters
	if(!success) {
		capture.release();
		return constructor(source, type);
	}
	return success;
}


bool ImgLoader::getNextFrame(Mat &frame) {
	capture >> frame;
	if(frame.empty())
		return false;

	if(sourceType == sFILE) {
		//if(resizeDynamically)
		//	resize(frame,frame,res,0,0,INTER_LANCZOS4);
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

Rect ImgLoader::getROI() { return roi; }

SourceType ImgLoader::getSourceType() {
	return sourceType;
}

Mat ImgLoader::getResampleMat() {
	Mat resampleMat = Mat::zeros(2,2,CV_32FC1);
	resampleMat.at<float>(0,0) = 2592/res.width;	// TODO
	resampleMat.at<float>(1,1) = 1944/res.height;	// TODO

	return resampleMat;
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

bool ImgLoader::isOpen() { return capture.isOpened() && opened; }

