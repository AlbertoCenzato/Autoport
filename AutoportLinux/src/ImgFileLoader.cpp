/*
 * ImgFileLoader.cpp
 *
 *  Created on: Nov 18, 2016
 *      Author: alberto
 */

#include "ImgFileLoader.hpp"

ImgFileLoader::ImgFileLoader() : ImgLoader() { }

ImgFileLoader::ImgFileLoader(const string &source, const Size &frameSize)
	: ImgLoader(source) {
	if(cleverConstr(source, frameSize))
		cerr << "Error setting capture parameters!" << endl;
}

bool ImgFileLoader::cleverConstr(const string &source, const Size &frameSize) {

	if(!constructor(source)) {
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
		return constructor(source);
	}
	return success;
}

bool ImgFileLoader::constructor(const string &source) {
	if(!capture.isOpened()) {
		return false;
	}

	int width  = getFrameWidth();
	int height = getFrameHeight();
	res    = Size(width,height);
	defRes = Size(width,height);
	roi = Rect(0, 0, res.width, res.height);

	return true;
}

bool ImgFileLoader::getNextFrame(Mat &frame) {
	capture >> frame;
	if(frame.empty())
		return false;

		//if(resizeDynamically)
		//	resize(frame,frame,res,0,0,INTER_LANCZOS4);
		frame = frame(roi);

	return true;
}

int  ImgFileLoader::getFrameWidth () {
	if(roi.width != 0 && roi.height != 0)
		return roi.width;
	return capture.get(CV_CAP_PROP_FRAME_WIDTH );
}
int  ImgFileLoader::getFrameHeight() {
	if(roi.width != 0 && roi.height != 0)
		return roi.height;
	return capture.get(CV_CAP_PROP_FRAME_HEIGHT);
}

Rect ImgFileLoader::getROI() { return roi; }

Mat ImgFileLoader::getResampleMat() {
	Mat resampleMat = Mat::zeros(2,2,CV_32FC1);
	resampleMat.at<float>(0,0) = 2592/res.width;	// TODO
	resampleMat.at<float>(1,1) = 1944/res.height;	// TODO

	return resampleMat;
}

void ImgFileLoader::getCropVector(Point2f &t) {
	t.x = roi.x;
	t.y = roi.y;
}

bool ImgFileLoader::setFrameWidth (int frameWidth)  {
	resizeDynamically = true;
	res.width = frameWidth;
	return true;
}

bool ImgFileLoader::setFrameHeight(int frameHeight) {
	resizeDynamically = true;
	res.height = frameHeight;
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
	bool success = 		 setFrameWidth( defRes.width);
	success = success && setFrameHeight(defRes.height);
	resizeDynamically = !success;
	return success;
}

void ImgFileLoader::resetROI() {
	roi = Rect(0, 0, res.width, res.height);
}


