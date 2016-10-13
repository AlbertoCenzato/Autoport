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

ImgLoader::ImgLoader(const string &source, int type, const Size &frameSize, int fps) {
	if(cleverConstr(source,type,frameSize,fps))
		cerr << "Error setting capture parameters!" << endl;

}

ImgLoader::ImgLoader(const string &source, int type) {

	Size frameSize(640,480);
	if(cleverConstr(source, type, frameSize, capture.get(CAP_PROP_FPS)))
		cerr << "Error setting capture parameters!" << endl;
}

ImgLoader::ImgLoader() : ImgLoader("",DEVICE) {}


bool ImgLoader::getNextFrame(Mat &frame) {
	capture >> frame;
	if(!frame.empty() && sourceType == FILE)
		frame = frame(roi);
	return !frame.empty();
}

int  ImgLoader::getFrameWidth () {
	if(sourceType == FILE && roi.width != 0 && roi.height != 0)
		return roi.width;
	return capture.get(CV_CAP_PROP_FRAME_WIDTH );
}
int  ImgLoader::getFrameHeight() {
	if(sourceType == FILE && roi.width != 0 && roi.height != 0)
			return roi.height;
	return capture.get(CV_CAP_PROP_FRAME_HEIGHT);
}

bool ImgLoader::setFrameWidth (int frameWidth)  {
	bool success = capture.set(CV_CAP_PROP_FRAME_WIDTH,  frameWidth);
	if(success)
		res.width = frameWidth;
	return success;
}

bool ImgLoader::setFrameHeight(int frameHeight) {
	bool success = capture.set(CV_CAP_PROP_FRAME_HEIGHT, frameHeight);
	if(success)
		res.height = frameHeight;
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

	if(sourceType == DEVICE) {
		// TODO: set roi on the device
	}
	return true;
}

void ImgLoader::clearROI() {
	roi = Rect(0, 0, res.width, res.height);
}


