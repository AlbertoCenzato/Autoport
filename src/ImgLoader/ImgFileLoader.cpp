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

#include "ImgFileLoader.hpp"

using namespace std;
using namespace cv;

ImgFileLoader::ImgFileLoader() : ImgLoader() { }

ImgFileLoader::ImgFileLoader(const string &source, bool resizeDyn, const Size &frameSize) : ImgLoader(source) {

	if(!capture.isOpened()) { 							// if can't open the stream...
		cerr << "Error opening image stream" << endl;
		return;											// ... return and do nothing
	}

	cout << "Found input!" << endl;

	resampleMat = Mat::zeros(2,2,CV_32FC1);
	this->resizeDyn = resizeDyn;

	int width  = capture.get(CV_CAP_PROP_FRAME_WIDTH);	// get native images resolution
	int height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);

	if(frameSize.height != 0 && frameSize.width != 0) { // reset resolution to frameSize
		height = frameSize.height;
		width  = frameSize.width;
	}

	defRes = Size(width,height);	// save default resolution
	setResolutionWidth (width);		// and set stream resolution to default resolution
	setResolutionHeight(height);

	roi = Rect(0, 0, res.width, res.height);	// roi is the entire image
}

ImgFileLoader::~ImgFileLoader() { }

bool ImgFileLoader::getNextFrame(Mat &frame) {
	capture >> frame;	// reads a frame from file
	if(frame.empty())
		return false;

	if(resizeDyn) {
		// resizes the image to desired resolution. If used for real-time
		// purposes a simpler (and faster) interpolation algorithm can be used
		// instead of Lanczos, for example INTER_NEAREST or INTER_LINEAR
		resize(frame,frame,res,0,0,INTER_LANCZOS4);
	}
	frame = frame(roi); // crops the image according to "roi"

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

void ImgFileLoader::getTranslVector(Point2f &t) {
	t.x = roi.x;
	t.y = roi.y;
}

bool ImgFileLoader::setResolutionWidth (int frameWidth)  {
	res.width = frameWidth;
	resampleMat.at<float>(0,0) = 2592/(float)frameWidth;
	return true;
}

bool ImgFileLoader::setResolutionHeight(int frameHeight) {
	res.height = frameHeight;
	resampleMat.at<float>(1,1) = 1944/(float)frameHeight;
	return true;
}

bool ImgFileLoader::setROI(const Rect& roi) {

	int x = roi.x + this->roi.x;
	int y = roi.y + this->roi.y;
	int roiWidth  = roi.width;
	int roiHeight = roi.height;

	// roi must be non negative and contained in the image resolution
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


