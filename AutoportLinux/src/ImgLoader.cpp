/*
 * ImageLoader.cpp
 *
 *  Created on: Sep 7, 2016
 *      Author: root
 */

#include "ImgLoader.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>

using namespace std;
using namespace cv;

extern string workingDir;

ImgLoader::ImgLoader(const string &source, int type, const Size &frameSize, int fps) {

	switch(type) {
		case DEVICE:
			capture = VideoCapture(1);
			break;
		case FILE:
			capture = VideoCapture(source);
			break;
		case STREAM:
			capture = VideoCapture(source, CAP_FFMPEG);
			break;
	}

	if(!capture.isOpened()) {
		cout << "Cannot find input" << endl;
		return;
	}

	setFrameHeight(frameSize.height);
	setFrameWidth(frameSize.width);
	capture.set(CAP_PROP_FPS, fps);
	capture.set(CAP_PROP_BRIGHTNESS, 0.0001);

	cout << "Found camera input, " << fps << "fps" << endl;
}

ImgLoader::ImgLoader(const string &source, int type) {

	switch(type) {
		case DEVICE:
			capture = VideoCapture(0);
			break;
		case FILE:
			capture = VideoCapture(source);
			break;
		case STREAM:
			capture = VideoCapture(source, CAP_FFMPEG);
			break;
	}

	if(!capture.isOpened()) {
		string imagesPath = workingDir + "Images/";
		cout << "Cannot find input" << endl;
		return;
	}

	capture.set(CV_CAP_PROP_FRAME_WIDTH , 640);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT , 480);
	fps = capture.get(CAP_PROP_FPS);

	cout << "Found input" << endl;
}

bool ImgLoader::getNextFrame(Mat &frame) {
	cout << "FPS: " << capture.get(CAP_PROP_FPS);
	capture >> frame;
	return !frame.empty();
}

