/*
 * ImageLoader.cpp
 *
 *  Created on: Sep 7, 2016
 *      Author: root
 */

#include "ImageLoader.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>

using namespace std;
using namespace cv;

extern string workingDir;

ImageLoader::ImageLoader(string source, int type) {

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
	else {
		cout << "Found camera input" << endl;
	}
}

void ImageLoader::getNextFrame(Mat &frame) {
	capture >> frame;
}

