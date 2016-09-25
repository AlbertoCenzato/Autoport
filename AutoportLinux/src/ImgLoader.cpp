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

ImgLoader::ImgLoader(string source, int type) {

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

void ImgLoader::getNextFrame(Mat &frame) {
	capture >> frame;
}

