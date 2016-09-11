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

ImageLoader::ImageLoader() {

	capture = VideoCapture(0);
	if(!capture.isOpened()) {
		string imagesPath = workingDir + "Images/";
		cout << "Cannot find a camera input, trying to load images from: " << imagesPath << endl;

		// load images...

	}
	else {
		cout << "Found camera input" << endl;
	}
}

void ImageLoader::getNextFrame(Mat &frame) {
	capture >> frame;
}

