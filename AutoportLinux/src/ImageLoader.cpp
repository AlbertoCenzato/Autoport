/*
 * ImageLoader.cpp
 *
 *  Created on: Sep 7, 2016
 *      Author: root
 */

#include "ImageLoader.hpp"

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

extern string workingDir;

ImageLoader::ImageLoader() {

	capture = VideoCapture(0);
	if(!capture.isOpened()) {

		string imagesPath = workingDir + "Images/";
		cout << "Could not find a camera input\n" <<
				"Trying to load images from: " << imagesPath << endl;
	}
}

ImageLoader::~ImageLoader() {
	// TODO Auto-generated destructor stub
}


void ImageLoader::getNextFrame(cv::Mat &frame) {
	capture >> frame;
}
