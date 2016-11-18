/*
 * ImageLoader.cpp
 *
 *  Created on: Sep 7, 2016
 *      Author: root
 */

#include "ImgLoader.hpp"

using namespace std;
using namespace cv;

ImgLoader::ImgLoader() {}

ImgLoader::ImgLoader(const string &source) {
	capture = VideoCapture(source);
}

ImgLoader::ImgLoader(int device) {
	capture = VideoCapture(device);
}

ImgLoader::~ImgLoader() {}

bool ImgLoader::halveRes() {
	return setFrameHeight(getFrameHeight() / 2) && setFrameWidth(getFrameWidth() / 2);
}

bool ImgLoader::doubleRes() {
	return setFrameHeight(getFrameHeight() * 2) && setFrameWidth(getFrameWidth() * 2);
}

bool ImgLoader::isOpen() { return capture.isOpened(); }
