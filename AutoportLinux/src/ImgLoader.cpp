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
	return !frame.empty();
}

