/*
 * ImageLoader.hpp
 *
 *  Created on: Sep 7, 2016
 *      Author: root
 */

#ifndef IMGLOADER_HPP_
#define IMGLOADER_HPP_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

using namespace std;
using namespace cv;

class ImgLoader {
public:

	static const int FILE 	= 0;
	static const int DEVICE = 1;
	static const int STREAM	= 2;

	ImgLoader();
	ImgLoader(const std::string &source, int type);
	ImgLoader(const std::string &source, int type, const Size &frameSize, int fps);	//FIXME setting frameSize throws an exception when reding frame

	void begin() { capture.set(CV_CAP_PROP_POS_FRAMES,0); }

	bool getNextFrame(Mat &frame);

	int  getFrameWidth () { return capture.get(CV_CAP_PROP_FRAME_WIDTH ); }
	int  getFrameHeight() { return capture.get(CV_CAP_PROP_FRAME_HEIGHT); }

	bool isOpen() { return capture.isOpened() && opened; }

	bool setFrameWidth (int frameWidth)  { return capture.set(CV_CAP_PROP_FRAME_WIDTH,  frameWidth); }
	bool setFrameHeight(int frameHeight) { return capture.set(CV_CAP_PROP_FRAME_HEIGHT, frameHeight); }

private:

	VideoCapture capture;
	int fps = 30;
	int sourceType;
	bool opened = false;

	bool constructor(const std::string &source, int type, const Size &frameSize, int fps) {
		switch(type) {
		case DEVICE:
			sourceType = DEVICE;
			capture = VideoCapture(0);
			break;
		case FILE:
			sourceType = FILE;
			capture = VideoCapture(source);
			break;
		case STREAM:	// TODO: never tested, may have some bugs...
			sourceType = STREAM;
			capture = VideoCapture(source, CAP_FFMPEG);
			break;
		}

		if(!capture.isOpened()) {
			opened = false;
			cerr << "Cannot find input" << endl;
			return false;
		}

		opened = true;
		return true;
	}

	bool cleverConstr(const std::string &source, int type, const Size &frameSize, int fps) {

		if(!constructor(source, type, frameSize, fps))
			return false;

		cout << "Found input, " << fps << "fps" << endl;

		bool error = setFrameHeight(frameSize.height);
		error = error && setFrameWidth(frameSize.width);
		error = error && capture.set(CAP_PROP_FPS, fps);
		error = error && capture.set(CAP_PROP_BRIGHTNESS, 0.0001);

		// if an error occurs fall back to default camera parameters
		if(error) {
			capture.release();
			return constructor(source, type, frameSize, fps);
		}
		return error;
	}
};



#endif /* IMGLOADER_HPP_ */
