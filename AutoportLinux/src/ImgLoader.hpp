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

	int  getFrameWidth ();
	int  getFrameHeight();
	Rect getROI() { return roi; }

	void clearROI();

	bool isOpen() { return capture.isOpened() && opened; }

	bool setFrameWidth (int frameWidth);
	bool setFrameHeight(int frameHeight);
	bool setROI(const Rect& roi);

private:

	VideoCapture capture;
	int fps = 30;
	int sourceType;
	bool opened = false;
	Rect roi;				// region of interest
	Size res;				// resolution

	bool constructor(const std::string &source, int type) {
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

		res = Size(getFrameWidth(),getFrameHeight());
		roi = Rect(0, 0, res.width, res.height);

		opened = true;
		return true;
	}

	bool cleverConstr(const std::string &source, int type, const Size &frameSize, int fps) {

		if(!constructor(source, type))
			return false;

		cout << "Found input, " << fps << "fps" << endl;

		bool success = 	 	 setFrameHeight(frameSize.height);
		success = success && setFrameWidth(frameSize.width);
		success = success && capture.set(CAP_PROP_FPS, fps);
		success = success && capture.set(CAP_PROP_BRIGHTNESS, 0.0001);

		// if an error occurs fall back to default camera parameters
		if(!success) {
			capture.release();
			return constructor(source, type);
		}
		return success;
	}
};



#endif /* IMGLOADER_HPP_ */
