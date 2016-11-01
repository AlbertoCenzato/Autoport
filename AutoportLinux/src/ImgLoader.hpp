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

enum SourceType {
	sNONE,
	sFILE,
	sDEVICE,
	sSTREAM
};


class ImgLoader {
public:

	ImgLoader();
	ImgLoader(const string &source, SourceType type);
	ImgLoader(const string &source, SourceType type, const Size &frameSize, int fps);	//FIXME setting frameSize throws an exception when reding frame

	void begin() { capture.set(CV_CAP_PROP_POS_FRAMES,0); }

	bool getNextFrame(Mat &frame);

	int  getFrameWidth ();
	int  getFrameHeight();
	Rect getROI() { return roi; }


	bool isOpen() { return capture.isOpened() && opened; }

	bool setFrameWidth (int frameWidth);
	bool setFrameHeight(int frameHeight);
	bool setROI(const Rect& roi);

	bool resetRes();
	void resetROI();

	bool halveRes();
	bool doubleRes();

private:

	VideoCapture capture;
	int fps = 30;
	SourceType sourceType = sNONE;
	bool opened = false;
	Rect roi;				// region of interest
	Size res;				// resolution
	Size defRes;			// default resolution
	bool resizeDynamically = false;

	bool constructor(const string &source, SourceType type) {
		switch(type) {
		case sDEVICE:
			capture = VideoCapture(0);
			break;
		case sFILE:
			capture = VideoCapture(source);
			break;
		case sSTREAM:	// WARNING: never tested, may have some bugs...
			capture = VideoCapture(source, CAP_FFMPEG);
			break;
		case sNONE:
			return true;
		}
		sourceType = type;

		if(!capture.isOpened()) {
			opened = false;
			return false;
		}

		int width  = getFrameWidth();
		int height = getFrameHeight();
		res    = Size(width,height);
		defRes = Size(width,height);
		roi = Rect(0, 0, res.width, res.height);

		opened = true;
		return true;
	}

	bool cleverConstr(const string &source, SourceType type, const Size &frameSize = Size(0,0), int fps = 30) {

		if(!constructor(source, type)) {
			cerr << "Cannot find input" << endl;
			return false;
		}

		cout << "Found input!" << endl;

		bool success = true;
		if(frameSize.height != 0 && frameSize.width != 0) {
			success = 	 	 	 setFrameHeight(frameSize.height);
			success = success && setFrameWidth (frameSize.width);
		}
		//success = success && capture.set(CAP_PROP_FPS, fps);
		//success = success && capture.set(CAP_PROP_BRIGHTNESS, 0.0001);

		// if an error occurs fall back to default camera parameters
		if(!success) {
			capture.release();
			return constructor(source, type);
		}
		return success;
	}
};



#endif /* IMGLOADER_HPP_ */
