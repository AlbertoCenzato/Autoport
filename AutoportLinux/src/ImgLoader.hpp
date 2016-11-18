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
	Rect getROI();
	SourceType getSourceType();
	Mat getResampleMat();
	void getCropVector(Point2f &t);

	bool isOpen();

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

	bool constructor(const string &source, SourceType type);
	bool cleverConstr(const string &source, SourceType type, const Size &frameSize = Size(0,0), int fps = 30);
};



#endif /* IMGLOADER_HPP_ */
