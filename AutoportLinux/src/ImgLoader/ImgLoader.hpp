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

	ImgLoader();
	ImgLoader(const string &source);
	ImgLoader(int device);

	virtual ~ImgLoader();

	virtual bool getNextFrame(Mat &frame) = 0;

	virtual int  getFrameWidth () = 0;
	virtual int  getFrameHeight() = 0;
	virtual Rect getROI() = 0;
	virtual Mat  getResampleMat() = 0;
	virtual void getCropVector(Point2f &t) = 0;

	virtual bool setFrameWidth (int frameWidth)  = 0;
	virtual bool setFrameHeight(int frameHeight) = 0;
	virtual bool setROI(const Rect& roi) = 0;

	virtual bool resetRes() = 0;
	virtual void resetROI() = 0;

	bool isOpen();
	bool halveRes();
	bool doubleRes();

protected:

	VideoCapture capture;

};



#endif /* IMGLOADER_HPP_ */
