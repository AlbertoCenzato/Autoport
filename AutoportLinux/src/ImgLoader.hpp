/*
 * ImageLoader.hpp
 *
 *  Created on: Sep 7, 2016
 *      Author: root
 */

#ifndef IMGLOADER_HPP_
#define IMGLOADER_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

using namespace cv;

class ImgLoader {
public:

	static const int FILE   = 0;
	static const int DEVICE = 1;
	static const int STREAM = 2;

	ImgLoader(const std::string &source, int type, const Size &frameSize, int fps);	//FIXME setting frameSize throws an exception when reding frame
	ImgLoader(const std::string &source, int type);

	bool getNextFrame(Mat &frame);

	int getFrameWidth()  { return capture.get(CV_CAP_PROP_FRAME_WIDTH ); }
	int getFrameHeight() { return capture.get(CV_CAP_PROP_FRAME_HEIGHT); }
	bool setFrameWidth (int frameWidth)  { return capture.set(CV_CAP_PROP_FRAME_WIDTH,  frameWidth); }
	bool setFrameHeight(int frameHeight) { return capture.set(CV_CAP_PROP_FRAME_HEIGHT, frameHeight); }

private:

	VideoCapture capture;
	int fps = 30;
};



#endif /* IMGLOADER_HPP_ */
