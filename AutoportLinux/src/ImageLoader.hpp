/*
 * ImageLoader.hpp
 *
 *  Created on: Sep 7, 2016
 *      Author: root
 */

#ifndef IMAGELOADER_HPP_
#define IMAGELOADER_HPP_


#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

using namespace cv;

class ImageLoader {
public:

	static const int FILE   = 0;
	static const int DEVICE = 1;
	static const int STREAM = 2;

	VideoCapture capture;

	ImageLoader(std::string source, int type);

	void getNextFrame(Mat &frame);


};



#endif /* IMAGELOADER_HPP_ */
