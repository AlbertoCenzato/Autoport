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

	VideoCapture capture;

	ImageLoader();

	void getNextFrame(Mat &frame);

};



#endif /* IMAGELOADER_HPP_ */
