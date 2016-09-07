/*
 * ImageLoader.h
 *
 *  Created on: Sep 7, 2016
 *      Author: root
 */

#pragma once

#include <opencv2/opencv.hpp>

class ImageLoader {
public:
	ImageLoader();
	virtual ~ImageLoader();

	void getNextFrame(cv::Mat &frame);

private:
	cv::VideoCapture capture;

};
