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

	VideoCapture capture;

	ImgLoader(std::string source, int type);

	void getNextFrame(Mat &frame);


};



#endif /* IMGLOADER_HPP_ */
