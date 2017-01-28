/*==============================================================================
Software for Autoport project

// Copyright   : Copyright (c) 2016, Alberto Cenzato
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the FreeBSD Project.
//============================================================================ */

#ifndef IMGLOADER_HPP_
#define IMGLOADER_HPP_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

/**
 * Abstract image loader class. This class provides a unique
 * interface for all its derived classes
 */
class ImgLoader {
public:

	//----- FIXME: an abstract class with constructors is meaningless, isn't it? -----

	/**
	 * Class default constructor.
	 */
	ImgLoader();

	ImgLoader(const std::string &source);

	/**
	 * Class constructor. Opens a stream from the specified video capture device.
	 * If there is only one video capture device its id is 0.
	 *
	 * @device: id of the opened video capturing device
	 */
	ImgLoader(int device);

	virtual ~ImgLoader();

	virtual bool getNextFrame(cv::Mat &frame) = 0;

	virtual cv::Rect getROI() = 0;
	virtual cv::Mat  getResampleMat() = 0;
	virtual void getTranslVector(cv::Point2f &t) = 0;

	virtual bool setResolutionWidth (int frameWidth)  = 0;
	virtual bool setResolutionHeight(int frameHeight) = 0;
	virtual bool setROI(const cv::Rect& roi) = 0;

	virtual bool resetRes() = 0;
	virtual void resetROI() = 0;

	/**
	 * @return: true if the image stream is open
	 */
	bool isOpen();

	/**
	 * Halves the resolution of frames given in output by ImgLoader
	 *
	 * @return: true if resolution changes after this function call
	 */
	bool halveRes();

	/**
	 * Doubles the resolution of frames given in output by ImgLoader
	 *
	 * @return: true if resolution changes after this function call
	 */
	bool doubleRes();

protected:

	cv::VideoCapture capture;

};



#endif /* IMGLOADER_HPP_ */
