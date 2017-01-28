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

#ifndef IMGFILELOADER_HPP_
#define IMGFILELOADER_HPP_

#include "ImgLoader.hpp"

/**
 * Concrete class for ImgLoader abstract class.
 * Provides support for loading image sequences from multiple files.
 */
class ImgFileLoader: public ImgLoader {
public:

	ImgFileLoader();

	/**
	 * Class constructor. Opens a stream for the specified source.
	 * It can read from a video file or image sequence.
	 * If source is an image sequence it must be in the format "img_%02d.jpg",
	 * this will make ImgLoader read samples like img_00.jpg, img_01.jpg, img_02.jpg...
	 *
	 * @source: path to video file or image sequence (as specified above).
	 * @resizeDyn: states if "getNextFrame()" gives in output frames with resolution
	 * 			   "res" or not. If false the original frame resolution is used.
	 * @frameSize: specifies width and height of "res"; if (0,0) keeps image resolution.
	 */
	ImgFileLoader(const std::string &source, bool resizeDyn = true,
			      const cv::Size &frameSize = cv::Size(0,0));

	~ImgFileLoader();

	/**
	 * Reads the next frame from the image sequence.
	 *
	 * @frame: the retrieved frame.
	 * @return: true if the new frame is not empty.
	 */
	bool getNextFrame(cv::Mat &frame);

	/**
	 * @return: "roi"
	 */
	cv::Rect getROI();

	/**
	 * Gives the resample matrix used to transform coordinates from
	 * current ImgFileLoader reference system to camera reference system.
	 * The reasmple matrix is given by the ratio of default camera resolution
	 * and "res".
	 * See IPPAnalysis::convertPointsToCamera() for further details.
	 *
	 * @return: resample matrix
	 */
	cv::Mat  getResampleMat();

	/**
	 * Gives the translation vector used to transform coordinates from
	 * current ImgFileLoader reference system to camera reference system.
	 * See IPPAnalysis::convertPointsToCamera() for further details.
	 *
	 * @t: returned translation vector
	 */
	void getTranslVector(cv::Point2f &t);

	/**
	 * Changes "res" width. Should be used carefully.
	 * Prefer using ImgLoader::halveRes() and
	 * ImgLoader::doubleRes() when possible.
	 *
	 * @frameWidth: new frame width.
	 * @return: always true
	 */
	bool setResolutionWidth (int frameWidth);

	/**
	 * Changes "res" height. Should be used carefully.
	 * Prefer using ImgLoader::halveRes() and
	 * ImgLoader::doubleRes() when possible.
	 *
	 * @frameHeight: new frame height.
	 * @return: always true
	 */
	bool setResolutionHeight(int frameHeight);

	/**
	 * Sets the region of interest of the image that
	 * getNextFrame() retrieves.
	 *
	 * @roi: new region of interest
	 * @return: always true
	 */
	bool setROI(const cv::Rect &roi);

	/**
	 * Sets "res" equal to "defRes", the default resolution
	 * of the opened image stream.
	 *
	 * @return: always true
	 */
	bool resetRes();

	/**
	 * Sets "roi" equal to the maximum possible area.
	 * getNextFrame will then return a full non cropped image
	 */
	void resetROI();

private:

	cv::Rect roi;				// region of interest
	cv::Size res;				// resolution
	cv::Size defRes;			// default resolution
	bool 	 resizeDyn = false;
	cv::Mat  resampleMat;

};

#endif /* IMGFILELOADER_HPP_ */
