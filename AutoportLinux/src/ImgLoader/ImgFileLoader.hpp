/*
 * ImgFileLoader.h
 *
 *  Created on: Nov 18, 2016
 *      Author: alberto
 */

#ifndef IMGFILELOADER_HPP_
#define IMGFILELOADER_HPP_

#include "ImgLoader.hpp"

class ImgFileLoader: public ImgLoader {
public:

	ImgFileLoader();
	ImgFileLoader(const string &source, bool resizeDynamically = true, const Size &frameSize = Size(0,0));

	~ImgFileLoader();

	bool getNextFrame(Mat &frame);

	Rect getROI();
	Mat  getResampleMat();
	void getCropVector(Point2f &t);

	bool setResolutionWidth (int frameWidth);
	bool setResolutionHeight(int frameHeight);
	bool setROI(const Rect& roi);

	bool resetRes();
	void resetROI();

private:

	Rect roi;				// region of interest
	Size res;				// resolution
	Size defRes;			// default resolution
	bool resizeDynamically = false;
	Mat resampleMat;

};

#endif /* IMGFILELOADER_HPP_ */
