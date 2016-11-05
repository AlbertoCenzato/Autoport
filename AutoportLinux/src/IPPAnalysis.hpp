/*
 * IPPAnalysis.hpp
 *
 *  Created on: Oct 12, 2016
 *      Author: alberto
 */

#ifndef IPPANALYSIS_HPP_
#define IPPANALYSIS_HPP_

#include "ImgLoader.hpp"
#include "ImgAnalysis.hpp"
#include "PatternAnalysis.hpp"
#include "PositionEstimation.hpp"

using namespace std;
using namespace cv;

class IPPAnalysis {
public:
	IPPAnalysis(ImgLoader* loader);
	virtual ~IPPAnalysis();

	Result evaluate(Mat& extrinsicFactors);
	bool reset();

private:

	ImgLoader* loader;
	ImgAnalysis imageAnalyzer;
	PatternAnalysis patternAnalyzer;
	PositionEstimation positionEstimator;
	int ROITol = 50;
	int sizeTol;
	int sizeSupTol;
	int sizeInfTol;
	int colorTol;

	bool updateROI(const vector<LedDescriptor>& descriptors) {
		const int SIZE = descriptors.size();
		vector<Point2f> points(SIZE);
		for(int i = 0; i < SIZE; ++i)
			points[i] = descriptors[i].getPosition();
		Rect boundBox = boundingRect(points);
		int x = boundBox.x - ROITol;
		int y = boundBox.y - ROITol;
		int roiWidth  = boundBox.width  + 2*ROITol;
		int roiHeight = boundBox.height + 2*ROITol;

		return loader->setROI(Rect(x, y, roiWidth, roiHeight));
	}

	bool updateImgRes(const vector<LedDescriptor> &descriptors) {
		const int SIZE = descriptors.size();
		float meanSize = 0;
		for(int i = 0; i < SIZE; ++i)
			meanSize += descriptors[i].getSize();
		meanSize /= SIZE;

		bool success = true;
		if(meanSize > sizeSupTol) {
			success = loader->halveRes();
			if(success)
				meanSize /= 4;
		}
		else if(meanSize < sizeInfTol) {
			success = loader->doubleRes();
			if(success)
				meanSize *= 4;
		}

		imageAnalyzer.setBlobSizeInterval(Interval<int>(meanSize - sizeTol,meanSize + sizeTol));

		return success;
	}

	bool updateColor(const vector<LedDescriptor> &descriptors) {
		const int SIZE = descriptors.size();
		float sum[] = {0,0,0};
		for(int i = 0; i < SIZE; ++i) {
			Scalar color = descriptors[i].getColor();
			sum[0] += color[0];
			sum[1] += color[1];
			sum[2] += color[2];
		}

		Scalar minCol, maxCol;
		for(int i = 0; i < 3; ++i) {
			float avrg = sum[i]/SIZE;
			int x = avrg - colorTol;
			if(x < 0) x = 0;
			minCol[i] = x;
			x = avrg + colorTol;
			if(x > 255) x = 255;
			maxCol[i] = x;
		}

		imageAnalyzer.setColorInterval(Interval<Scalar>(minCol,maxCol));

		return true;
	}

	bool resetROIandRes() {
		bool success = loader->resetRes();
		if(!success)
			return false;
		loader->resetROI();
		return true;
	}

	bool resetColor() {

		return true;
	}

	void convertPointsToCamera(vector<Point2f> &points, Point2f &t, Mat &resampleMat) {
		const int SIZE = points.size();
		for(int i = 0; i < SIZE; ++i) {
			Point2f trasl = points[i] + t;
			points[i] = Point2f(trasl.x*resampleMat.at<float>(0,0),
								trasl.y*resampleMat.at<float>(1,1));
		}
	}

};

#endif /* IPPANALYSIS_HPP_ */
