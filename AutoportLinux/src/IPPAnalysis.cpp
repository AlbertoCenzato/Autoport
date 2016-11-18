/*
 * IPPAnalysis.cpp
 *
 *  Created on: Oct 12, 2016
 *      Author: alberto
 */

#include <chrono>
#include "IPPAnalysis.hpp"
#include "GenPurpFunc.hpp"
#include "ImgLoader.hpp"

extern Status status;
extern ofstream ledStream;
extern ofstream times;

IPPAnalysis::IPPAnalysis(ImgLoader* loader) {
	this->loader = loader;
	Settings *settings = Settings::getInstance();
	imageAnalyzer 	   = ImgAnalysis();
	patternAnalyzer    = PatternAnalysis();
	positionEstimator  = PositionEstimation();

	ROITol     = settings->ROITolerance;
	sizeInfTol = 4;							//FIXME: add this to Settings
	sizeSupTol = settings->sizeSupTolerance;
	sizeTol    = settings->sizeTolerance;
	colorTol   = settings->colorTolerance;
}

IPPAnalysis::~IPPAnalysis() {}


Result IPPAnalysis::evaluate(Mat& extrinsicFactors) {

	cout << "\n-----------------------------------------\n" << endl;

	// retieve image from camera or video
	Mat image;
	bool retrieved = loader->getNextFrame(image);
	if(!retrieved) {
		cerr << "Couldn't retrieve frame!" << endl;
		return Result::END;
	}

	Point2f t;
	loader->getCropVector(t);
	Mat resampleMat = loader->getResampleMat();

	vector<LedDescriptor> points(10);

	namedWindow("Original image", WINDOW_NORMAL);
	imshow("Original image", image);

	auto begin = chrono::high_resolution_clock::now();
	bool success = imageAnalyzer.evaluate(image, points);
	auto end = chrono::high_resolution_clock::now();
	times << chrono::duration_cast<chrono::milliseconds>(end-begin).count();

	// find leds
	if(!success) {
		cerr << "ImageAnalysis failed!" << endl;
		for(int i = 0; i < 2; ++i) {
			for(int j = 0; j < 5; ++j) {
				ledStream << 0 << " ";
			}
			ledStream << endl;
		}
		return Result::FAILURE;
	}

	Scalar red(0,0,255);
	drawDetectedPoints(image, points,red);
	imshow("Original image", image);

	// register leds to match pattern
	success = patternAnalyzer.evaluate(points);

	if(!success) {
		if(status == Status::FIRST_LANDING_PHASE || status == Status::SECOND_LANDING_PHASE) {
			cout << "Target lost!" << endl;
			status = Status::LOOKING_FOR_TARGET;
		}
		cerr << "PatternAnalysis failed!" << endl;
		for(int i = 0; i < 2; ++i) {
			for(int j = 0; j < 5; ++j) {
				ledStream << 0 << " ";
			}
			ledStream << endl;
		}
		return Result::FAILURE;
	}
	//status = Status::FIRST_LANDING_PHASE;
	cout << "PatternAnalysis succeded!" << endl;

	Mat ledPositions = Mat::zeros(2,5,CV_32FC1);
	for(uint i = 0; i < points.size(); ++i) {
		ledPositions.at<float>(0,i) = points[i].position.x;
		ledPositions.at<float>(1,i) = points[i].position.y;
	}

	for(int i = 0; i < 2; ++i) {
		for(int j = 0; j < 5; ++j) {
			ledStream << ledPositions.at<float>(i,j) << " ";
		}
		ledStream << endl;
	}


	Scalar green(0,255,0);
	GenPurpFunc::numberDetectedPoints(image, points,green);
	imshow("Original image", image);

	updateROI(points);
	updateImgRes(points);
	updateColor(points);

	convertPointsToCamera(points, t, resampleMat);

	//estimate position
	begin = chrono::high_resolution_clock::now();
	success = positionEstimator.evaluate(points, extrinsicFactors);
	end = chrono::high_resolution_clock::now();
	times << " " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << endl;

	if(!success) return Result::FAILURE;

	return Result::SUCCESS;
}

bool IPPAnalysis::reset() {
	return resetROIandRes() && resetColor() && positionEstimator.resetInitialPosition();
}

// --- private members ---

bool IPPAnalysis::updateROI(const vector<LedDescriptor>& descriptors) {
	const int SIZE = descriptors.size();
	vector<Point2f> points(SIZE);
	for(int i = 0; i < SIZE; ++i)
		points[i] = descriptors[i].position;
	Rect boundBox = boundingRect(points);
	int x = boundBox.x - ROITol;
	int y = boundBox.y - ROITol;
	int roiWidth  = boundBox.width  + 2*ROITol;
	int roiHeight = boundBox.height + 2*ROITol;

	return loader->setROI(Rect(x, y, roiWidth, roiHeight));
}

bool IPPAnalysis::updateImgRes(const vector<LedDescriptor> &descriptors) {
	const int SIZE = descriptors.size();
	float meanSize = 0;
	for(int i = 0; i < SIZE; ++i)
		meanSize += descriptors[i].size;
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

bool IPPAnalysis::updateColor(const vector<LedDescriptor> &descriptors) {
	const int SIZE = descriptors.size();
	float sum[] = {0,0,0};
	for(int i = 0; i < SIZE; ++i) {
		sum[0] += descriptors[i].color[0];
		sum[1] += descriptors[i].color[1];
		sum[2] += descriptors[i].color[2];
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

		cout << "AVG color channel " << i << ": " << avrg << endl;
	}

	imageAnalyzer.setColorInterval(Interval<Scalar>(minCol,maxCol));

	return true;
}

bool IPPAnalysis::resetROIandRes() {
	bool success = loader->resetRes();
	if(!success)
		return false;
	loader->resetROI();
	return true;
}

bool IPPAnalysis::resetColor() {
	imageAnalyzer.resetColorInterval();
	return true;
}

void IPPAnalysis::convertPointsToCamera(vector<LedDescriptor> &points, Point2f &t, Mat &resampleMat) {
	const int SIZE = points.size();
	for(int i = 0; i < SIZE; ++i) {
		if(!points[i].isEmpty()) {
			Point2f trasl = points[i].position + t;
			points[i].position = Point2f(trasl.x*resampleMat.at<float>(0,0),
					trasl.y*resampleMat.at<float>(1,1));
		}
	}
}


