/*
 * IPPAnalysis.cpp
 *
 *  Created on: Oct 12, 2016
 *      Author: alberto
 */

#include "IPPAnalysis.hpp"

extern Status status;

IPPAnalysis::IPPAnalysis(ImgLoader* loader) {
	this->loader = loader;
	Settings& settings =  Settings::getInstance();
	imageAnalyzer 	  = ImgAnalysis();
	patternAnalyzer   = PatternAnalysis();
	positionEstimator = PositionEstimation();

	ROITol     = settings.ROITolerance;
	sizeInfTol = 4;							//FIXME: add this to Settings
	sizeSupTol = settings.sizeSupTolerance;
	sizeTol    = settings.sizeTolerance;
	colorTol   = settings.colorTolerance;
}

IPPAnalysis::~IPPAnalysis() {}


bool IPPAnalysis::evaluate(Mat& extrinsicFactors) {

	cout << "\n-----------------------------------------\n" << endl;

	// retieve image from camera or video
	Mat image;
	bool retrieved = loader->getNextFrame(image);
	if(!retrieved) {
		cerr << "Couldn't retrieve frame!" << endl;
		return false;
	}

	Mat resampleMat;
	Point2f t;
	loader->getResampleMat(resampleMat);
	loader->getCropVector (t);

	vector<LedDescriptor> points(10);

	// find leds
	bool success = imageAnalyzer.evaluate(image, points, 1);
	if(!success) {
		cerr << "ImageAnalysis failed!" << endl;
		return false;
	}

	// register leds to match pattern
	success = patternAnalyzer.evaluate(points);
	if(!success) {
		if(status == Status::FIRST_LANDING_PHASE || status == Status::SECOND_LANDING_PHASE) {
			cout << "Target lost!" << endl;
			status = Status::LOOKING_FOR_TARGET;
		}
		cerr << "PatternAnalysis failed!" << endl;
		return false;
	}
	status = Status::FIRST_LANDING_PHASE;
	cout << "PatternAnalysis succeded!" << endl;

	Scalar blue(255,0,0);
	for(uint i = 0; i < points.size(); ++i) {
		string number = to_string(i);
		if(!points[i].isEmpty()) {
			circle(image, points[i].getPosition(), 30, blue, 10);
			putText(image, number, points[i].getPosition(), HersheyFonts::FONT_HERSHEY_PLAIN,
					2,blue,10,8);
		}
	}

	if(success) {
		updateROI(points);
		updateImgRes(points);

		//TODO: find an adequate tolerance for each color channel
		//updateColor(points);
	}

	Eigen::Matrix<double,3,2> evaluatedPosition;
	vector<Point2f> positions(points.size());
	for(uint i = 0; i < points.size(); ++i)
		positions[i] = points[i].getPosition();

	cout << "Frame points:\n" << GenPurpFunc::pointVectorToStrng(positions) << endl;
	convertPointsToCamera(positions, t, resampleMat);
	cout << "Camera points:\n" << GenPurpFunc::pointVectorToStrng(positions) << endl;
	success = positionEstimator.evaluate(positions, evaluatedPosition);
	cout << "Position:\n" << evaluatedPosition << endl;
	if(!success) return false;

	namedWindow("Original image", WINDOW_NORMAL);
	imshow("Original image", image);
	waitKey(0);

	return true;
}

bool IPPAnalysis::reset() {
	return resetROIandRes() && resetColor();
}
