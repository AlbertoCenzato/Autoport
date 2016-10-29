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
	imageAnalyzer 	  = ImgAnalysis();
	patternAnalyzer   = PatternAnalysis();
	positionEstimator = PositionEstimation();
	tol = Settings::getInstance().ROITolerance;
}

IPPAnalysis::~IPPAnalysis() {
	// TODO Auto-generated destructor stub
}


bool IPPAnalysis::evaluate(Mat& extrinsicFactors) {

	cout << "\n-----------------------------------------\n" << endl;

	// retieve image from camera or video
	Mat image;
	bool retrieved = loader->getNextFrame(image);
	if(!retrieved) {
		cerr << "Couldn't retrieve frame!" << endl;
		return false;
	}

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
		circle(image, points[i].getPosition(), 30, blue, 10);
		putText(image, number, points[i].getPosition(), HersheyFonts::FONT_HERSHEY_PLAIN,
				2,blue,10,8);
	}

	Rect roi;
	findROI(points, roi);
	loader->setROI(roi);
	roi = loader->getROI();
	rectangle(image,roi,Scalar(255,255,255));
	cout << roi << endl;
	namedWindow("ROI",WINDOW_NORMAL);
	imshow("ROI",image);
/*
	success		 = positionEstimator.evaluate(points, extrinsicFactors);
	if(!success) return false;
*/


	return true;
}
