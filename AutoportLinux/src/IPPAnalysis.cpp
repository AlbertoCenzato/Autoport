/*
 * IPPAnalysis.cpp
 *
 *  Created on: Oct 12, 2016
 *      Author: alberto
 */

#include "IPPAnalysis.hpp"

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

	Mat image;
	bool retrieved = loader->getNextFrame(image);
	if(!retrieved) {
		cerr << "Couldn't retrieve frame!" << endl;
		return false;
	}

	vector<Point2f> points = vector<Point2f>(10);

	bool success = imageAnalyzer.evaluate(image, points, 1);
	if(!success) {
		cerr << "ImageAnalysis failed!" << endl;
		return false;
	}
	cout << "ImageAnalysis succeded!" << endl;

	Scalar red(0,0,255);
	for(uint i = 0; i < points.size(); ++i) {
		string number = to_string(i);
		putText(image, number, points[i], HersheyFonts::FONT_HERSHEY_PLAIN,
				2,red,10,8);
	}

	success 	 = patternAnalyzer.evaluate(points, 10);
	if(!success) {
		cerr << "PatternAnalysis failed!" << endl;
		return false;
	}
	cout << "PatternAnalysis succeded!" << endl;

	Scalar blue(255,0,0);
	for(int i = 0; i < points.size(); ++i) {
		string number = to_string(i);
		putText(image, number, points[i], HersheyFonts::FONT_HERSHEY_PLAIN,
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
