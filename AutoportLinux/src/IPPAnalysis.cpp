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

	/*
	success 	 = patternAnalyzer.evaluate(points, tolerance);
	if(!success) {
		cerr << "PatternAnalysis failed!" << endl;
		return false;
	}
	cout << "PatternAnalysis succeded!" << endl;
*/
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
