/*
 * PatternAnalysis.cpp
 *
 *  Created on: Apr 6, 2016
 *      Author: alberto
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "GenPurpFunc.h"
#include "PatternAnalysis.h"

using namespace std;
using namespace cv;

// Led recognition algorithm. Gives a number to every led using the numbering convention
// in patterns' file (see "Sensori" folder in dropbox).
// @points: vector of Point2f of the leds' blob centroid.
// @img: image used to visualize identified leds' position and number.
// @tolerance: tolerance in the alignement (in pixels).
//returns: vector of Point2f ordered with the numbering convention

void PatternAnalysis::evaluate(vector<KeyPoint> *ledPoints, UMat &img, int tolerance) {

	patternMirko(ledPoints, img, tolerance);
	//nearerPoints(ledPoints, img, tolerance);

}

