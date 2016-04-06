/*
 * PatternAnalysis.h
 *
 *  Created on: Apr 6, 2016
 *      Author: alberto
 */

#ifndef PATTERNANALYSIS_H_
#define PATTERNANALYSIS_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "GenPurpFunc.h"

using namespace std;
using namespace cv;

namespace PatternAnalysis {

	void patternMirko(vector<KeyPoint>*, Mat&, int);
}



#endif /* PATTERNANALYSIS_H_ */
