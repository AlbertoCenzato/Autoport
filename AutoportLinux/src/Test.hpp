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

#ifndef TEST_HPP_
#define TEST_HPP_

#include <iostream>

#include "ImgLoader/ImgFileLoader.hpp"
#include "Analysis/IPPAnalysis.hpp"

using namespace cv;

extern string   workingDir;
extern Settings settings;

ofstream ledStream("led.txt");
ofstream times("times.txt");

namespace Test {

void ippAnalysis(const string& path) {
	ImgLoader *loader;
	if(path.compare("d") != 0) {
		loader = new ImgFileLoader(path,false);
	}
	else {
		//loader = new ImgDeviceLoader();
	}

	if(!loader->isOpen()) {
		cerr << "Input not found!" << endl;
		return;
	}
	cout << "LOADER OK" << endl;

	Mat extrinsicFactors = Mat::zeros(3,4,CV_32FC1);
	auto ipp = IPPAnalysis(loader);
	//int count = 0, maxFramesToSkip = 5;

	ofstream stream("drone.txt");

	if(!stream.is_open()) {
		cout << "Couldn't open stream!";
		return;
	}

	char ch = 64;
	Result success;

	while(ch != 27) {
		success = ipp.evaluate(extrinsicFactors);
		if(success == Result::END)
			break;
		ipp.reset();
		if(success == Result::FAILURE) {
			//++count;
			//if(count > maxFramesToSkip) {
			//	count = 0;
			//}
			extrinsicFactors = Mat::zeros(3,4,CV_32FC1);
		}
		cout << "Position:\n" << extrinsicFactors << endl;
		for(int i = 0; i < 3; ++i) {
			for(int j = 0; j < 4; ++j) {
				stream << extrinsicFactors.at<float>(i,j);
				stream << " ";
			}
			stream << "\n";
		}
		ch = waitKey(0);
	}

	stream.close();
	waitKey(0);

	delete loader;
}

void positionSensitivity() {
	auto positionEstimator = PositionEstimation();
	Mat extrinsicFactors = Mat::zeros(3,4,CV_32FC1);
	float f = 0;
	Point2f p1 = Point2f(1741,1108.42);
	Point2f p2 = Point2f(1832.81,1110.54);
	Point2f p3 = Point2f(1831.29,882.216);
	Point2f p4 = Point2f(0,0);
	Point2f p5 = Point2f(1684.48,790.512);
	Scalar color(0,0,0);
	vector<LedDescriptor> points = {LedDescriptor(p1,color,f),
			LedDescriptor(p2,color,f),
			LedDescriptor(p3,color,f),
			LedDescriptor(p4,color,f),
			LedDescriptor(p5,color,f)};
	bool success = positionEstimator.evaluate(points,extrinsicFactors);
	if(success)
		cout << extrinsicFactors << endl;
	else
		cout << "Fallito" << endl;
}

} //namespace Test

#endif
