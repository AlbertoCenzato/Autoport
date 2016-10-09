//============================================================================
// Name        : Autoport.cpp
// Author      : Alberto Cenzato
// Version     : 2.0
// Copyright   : Copyright (c) 2016 Alberto Cenzato. All rights reserved.
// Description : Software for Autoport project
//============================================================================

#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/ocl.hpp>

#include "GenPurpFunc.hpp"
#include "ImgAnalysis.hpp"
#include "ImgLoader.hpp"
#include "PatternAnalysis.hpp"
#include "Settings.hpp"
#include "Test.hpp"

using namespace std;
using namespace cv;

enum Status {
	LOOKING_FOR_TARGET,
	FIRST_LANDING_PHASE,
	SECOND_LANDING_PHASE,

	HOVERING,
	ERROR,
	MANUAL_CONTROL,

	LANDED
};

string workingDir;
const string configFileName = "autoport.config";
Status status = Status::LOOKING_FOR_TARGET;

int main() {

	cout << "****** AUTOPORT SOFTWARE ******\n" << endl;

	cout << "Reading working directory..." << endl;
	const int bufferSize = 500;
	char workDir[bufferSize];
	workingDir = getcwd(workDir, bufferSize);
	workingDir += "/";

	cout << "Done. Working directory is: " << workingDir << endl;

	string path = workingDir + configFileName;
	cout << "Opening and parsing " << path << endl;
	cout << "\n*** Settings ***" << endl;
	Settings::loadConfiguration(path);
	cout << Settings::toString() << "\n\n" << endl;

	//Test::notteDellaRicerca();
	Test::imgAnalysisPositionEstimationPic();

	getchar();
	return 0;
}

