//============================================================================
// Name        : Autoport.cpp
// Author      : Alberto Cenzato
// Version     : 2.0
// Copyright   : Copyright (c) 2016 Alberto Cenzato. All rights reserved.
// Description : Image analysis software for Autoport project
//============================================================================

#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/ocl.hpp>

#include "GenPurpFunc.hpp"
#include "ImgAnalysis.hpp"
#include "PatternAnalysis.hpp"
#include "Settings.hpp"
#include "ImageLoader.hpp"

#include "Test.hpp"

using namespace std;
using namespace cv;

//string resourcesPath;
string workingDir;
const string configFileName = "autoport.config";

int main() {

	cout << "****** AUTOPORT SOFTWARE ******\n" << endl;

	cout << "Reading working directory..." << endl;
	int bufferSize = 500;
	char* workDir = new char[bufferSize];
	workingDir = getcwd(workDir, bufferSize);
	workingDir += "/";
	delete [] workDir;

	cout << "Done. Working directory is: " << workingDir << endl;

	string path = workingDir + configFileName;
	cout << "Opening and parsing " << path << endl;
	cout << "\n*** Settings ***" << endl;
	Settings::loadConfiguration(path);
	cout << Settings::toString();

	//ocl::setUseOpenCL(true); // enable OpenCL in the processing of Mat

	Test::cameraCapture();

	getchar();
	return 0;
}

