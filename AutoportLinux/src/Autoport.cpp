//============================================================================
// Name        : Autoport.cpp
// Author      : Alberto Cenzato
// Version     : 2.0
// Copyright   : Copyright (c) 2016 Alberto Cenzato. All rights reserved.
// Description : Image analysis software for Autoport project
//============================================================================

#include <iostream>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/ocl.hpp>
#include <unistd.h>

#include "GenPurpFunc.h"
#include "ImgAnalysis.h"
#include "PatternAnalysis.h"
#include "Test.h"
//#include "Simulations.h"

using namespace std;
using namespace cv;

string resourcesPath;

int main() {

	const char *homeDir = getenv("HOME");
	string homeDirectory = string(homeDir);

	//check if the workspace name is "workspace"
	string thisFilePath = homeDirectory + "/workspace/Autoport/src/Autoport.cpp";
	if( access( thisFilePath.c_str(), F_OK ) == -1 ) {
		cout << "ERRORE: DIRECTORY CORRENTE ERRATA, PROBABILMENTE IL NOME DEL WORKSPACE NON E' \"WORKSPACE\"";
		sleep(10000);
		return -1;
	}

	resourcesPath = homeDirectory + "/workspace/Autoport/Resources/";


	ocl::setUseOpenCL(true); // enable OpenCL in the processing of Mat

	
	testImgAnalysisPositionEstimationPic();
	//testImgAnalysisPositionEstimationPic();

	getchar();
	return 0;
}

