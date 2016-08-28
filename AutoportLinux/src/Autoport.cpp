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

#include "Test.hpp"

using namespace std;
using namespace cv;

string resourcesPath;
string workingDir;
string configFileName = "autoport.config";

int main() {

	cout << "Autoport funziona!!!!!!!!!!!!!!!!!!!!!!" << endl;

	const char *homeDir = getenv("HOME");
	string homeDirectory = string(homeDir);

	//check if the workspace name is "workspace"
	string thisFilePath = homeDirectory + "/workspace/Autoport/src/Autoport.cpp";
	if( access( thisFilePath.c_str(), F_OK ) == -1 ) {
		cout << "ERRORE: DIRECTORY CORRENTE ERRATA, PROBABILMENTE IL NOME DEL WORKSPACE NON E' \"WORKSPACE\"" << endl;
		sleep(5000);
		return -1;
	}

	resourcesPath = homeDirectory + "/workspace/Autoport/Resources/";

	cout << "Reading working directory..." << endl;
	int bufferSize = 200;
	char* workDir = new char[bufferSize];
	workingDir = getcwd(workDir, bufferSize);
	cout << "Done" << endl;

	string path = workingDir + "/" + configFileName;
	cout << "Opening and parsing " << path << endl;
	Settings::loadConfiguration(path);
	cout << Settings::toString();

	ocl::setUseOpenCL(true); // enable OpenCL in the processing of Mat

	
	//testImgAnalysisPositionEstimationPic();
	//testImgAnalysisPositionEstimationPic();

	getchar();
	return 0;
}

