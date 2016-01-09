
#include "stdafx.h"
#include <iostream>
#include "ImgAnalysisFunctions.h"
#include "Functions.h"
#include "Simulations.h"

using namespace std;

//entrypoint function from the main (OpenCVtest2.cpp) 
void run() {

	string imgName = "pattern1.jpg";
	//vidLedDetection(imgName);
	
	cv::Mat image;
	vector<cv::KeyPoint> keyPoints = pattern1(imgLedDetection(imgName,image),image);
	
	
	getchar();
	return;
}