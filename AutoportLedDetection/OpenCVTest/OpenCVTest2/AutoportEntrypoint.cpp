
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
	for (int i = 0; i < 12; i++){
		cout << "\nPoint " << i + 1 << ": x[" << keyPoints[i].pt.x << "] y[" << keyPoints[i].pt.y << "]";
	}
	
	getchar();
	return;
}