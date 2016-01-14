
#include "stdafx.h"
#include <iostream>
#include "ImgAnalysisFunctions.h"
#include "Functions.h"
#include "Simulations.h"

using namespace std;

//entrypoint function from the main (OpenCVtest2.cpp) 
void run() {

	string path = "C:\\Users\\alber\\OneDrive\\Documenti\\Universita\\Progetto Autoport\\Sensori\\foto\\Primo laboratorio\\";
	string imgName = path + "p7d300a1.bmp";
	cv::Mat image;
	vector<cv::Point2f> keyPoints = patternMirko(imgLedDetection(imgName,image),image, 20);
	
	getchar();
	return;
}