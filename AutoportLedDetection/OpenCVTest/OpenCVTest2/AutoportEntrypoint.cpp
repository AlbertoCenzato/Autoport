
#include "stdafx.h"
#include "ImgAnalysisFunctions.h"
#include "Functions.h"
#include "Simulations.h"

using namespace std;

//entrypoint function from the main (OpenCVtest2.cpp) 
void run() {

	string imgName = "image.jpg";
	pattern1(imgLedDetection(imgName));

	getchar();
	return;
}