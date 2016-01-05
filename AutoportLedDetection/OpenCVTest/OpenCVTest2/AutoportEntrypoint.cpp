
#include "stdafx.h"
#include <stdio.h>
#include <io.h>
#include "ImgAnalysisFunctions.h"
#include "Functions.h"
#include "Simulations.h"

using namespace std;

//entrypoint function from the main (OpenCVtest2.cpp) 
void run() {

	string imgName = "image.jpg";
	imgLedDetection(imgName);
	

	return;
}