
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
	Matrix<double, 3, 4> realPoints;
	realPoints << -50, -50,  30, -30,  //1, 3, 7, 5
				  -30,  20, -20, -10,
					0,   0,  20,   0;
	double focale = 3.46031; //[mm]
	Vector3d p1 = { keyPoints[0].x, keyPoints[0].y, focale };
	Vector3d p2 = { keyPoints[2].x, keyPoints[2].y, focale };
	Vector3d p3 = { keyPoints[6].x, keyPoints[6].y, focale };
	Vector3d p4 = { keyPoints[4].x, keyPoints[4].y, focale };
	Vector3d translation = { 2592 / 2, 1944 / 2, 0 };
	Vector3d p1t = (p1 - translation);
	Vector3d p2t = (p2 - translation);
	Vector3d p3t = (p3 - translation);
	Vector3d p4t = (p4 - translation);
	p1t.normalize();
	p2t.normalize();
	p3t.normalize();
	p4t.normalize();
	Matrix<double, 3, 4> cameraSystemPoints;
	cameraSystemPoints << p1t, p2t, p3t, p4t;
	Matrix<double, 3, 4> ret = p3p_solver(realPoints, cameraSystemPoints);
	printMatrix(ret, 3, 4);
	getchar();
	return;
}