
#include <iostream>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unistd.h>

#include "GenPurpFunc.h"
#include "ImgAnalysis.h"
#include "PatternAnalysis.h"
#include "PositionEstimation.h"

extern string resourcesPath;

void testImgAnalysisPositionEstimationPic() {
	//--- Image analysis and position estimation test ----

	int lowH = 105, highH = 135;
	int lowS = 150, highS = 255;
	int lowV = 0,   highV = 255;

	Scalar low =  Scalar(lowH, lowS, lowV);
	Scalar high = Scalar(highH, highS, highV);
	int colorTolerance = 60;
	int ROItolerance = 500;
	int sizeTolerance = 300;
	int sizeSupTolerance = 128;

	function<void(vector<KeyPoint>*, Mat&, int)> patternAnalysis = &PatternAnalysis::patternMirko;
	ImgAnalysis *imgAnalyzer = new ImgAnalysis(low, high, LedColor::RED, patternAnalysis);
	imgAnalyzer->setColorTolerance  (colorTolerance)
			   ->setROItolerance    (ROItolerance)
			   ->setSizeTolerance   (sizeTolerance)
			   ->setSizeSupTolerance(sizeSupTolerance);

	Position_XYZ_YPR *initialPosition = new Position_XYZ_YPR();
	initialPosition->x 	   = 0;
	initialPosition->y 	   = 0;
	initialPosition->z 	   = 170;
	initialPosition->yaw   = 0;
	initialPosition->pitch = 0;
	initialPosition->roll  = 0;

	vector<Point3f> *realWorldPoints = new vector<Point3f>(8);
	realWorldPoints->at(0) = { 90, 70,0};
	realWorldPoints->at(1) = { 90, 30,0};
	realWorldPoints->at(2) = { 90,-90,0};
	realWorldPoints->at(3) = { 50, 70,0};
	realWorldPoints->at(4) = { 50, 30,0};
	realWorldPoints->at(5) = { 50,-90,0};
	realWorldPoints->at(6) = {-90, 70,0};
	realWorldPoints->at(7) = {-90, 90,0};

	PositionEstimation *posEstimator = new PositionEstimation(initialPosition,realWorldPoints);
	string imgName;
	vector<Point2f> *ledPoints = new vector<Point2f>();
	for (int i = 0; i < 3; i++) {

		switch (i) {
			case 0:
				imgName = resourcesPath + "secondo_laboratorio/1ms170cm.jpg";
				break;
			case 1:
				imgName = resourcesPath + "secondo_laboratorio/1ms100cm0deg.jpg";
				break;
			case 2:
				imgName = resourcesPath + "secondo_laboratorio/01ms50cm0deg.jpg";
				break;
			case 3:
				imgName = resourcesPath + "secondo_laboratorio/1ms30cm0deg.jpg";
				break;
			case 4:
				imgName = resourcesPath + "secondo_laboratorio/1ms15cm0deg.jpg";
				break;
		}

		Mat img = imread(imgName, IMREAD_COLOR);
		imwrite(resourcesPath + "output/originalImage.jpg",img);

		int downscalingFactor = 1;
		bool downscalingNeeded = imgAnalyzer->evaluate(img, ledPoints, downscalingFactor);
		if(downscalingNeeded)
			cout << "\nDownscaling needed!";

		cout << "\n\nEVALUATING POSITION...";
		Matrix<float,3,2> *position = posEstimator->evaluate(ledPoints);
		cout << "\nCurrent position is:\n";
		GenPurpFunc::printMatrixf(*position,3,2);
		/*
		Matrix<double, 3, 4> realPoints;
		realPoints << -50, -50, 30, -30,  //1, 3, 7, 5
				-30, 20, -20, -10,
				0, 0, 20, 0;
		double focale = 3.46031; //[mm]
		Point2f point = ledPoints->at(0);
		Vector3d p1 = { 1.4 / 1000 * point.x, 1.4 / 1000 * point.y, focale };
		point = ledPoints->at(2);
		Vector3d p2 = { 1.4 / 1000 * point.x, 1.4 / 1000 * point.y, focale };
		point = ledPoints->at(6);
		Vector3d p3 = { 1.4 / 1000 * point.x, 1.4 / 1000 * point.y, focale };
		point = ledPoints->at(4);
		Vector3d p4 = { 1.4 / 1000 * point.x, 1.4 / 1000 * point.y, focale };

		Vector3d translation = { 1.4 / 1000 * 2592 / 2, 1.4 / 1000 * 1944 / 2, 0 };
		Vector3d p1t = (translation - p1);
		Vector3d p2t = (translation - p2);
		Vector3d p3t = (translation - p3);
		Vector3d p4t = (translation - p4);
		p1t.normalize();
		p2t.normalize();
		p3t.normalize();
		p4t.normalize();
		Matrix<double, 3, 4> cameraSystemPoints;
		cameraSystemPoints << p1t, p2t, p3t, p4t;
		Matrix<double, 3, 4> ret = GenPurpFunc::p3p_solver(realPoints, cameraSystemPoints);
		GenPurpFunc::printMatrix(ret, 3, 4);
		*/
	}

	delete imgAnalyzer;
	delete initialPosition;
	delete realWorldPoints;
	delete posEstimator;
	delete ledPoints;

	return;
}

void testImgAnalysisVid() {

	return;
}
