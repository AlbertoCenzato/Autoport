
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

	PatternAnalysis *patternAnalysis = new PatternAnalysis();
	ImgAnalysis *imgAnalyzer = new ImgAnalysis(low, high, LedColor::RED, patternAnalysis);
	imgAnalyzer->setColorTolerance  (colorTolerance)
			   ->setROItolerance    (ROItolerance)
			   ->setSizeTolerance   (sizeTolerance)
			   ->setSizeSupTolerance(sizeSupTolerance);

	Position_XYZ_YPR *initialPosition = new Position_XYZ_YPR();
	initialPosition->x 	   = 0;
	initialPosition->y 	   = 0;
	initialPosition->z 	   = 1700;
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
	for (int i = 0; i < 4; i++) {

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

		Mat img;
		imread(imgName, IMREAD_COLOR).copyTo(img);
		imwrite(resourcesPath + "output/originalImage.jpg",img);

		int downscalingFactor = 1;
		bool downscalingNeeded = imgAnalyzer->evaluate(img, ledPoints, downscalingFactor);
		if(downscalingNeeded)
			cout << "\nDownscaling needed!";

		cout << "\n\nPunti traslati:";
		Point2f trasl = Point2f(-1296,972);
		for(int i = 0; i < 8; i++) {
			ledPoints->at(i).y = -ledPoints->at(i).y;
			ledPoints->at(i) = ledPoints->at(i) + trasl;
			cout << "\nPunto " << i << ": [" << ledPoints->at(i).x << "," << ledPoints->at(i).y << "]";
		}

		cout << "\n\nEVALUATING POSITION...";
		Matrix<float,3,2> *position = posEstimator->evaluate(ledPoints);
		cout << "\nCurrent position is:\n";
		GenPurpFunc::printMatrixf(*position,3,2);
	}

	delete imgAnalyzer;
	delete initialPosition;
	delete realWorldPoints;
	delete posEstimator;
	delete ledPoints;

	return;
}

void testPositionEstimation() {

	vector<Point2f> *ledPoints = new vector<Point2f>(8);
	ledPoints->at(0) = {576.71, 741.49};
	ledPoints->at(1) = {247.17, 741.49};
	ledPoints->at(2) = {-741.5, 741.5};
	ledPoints->at(3) = {617.91, 441.37};
	ledPoints->at(4) = {247.17, 441.94};
	ledPoints->at(5) = {741.49, 441.94};
	ledPoints->at(6) = {576.72, -741.49};
	ledPoints->at(7) = {-741.5, -741.5};

	Position_XYZ_YPR *initialPos = new Position_XYZ_YPR();
		initialPos->x 	   = 0;
		initialPos->y 	   = 0;
		initialPos->z 	   = 300;
		initialPos->yaw   = 90;
		initialPos->pitch = 0;
		initialPos->roll  = 180;

		vector<Point3f> *realWorldPoints = new vector<Point3f>(8);
		realWorldPoints->at(0) = { 90, 70,0};
		realWorldPoints->at(1) = { 90, 30,0};
		realWorldPoints->at(2) = { 90,-90,0};
		realWorldPoints->at(3) = { 50, 70,0};
		realWorldPoints->at(4) = { 50, 30,0};
		realWorldPoints->at(5) = { 50,-90,0};
		realWorldPoints->at(6) = {-90, 70,0};
		realWorldPoints->at(7) = {-90, 90,0};

	PositionEstimation *posEstimator = new PositionEstimation(initialPos, realWorldPoints);
	Matrix<float,3,2> *position = posEstimator->evaluate(ledPoints);
	cout << "\nCurrent position is:\n";
	GenPurpFunc::printMatrixf(*position,3,2);

	getchar();
	return;
}
