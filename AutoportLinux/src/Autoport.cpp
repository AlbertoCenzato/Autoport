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

#include "GenPurpFunc.h"
#include "ImgAnalysis.h"
//#include "Simulations.h"

using namespace std;
using namespace cv;

int main() {

	string path = "/home/alberto/Pictures/foto/secondo_laboratorio/";
	string imgName = path + "1ms170cm.jpg";
	
	//---Color filtering----
		int lowH = 105;
		int highH = 135;

		int lowS = 150;//100;
		int highS = 255;//200;

		int lowV = 0;//50;
		int highV = 255;//150;
		
		Mat img = imread(imgName, IMREAD_COLOR);
		imwrite("/home/alberto/Pictures/foto/output/originalImage.jpg",img);
		Scalar low =  Scalar(lowH, lowS, lowV);
		Scalar high = Scalar(highH, highS, highV);
		int colorTolerance = 60;
		int ROItolerance = 500;
		int sizeTolerance = 300;

		SimpleBlobDetector::Params startParam;
		startParam.filterByColor = true;
		startParam.blobColor = 255;
		startParam.filterByInertia = false;
		startParam.minInertiaRatio = 0.3;
		startParam.maxInertiaRatio = 1;
		startParam.filterByArea = true;
		startParam.minArea = 30;
		startParam.maxArea = 1000;
		startParam.filterByConvexity = false;
		startParam.minConvexity = 0.2;
		startParam.maxConvexity = 1;
		startParam.filterByCircularity = false;
		startParam.minCircularity = 0.2;
		startParam.maxCircularity = 1;

		ImgAnalysis *imgAnalyzer = new ImgAnalysis(low, high, startParam, LedColor::RED);
		imgAnalyzer->setColorTolerance(colorTolerance)->setROItolerance(ROItolerance)->setSizeTolerance(sizeTolerance);
		for (int i = 0; i < 3; i++) {

			vector<Point2f> *ledPoints = imgAnalyzer->evaluate(img);

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

			switch (i) {
				case 0:
					imgName = path + "1ms100cm0deg.jpg";
					break;
				case 1:
					imgName = path + "01ms50cm0deg.jpg";
					break;
				case 2:
					imgName = path + "1ms30cm0deg.jpg";
					break;
				case 3:
					imgName = path + "1ms15cm0deg.jpg";
					break;
			}
			img = imread(imgName, IMREAD_COLOR);
			imwrite("/home/alberto/Pictures/foto/output/originalImage.jpg",img);
			/*
			// Crop the full image to that image contained by the rectangle myROI
			// Note that this doesn't copy the data
			img = img(regionOfInterest);
			namedWindow("Original", WINDOW_NORMAL);
			imshow("Original", img); //show the original image

			//downsampling
			//pyrDown(img, img);
			//pyrDown(img, img);
			Mat imageThresholded;

			//Convert the captured frame from BGR to HSV
			int64 start = getTickCount();
			cvtColor(img, img, COLOR_BGR2HSV);
			std::cout << "\nTime elapsed in RGB->HSV conversion: " << (getTickCount() - start) / getTickFrequency() << "s";

			//color filtering
			start = getTickCount();
			imageThresholded = ImgAnalysis::filterByColor(img, low, high);
			std::cout << "\nTime elapsed in color filtering: " << (getTickCount() - start) / getTickFrequency() << "s";

			//double totalTime = (((double)getTickCount()) - start) / getTickFrequency();

			//---Blob detection---
			SimpleBlobDetector::Params params;
			params.filterByColor = true;
			params.blobColor = 255;
			params.filterByInertia = true;
			params.minInertiaRatio = 0.3;
			params.maxInertiaRatio = 1;
			params.filterByArea = true;
			params.minArea = 50;
			params.maxArea = 700;
			params.filterByConvexity = true;
			params.minConvexity = 0.5;
			params.maxConvexity = 1;
			params.filterByCircularity = true;
			params.minCircularity = 0.5;
			params.maxCircularity = 1;

			start = getTickCount();
			vector<Point2f> ledPoints = ImgAnalysis::findBlobs(imageThresholded, params);
			std::cout << "\nTime elapsed in blob detection: " << (getTickCount() - start) / getTickFrequency() << "s";
			int ledPointsLength = ledPoints.size();
			for (int i = 0; i < ledPointsLength; i++) {
				std::cout << "\nPoint " << i + 1 << ": x[" << ledPoints[i].x << "] y[" << ledPoints[i].y << "]";
			}

			waitKey(25);

			int maxH = 0;
			int maxS = 0;
			int maxV = 0;
			int minH = 255;
			int minS = 255;
			int minV = 255;

			for (int i = 0; i < ledPointsLength; i++) {
				Point2f p = ledPoints[i];
				Vec3b color = img.at<Vec3b>(p);
				if (color[0] > maxH)	maxH = color[0];
				if (color[1] > maxS)	maxS = color[1];
				if (color[2] > maxV)	maxV = color[2];
				if (color[0] < minH)	minH = color[0];
				if (color[1] < minS)	minS = color[1];
				if (color[2] < minV)	minV = color[2];
			}
			low  = Scalar(minH - tolerance, minS - tolerance, minV - tolerance);
			high = Scalar(maxH + tolerance, maxS + tolerance, maxV + tolerance);

			start = getTickCount();
			vector<Point2f> keyPoints = ImgAnalysis::patternMirko(ledPoints, imageThresholded, 10);
			std::cout << "\nTime elapsed in pattern analysis: " << (getTickCount() - start) / getTickFrequency() << "s";

			Point2f *maxX = GenPurpFunc::findMaxXInVec(keyPoints);
			Point2f *maxY = GenPurpFunc::findMaxYInVec(keyPoints);
			Point2f *minX = GenPurpFunc::findMinXInVec(keyPoints);
			Point2f *minY = GenPurpFunc::findMinYInVec(keyPoints);

			regionOfInterest = Rect(minX->x - 100, minY->y - 100, maxX->x - minX->x + 200, maxY->y - minY->y + 200);

			Matrix<double, 3, 4> realPoints;
			realPoints << -50, -50, 30, -30,  //1, 3, 7, 5
				-30, 20, -20, -10,
				0, 0, 20, 0;
			double focale = 3.46031; //[mm]
			Vector3d p1 = { 1.4 / 1000 * keyPoints[0].x, 1.4 / 1000 * keyPoints[0].y, focale };
			Vector3d p2 = { 1.4 / 1000 * keyPoints[2].x, 1.4 / 1000 * keyPoints[2].y, focale };
			Vector3d p3 = { 1.4 / 1000 * keyPoints[6].x, 1.4 / 1000 * keyPoints[6].y, focale };
			Vector3d p4 = { 1.4 / 1000 * keyPoints[4].x, 1.4 / 1000 * keyPoints[4].y, focale };
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

			imgName = path + "p7d" + to_string(i) + "a30.bmp";
			img = imread(imgName, ImreadModes::IMREAD_COLOR);

			*/
		}
		getchar();
		return 0;
}

