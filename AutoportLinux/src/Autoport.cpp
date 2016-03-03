//============================================================================
// Name        : Autoport.cpp
// Author      : Alberto Cenzato
// Version     : 2.0
// Copyright   : Your copyright notice
// Description : Image analysis software for Autoport project
//============================================================================

#include <iostream>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "Functions.h"
#include "ImgAnalysis.h"
#include "Simulations.h"

using namespace std;
using namespace cv;

int main() {

	string path = "/home/alberto/Pictures/foto/primo_laboratorio/";

	//---Color filtering----
		int iLowH = 80;
		int iHighH = 110;

		int iLowS = 100;
		int iHighS = 200;

		int iLowV = 50;
		int iHighV = 150;

		string imgName = path + "p7d600a30.bmp";
		Mat img = imread(imgName, IMREAD_COLOR);

		Scalar low =  Scalar(iLowH, iLowS, iLowV);
		Scalar high = Scalar(iHighH, iHighS, iHighV);
		int tolerance = 20;
		Rect regionOfInterest(0, 0, img.cols, img.rows);

		for (int i = 500; i > 100; i=i-100) {

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

			Point2f *maxX = findMaxXInVec(keyPoints);
			Point2f *maxY = findMaxYInVec(keyPoints);
			Point2f *minX = findMinXInVec(keyPoints);
			Point2f *minY = findMinYInVec(keyPoints);

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
			Matrix<double, 3, 4> ret = p3p_solver(realPoints, cameraSystemPoints);
			printMatrix(ret, 3, 4);

			imgName = path + "p7d" + to_string(i) + "a30.bmp";
			img = imread(imgName, ImreadModes::IMREAD_COLOR);

		}
		getchar();
		return 0;
}
