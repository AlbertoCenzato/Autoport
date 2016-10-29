#include <iostream>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/core.hpp>
#include <unistd.h>

#include "GenPurpFunc.hpp"
#include "IPPAnalysis.hpp"

using namespace cv;

extern string   workingDir;
extern Settings settings;

ImgAnalysis imgAnalyzer;
const int MAX_VAL = 255;
int minHue = 0, maxHue = 255;
int minSat = 0, maxSat = 255;
int minVal = 0, maxVal = 255;

namespace Test {

	void on_trackbar(int, void*) {
		Scalar low (minHue, minSat, minVal);
		Scalar high(maxHue, maxSat, maxVal);
		Interval<Scalar> colorInterval(low, high);
		imgAnalyzer.setColorInterval(colorInterval);
	}

	void pointCloudRegister() {
		srand (time(NULL));
		Scalar white(255,255,255);
		Scalar red(0,0,255);
		Scalar green(0,255,0);
		Scalar blue(125,125,125);

		double meanError = 0;
		long timeElapsed = 0;
		int count = 0;

		const int ITERATIONS = 100000;
		for(int j = 0; j < ITERATIONS; ++j) {
			cout << j << endl;

			Vec3f points[5];
			Point2f originalPoints[5];
			for(int i = 0; i < 5; ++i) {
				points[i] = Vec3f(rand()%500,rand()%500,1);
				originalPoints[i] = Point2f(points[i].val[0],points[i].val[1]);
			}

			Point2f pivot(rand()%250+125,rand()%250+125);
			double angle = rand()%180;
			Mat_<float> rot = getRotationMatrix2D(pivot, angle, 1);

			Vec2f rotPoint[5];
			for(int i = 0; i< 5; ++i) {
				Mat_<float> result = rot*Mat_<float>(points[i]);
				rotPoint[i] = Vec2f(result);
			}

			vector<Point2f> rotPointsVec(5);
			vector<Point2f> pointsVec(5);
			for(int i = 0; i < 5; ++i) {
				rotPointsVec[i] = Point2f(rotPoint[i]);
				pointsVec[i]	= Point2f(points[i].val[0], points[i].val[1]);
			}
			rotPointsVec[rand()%5] = Point2f(rand()%500,rand()%500);

			auto begin = chrono::high_resolution_clock::now();
			Mat_<float> H = findHomography(rotPointsVec, pointsVec, RANSAC);
			if(H.empty())
				cout << "EMPTY!!" << endl;
			else {
				++count;
				auto end = chrono::high_resolution_clock::now();
				timeElapsed += chrono::duration_cast<chrono::nanoseconds>(end-begin).count();

				Point2f revPoint[5];
				for(int i = 0; i< 5; ++i) {
					Vec3f vec(rotPointsVec[i].x, rotPointsVec[i].y, 1);
					Mat_<float> result = H*Mat_<float>(vec);
					Vec3f reversedPoint(result);
					Point2f revPoint[i] = Point2f(reversedPoint.val[0],reversedPoint.val[1]);
				}

				double meanDistance = 0;
				for(int i = 0; i < 5; ++i) {
					meanDistance += GenPurpFunc::distPoint2Point(revPoint[i],originalPoints[i]);
				}
				meanError += meanDistance/5;
			}
		}

		meanError /= count;

		cout << "Mean time elapsed: " << timeElapsed/1000/ITERATIONS << "us" << endl;
		cout << "Mean error: " << meanError << endl;
	}

	void notteDellaRicerca() {

		Size frameSize(800,600);
		int fps = 25;
		cout << "opening image loader" << endl;
		ImgLoader loader(workingDir+"video.mp4", ImgLoader::DEVICE, frameSize, fps);
		cout << "done" << endl;
		imgAnalyzer = ImgAnalysis();
		vector<LedDescriptor> ledPoints(7);

		const string settingsWindow("Settings");
		const string processedFrame("Processed stream");
		//namedWindow(originalFrame,  WINDOW_NORMAL);
		namedWindow(processedFrame, WINDOW_NORMAL);

		Interval<Scalar> colorInterval;
		imgAnalyzer.getColorInterval(colorInterval);

		Scalar h = colorInterval.high;
		Scalar l = colorInterval.low;

		maxHue = h[0];
		maxSat = h[1];
		maxVal = h[2];
		minHue = l[0];
		minSat = l[1];
		minVal = l[2];

		imshow(settingsWindow, Mat::zeros(1,800,3));

		createTrackbar("Min hue", settingsWindow, &minHue, MAX_VAL, on_trackbar);
		createTrackbar("Max hue", settingsWindow, &maxHue, MAX_VAL, on_trackbar);

		createTrackbar("Min sat", settingsWindow, &minSat, MAX_VAL, on_trackbar);
		createTrackbar("Max sat", settingsWindow, &maxSat, MAX_VAL, on_trackbar);

		createTrackbar("Min val", settingsWindow, &minVal, MAX_VAL, on_trackbar);
		createTrackbar("Max val", settingsWindow, &maxVal, MAX_VAL, on_trackbar);


		Mat frame;
		char c = 64;
		float downscalingFactor = 1;
		while(c != 27 && loader.getNextFrame(frame)) {

			bool downScalingNeeded = imgAnalyzer.evaluate(frame, ledPoints, downscalingFactor);
			if(downScalingNeeded) {
				loader.setFrameHeight(loader.getFrameHeight()/2);
				loader.setFrameWidth (loader.getFrameWidth ()/2);
				downscalingFactor = 0.5;
			}
			imshow(processedFrame, frame);

			c = (char)waitKey(33);
		}

		//destroyWindow(processedFrame);

		const string hue = "hue";
		const string sat = "saturation";
		const string val = "value";

		const string low  = "low";
		const string high = "high";

		Settings& settings = Settings::getInstance();
		settings.modifyConfigParam(hue, low,  minHue);
		settings.modifyConfigParam(sat, low,  minSat);
		settings.modifyConfigParam(val, low,  minVal);
		settings.modifyConfigParam(hue, high, maxHue);
		settings.modifyConfigParam(sat, high, maxSat);
		settings.modifyConfigParam(val, high, maxVal);

		settings.saveConfig();
	}

	void cameraCapture() {

		//Size frameSize(800,600);
		auto loader = ImgLoader("", ImgLoader::DEVICE);
		const string windowName("Video stream");
		namedWindow(windowName, WINDOW_AUTOSIZE);

		/*
		const string fileName = workingDir + "output.avi";
		int frameWidht = loader.getFrameWidth();
		int frameHeight = loader.getFrameHeight();
		VideoWriter video(fileName, CV_FOURCC('M','J','P','G'),10, Size(frameWidht,frameHeight), true);
		*/
		Mat frame;
		char c = 64;
		while(c != 27) {
			loader.getNextFrame(frame);
			//video.write(frame);
			imshow(windowName, frame);
			c = (char)waitKey(33);
		}



	}

	//--- Image analysis and position estimation test ----
	void taraturaParametriChristian(const string &path) {

		auto posEstimator = PositionEstimation();
		string imgName;
		auto ledPoints = vector<LedDescriptor>();
		ImgLoader loader;
		if(path.compare("d") == 0)
			loader = ImgLoader(path,ImgLoader::DEVICE);
		else
			loader = ImgLoader(path,ImgLoader::FILE);

		Interval<Scalar> colorInterval;
		imgAnalyzer = ImgAnalysis();
		imgAnalyzer.getColorInterval(colorInterval);

		Scalar h = colorInterval.high;
		Scalar l = colorInterval.low;

		maxHue = h[0];
		maxSat = h[1];
		maxVal = h[2];
		minHue = l[0];
		minSat = l[1];
		minVal = l[2];

		const string settingsWindow("Settings");
		imshow(settingsWindow, Mat::zeros(1,800,3));

		createTrackbar("Min hue", settingsWindow, &minHue, MAX_VAL, on_trackbar);
		createTrackbar("Max hue", settingsWindow, &maxHue, MAX_VAL, on_trackbar);

		createTrackbar("Min sat", settingsWindow, &minSat, MAX_VAL, on_trackbar);
		createTrackbar("Max sat", settingsWindow, &maxSat, MAX_VAL, on_trackbar);

		createTrackbar("Min val", settingsWindow, &minVal, MAX_VAL, on_trackbar);
		createTrackbar("Max val", settingsWindow, &maxVal, MAX_VAL, on_trackbar);

		Mat image;
		loader.getNextFrame(image);
		int downscalingFactor = 1;
		namedWindow("Original image", WINDOW_NORMAL);
		char ch = 64;
		while(ch != 27) {

			cout << "\n\nLoading image " << imgName << endl;
			if(!loader.getNextFrame(image)) {
				cout << "Reached video ending" << endl;
				break;
			}

			imgAnalyzer.evaluate(image, ledPoints, downscalingFactor);
			for(uint i = 0; i < ledPoints.size(); ++i)
				circle(image,ledPoints[i].getPosition(),20,Scalar(0,255,0),10);

			imshow("Original image", image);
			ch = waitKey(0);

			/*
			cout << "\n\nPunti traslati:";
			Point2f trasl = Point2f(-1296,972);
			for(int i = 0; i < 8; i++) {
				ledPoints.at(i).y = -ledPoints.at(i).y;
				ledPoints.at(i) = ledPoints.at(i) + trasl;
				cout << "\nPunto " << i << ": [" << ledPoints.at(i).x << "," << ledPoints.at(i).y << "]";
			}

			cout << "\n\nEVALUATING POSITION...";
			auto position = Matrix<double,3,2>();
			posEstimator.evaluate(ledPoints, position);
			cout << "\nCurrent position is:\n";
			GenPurpFunc::printMatrixd(position,3,2);
			*/

		}

		Settings& settings = Settings::getInstance();
		settings.modifyConfigParam("hue",	     "low", minHue);
		settings.modifyConfigParam("hue",	     "high",maxHue);
		settings.modifyConfigParam("saturation", "low", minSat);
		settings.modifyConfigParam("saturation", "high",maxSat);
		settings.modifyConfigParam("value",		 "low", minVal);
		settings.modifyConfigParam("value",		 "high",maxVal);

		settings.saveConfig();

		return;
	}

	void ippAnalysis(const string& path) {
		ImgLoader loader;
		if(path.compare("d") == 0)
			loader = ImgLoader(path,ImgLoader::DEVICE);
		else
			loader = ImgLoader(path,ImgLoader::FILE);

		cout << "LOADER OK" << endl;

		Mat extrinsicFactors;
		auto ipp = IPPAnalysis(&loader);
		char ch = 64;
		while(ch != 23) {
			ipp.evaluate(extrinsicFactors);
			ch = waitKey(0);
		}
	}
}

