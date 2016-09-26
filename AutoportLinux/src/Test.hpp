#include <iostream>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/core.hpp>
#include <unistd.h>

#include "GenPurpFunc.hpp"
#include "ImgAnalysis.hpp"
#include "ImgLoader.hpp"
#include "PatternAnalysis.hpp"
#include "PositionEstimation.hpp"

using namespace cv;

extern string workingDir;

namespace Test {

	void cameraCapture() {

		Size frameSize(800,600);
		auto loader = ImgLoader("", ImgLoader::DEVICE);
		const string windowName("Video stream");
		namedWindow(windowName, WINDOW_AUTOSIZE);

		const string fileName = workingDir + "output.avi";
		int frameWidht = loader.getFrameWidth();
		int frameHeight = loader.getFrameHeight();
		VideoWriter video(fileName, CV_FOURCC('M','J','P','G'),10, Size(frameWidht,frameHeight), true);
		Mat frame;
		char c = 64;
		while(c != 27) {
			loader.getNextFrame(frame);
			video.write(frame);
			imshow(windowName, frame);
			c = (char)waitKey(33);
		}



	}

	void imgAnalysisPositionEstimationPic() {
		//--- Image analysis and position estimation test ----

		auto imgAnalyzer = ImgAnalysis(LedColor::RED);

		auto posEstimator = PositionEstimation();
		string imgName;
		auto ledPoints = vector<Point2f>();
		ImgLoader loader("",ImgLoader::DEVICE);
		Mat img;
		int downscalingFactor = 1;
		while(true) {


			cout << "\n\nLoading image " << imgName << endl;
			loader.getNextFrame(img);
			bool downscalingNeeded = imgAnalyzer.evaluate(img, ledPoints, downscalingFactor);
			if(downscalingNeeded)
				cout << "\nDownscaling needed!";

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
		}

		return;
	}

	void positionEstimation() {

		auto ledPoints = vector<Point2f>(8);
		ledPoints.at(0) = {576.71, 741.49};
		ledPoints.at(1) = {247.17, 741.49};
		ledPoints.at(2) = {-741.5, 741.5};
		ledPoints.at(3) = {617.91, 441.37};
		ledPoints.at(4) = {247.17, 441.94};
		ledPoints.at(5) = {741.49, 441.94};
		ledPoints.at(6) = {576.72, -741.49};
		ledPoints.at(7) = {-741.5, -741.5};

		auto posEstimator = PositionEstimation();
		auto position = Matrix<double,3,2>();
		posEstimator.setPointsToEvaluate(0xAB)->evaluate(ledPoints, position);
		cout << "\nCurrent position is:\n";
		GenPurpFunc::printMatrixd(position,3,2);

		getchar();
		return;
	}

}

