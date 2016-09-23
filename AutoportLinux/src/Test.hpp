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
#include "PatternAnalysis.hpp"
#include "PositionEstimation.hpp"
#include "ImageLoader.hpp"

using namespace cv;

extern string workingDir;

namespace Test {

	void cameraCapture() {

		auto loader = ImageLoader("http://192.168.1.6:8080/stream", ImageLoader::STREAM);
		Mat frame;
		loader.getNextFrame(frame);
		if(imwrite(workingDir + "Resources/output/frame.jpg",frame))
			cout << "Frame saved" << endl;
		else
			cout << "Error while saving frame" << endl;

	}

	void imgAnalysisPositionEstimationPic() {
		//--- Image analysis and position estimation test ----

		auto patternAnalysis = PatternAnalysis();
		auto imgAnalyzer = ImgAnalysis(LedColor::RED, patternAnalysis);

		auto posEstimator = PositionEstimation();
		string imgName;
		auto ledPoints = vector<Point2f>();
		for (int i = 0; i < 1; i++) {

			switch (i) {
			case 0:
				imgName = workingDir + "Resources/secondo_laboratorio/1ms170cm.jpg";
				break;
			case 1:
				imgName = workingDir + "Resources/secondo_laboratorio/1ms100cm0deg.jpg";
				break;
			case 2:
				imgName = workingDir + "Resources/secondo_laboratorio/01ms50cm0deg.jpg";
				break;
			case 3:
				imgName = workingDir + "Resources/secondo_laboratorio/1ms30cm0deg.jpg";
				break;
			case 4:
				imgName = workingDir + "Resources/secondo_laboratorio/1ms15cm0deg.jpg";
				break;
			}

			cout << "\n\nLoading image " << imgName << endl;
			Mat img;
			imread(imgName, IMREAD_COLOR).copyTo(img);
			imwrite(workingDir + "Resources/output/originalImage.jpg",img);

			int downscalingFactor = 1;
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

		vector<Point2f> *ledPoints = new vector<Point2f>(8);
		ledPoints->at(0) = {576.71, 741.49};
		ledPoints->at(1) = {247.17, 741.49};
		ledPoints->at(2) = {-741.5, 741.5};
		ledPoints->at(3) = {617.91, 441.37};
		ledPoints->at(4) = {247.17, 441.94};
		ledPoints->at(5) = {741.49, 441.94};
		ledPoints->at(6) = {576.72, -741.49};
		ledPoints->at(7) = {-741.5, -741.5};

		auto posEstimator = PositionEstimation();
		auto position = Matrix<double,3,2>();
		posEstimator.setPointsToEvaluate(0xAB)->evaluate(*ledPoints, position);
		cout << "\nCurrent position is:\n";
		GenPurpFunc::printMatrixd(position,3,2);

		getchar();
		return;
	}

}

