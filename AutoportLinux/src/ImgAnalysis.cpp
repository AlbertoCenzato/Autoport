//Copyright (c) 2016 Alberto Cenzato. All rights reserved.

#include "ImgAnalysis.hpp"
#include <chrono>

using namespace cv;
using namespace std;

extern string workingDir;

//--- Functions ---

//TODO: make the function accept a pointer to a pattern analysis function
bool ImgAnalysis::evaluate(Mat &image, vector<Point2f> &points, float downscalingFactor) {

	// Crop the full image according to the region of interest
	// Note that this doesn't copy the data
	if(regionOfInterest.height != 0 && regionOfInterest.width != 0)
		image = image(regionOfInterest);

	Mat hsvImg(image.rows,image.cols,image.depth());
	Mat colorFilteredImg(image.rows,image.cols,image.depth());

	//change color space: from BGR to HSV;
    //TODO: color conversion and filterByColor can be performed with a shader (?)
	auto begin = chrono::high_resolution_clock::now();
	cvtColor(image,hsvImg,colorConversion);
	auto end = chrono::high_resolution_clock::now();
	cout << "\nConvert color: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;
/*
	namedWindow("Cropped image", WINDOW_NORMAL);
	imshow("Cropped image", hsvImg);
	waitKey(1);
*/

	//filter the color according to this->low and this->high tolerances
	begin = chrono::high_resolution_clock::now();
	filterByColor(hsvImg,colorFilteredImg);
	end = chrono::high_resolution_clock::now();
	cout << "\nFilter color: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;
/*
	namedWindow("Filtered image", WINDOW_NORMAL);
	imshow("Filtered image", colorFilteredImg);
	waitKey(1);
	//imwrite(workingDir + "output/filterByColor.jpg", colorFilteredImg);
*/

	//put in this->points detected blobs that satisfy this->params tolerance
	begin = chrono::high_resolution_clock::now();
	int blobNumber = findBlobs(colorFilteredImg, image, downscalingFactor);
	end = chrono::high_resolution_clock::now();
	cout << "\nFound " << blobNumber << " blobs: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;

	namedWindow("Blobs found", WINDOW_NORMAL);
	imshow("Blobs found", colorFilteredImg);
	waitKey(1);

	/*
	cout << "Skip frame? [y/n]" << endl;
	char skip;
	cin >> skip;
	if(skip == 'n') {
	*/

	/*
		//order this->ledPoints accordingly to the led pattern numbering
		begin = chrono::high_resolution_clock::now();
		bool matchFound = patternAnalysis.evaluate(ledPoints, 10);
		end = chrono::high_resolution_clock::now();
		cout << "\nPattern: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;

		if(matchFound) {

			cout << "Match found!" << endl;
			for(int i = 0; i < ledPoints.size(); ++i) {
				char str[2];
				sprintf(str,"%d",i);
				putText(image, str, ledPoints[i], FONT_HERSHEY_PLAIN, 2,  Scalar(0,0,255,255));
			}

			GenPurpFunc::pointVectorToStrng(ledPoints);

			Interval<int> hue(0,255);
			Interval<int> sat(0,255);
			Interval<int> val(0,255);

			int ledPointsLength = ledPoints.size();
			for (int i = 0; i < ledPointsLength; i++) {
				Point2f p = ledPoints[i];
				Vec3b color = hsvImg.at<Vec3b>(p);
				if (color[0] > hue.high) hue.high = color[0];
				if (color[1] > sat.high) sat.high = color[1];
				if (color[2] > val.high) val.high = color[2];
				if (color[0] < hue.low)	 hue.low  = color[0];
				if (color[1] < sat.low)	 sat.low  = color[1];
				if (color[2] < val.low)	 val.low  = color[2];
			}
			colorInterval.low  = Scalar(hue.low  - colorTolerance, sat.low  - colorTolerance, val.low  - colorTolerance);
			colorInterval.high = Scalar(hue.high + colorTolerance, sat.high + colorTolerance, val.high + colorTolerance);

			findROI();

		}
		int averageSize = (oldKeyPointSizeInterval.low + oldKeyPointSizeInterval.high)/2;
		return false;//averageSize > sizeSupTolerance;
	//}
*/
	points = ledPoints;
	return true;
}

ImgAnalysis* ImgAnalysis::setROItolerance(int ROItolerance) {
	this->ROItolerance = ROItolerance;
	return this;
}
ImgAnalysis* ImgAnalysis::setColorTolerance(int colorTolerance) {
	this->colorTolerance = colorTolerance;
	return this;
}
ImgAnalysis* ImgAnalysis::setSizeTolerance(int sizeTolerance) {
	this->sizeTolerance = sizeTolerance;
	return this;
}
ImgAnalysis* ImgAnalysis::setSizeSupTolerance(int sizeSupTolerance) {
	this->sizeSupTolerance = sizeSupTolerance;
	return this;
}
ImgAnalysis* ImgAnalysis::setColorInterval(Interval<Scalar> &colorInterval) {
	this->colorInterval.high[0] = colorInterval.high[0];
	this->colorInterval.high[1] = colorInterval.high[1];
	this->colorInterval.high[2] = colorInterval.high[2];

	this->colorInterval.low[0] = colorInterval.low[0];
	this->colorInterval.low[1] = colorInterval.low[1];
	this->colorInterval.low[2] = colorInterval.low[2];

	return this;
}

void ImgAnalysis::getColorInterval(Interval<Scalar> &colorInterval) {
	colorInterval.high[0] = this->colorInterval.high[0];
	colorInterval.high[1] = this->colorInterval.high[1];
	colorInterval.high[2] = this->colorInterval.high[2];

	colorInterval.low[0] = this->colorInterval.low[0];
	colorInterval.low[1] = this->colorInterval.low[1];
	colorInterval.low[2] = this->colorInterval.low[2];

}

