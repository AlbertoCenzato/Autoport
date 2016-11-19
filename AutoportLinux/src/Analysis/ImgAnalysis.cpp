//Copyright (c) 2016 Alberto Cenzato. All rights reserved.

#include "ImgAnalysis.hpp"
#include <chrono>

using namespace cv;
using namespace std;

ImgAnalysis::ImgAnalysis(const Interval<Scalar> &colorInterval, LedColor ledColor) {
	constructor(colorInterval, ledColor);
}

ImgAnalysis::ImgAnalysis() {

	Settings *settings = Settings::getInstance();
	Scalar low  = Scalar(settings->hue.min, settings->saturation.min, settings->value.min);
	Scalar high = Scalar(settings->hue.max, settings->saturation.max, settings->value.max);

	//auto patternAnalysis = PatternAnalysis();
	constructor(Interval<Scalar>(low,high), settings->patternColor);
}

void ImgAnalysis::constructor(const Interval<Scalar> &colorInterval, LedColor ledColor) {
	this->colorInterval = colorInterval; //FIXME: what happens here? is it coping the object? Probably yes
	defColorInterval = colorInterval;

	if(ledColor == LedColor::RED) colorConversion = COLOR_RGB2HSV;
	else						  colorConversion = COLOR_BGR2HSV;

	params = SimpleBlobDetector::Params();

	params.filterByColor = true;
	params.blobColor = 255;
	params.filterByArea = false;	//FIXME: set this to true and find default params
	//params.minArea = 0;
	//params.maxArea = 10000;
	params.filterByInertia = false;
	params.filterByConvexity = false;
	params.filterByCircularity = true;
	params.minCircularity = 0.5;
	params.maxCircularity = 1;

	//Settings& settings = Settings::getInstance();
	//colorTolerance   = settings.colorTolerance;
}

bool ImgAnalysis::evaluate(Mat &image, vector<LedDescriptor> &points) {

	//change color space: from BGR to HSV;
	//TODO: color conversion and filterByColor can be performed with a shader (?)
	auto begin = chrono::high_resolution_clock::now();
	cvtColor(image,hsvImage,colorConversion);
	auto end = chrono::high_resolution_clock::now();
	cout << "\nConvert color: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;

	//filter the color according to this->low and this->high tolerances
	begin = chrono::high_resolution_clock::now();
	Mat colorFilteredImg = filterByColor(hsvImage);
	end = chrono::high_resolution_clock::now();
	cout << "\nFilter color: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;

	namedWindow("Filtered image", WINDOW_NORMAL);
	imshow("Filtered image", colorFilteredImg);

	//put in this->points detected blobs that satisfy this->params tolerance
	begin = chrono::high_resolution_clock::now();
	int blobNumber = findBlobs(colorFilteredImg, points);
	end = chrono::high_resolution_clock::now();
	cout << "\nFound " << blobNumber << " blobs: " << chrono::duration_cast<chrono::milliseconds>(end-begin).count() << "ms" << endl;

	waitKey(1);

	return true;
}

ImgAnalysis* ImgAnalysis::setColorInterval(const Interval<Scalar> &colorInterval) {
	this->colorInterval.max[0] = colorInterval.max[0];
	this->colorInterval.max[1] = colorInterval.max[1];
	this->colorInterval.max[2] = colorInterval.max[2];

	this->colorInterval.min[0] = colorInterval.min[0];
	this->colorInterval.min[1] = colorInterval.min[1];
	this->colorInterval.min[2] = colorInterval.min[2];

	return this;
}

ImgAnalysis* ImgAnalysis::setBlobSizeInterval(const Interval<int> &blobSizeInterval) {
	params.minArea = blobSizeInterval.min;
	params.maxArea = blobSizeInterval.max;
	return this;
}

void ImgAnalysis::getColorInterval(Interval<Scalar> &colorInterval) {
	colorInterval.max[0] = this->colorInterval.max[0];
	colorInterval.max[1] = this->colorInterval.max[1];
	colorInterval.max[2] = this->colorInterval.max[2];

	colorInterval.min[0] = this->colorInterval.min[0];
	colorInterval.min[1] = this->colorInterval.min[1];
	colorInterval.min[2] = this->colorInterval.min[2];

}

void ImgAnalysis::resetColorInterval() {
	colorInterval = defColorInterval;
}

// --- private members ---

// Processes the input image (in HSV color space) filtering out (setting to black)
// all colors which are not in the interval [min,max], the others are set to white.
// @img: the Mat object (HSV color space) containing the image to process.
// @min: the lower bound specified in the HSV color space.
// @max: the upper bound specified in the HSV color space.
// returns: black and white image as a Mat object.
Mat ImgAnalysis::filterByColor(const Mat &hsvImg) {

	Mat colorFilteredImg;
	// Sets to white all colors in the threshold interval [min,max] and to black the others
	inRange(hsvImg, colorInterval.min, colorInterval.max, colorFilteredImg);

	//morphological opening (remove small objects from the foreground)
	erode (colorFilteredImg, colorFilteredImg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(colorFilteredImg, colorFilteredImg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//morphological closing (fill small holes in the foreground)
	dilate(colorFilteredImg, colorFilteredImg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode (colorFilteredImg, colorFilteredImg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	return colorFilteredImg;
}

// Finds all color blobs that fit the specified parameters. Blobs which distance
// from the centroid of the blob set is grater than 2*meanDistance are ignored.
// @img: image to analyze.
// @blobParam: parameters to fit.
// returns: a vector of Point2f containing centroids coordinates of detected blobs.
int ImgAnalysis::findBlobs(const Mat &colorFilteredImg, vector<LedDescriptor>& ledPoints) {

	//TODO: check this way of computing valid led sizes interval: it can lead to
	//a degeneration of the interval amplitude continuously increasing it in presence
	//of noise similar to leds, maybe it would be better to use the medium value of led sizes

	ledPoints.clear();
	vector<KeyPoint> keyPoints(10);

	Ptr<SimpleBlobDetector> featureDetector = SimpleBlobDetector::create(params);
	featureDetector->detect(colorFilteredImg, keyPoints);

	const int SIZE = keyPoints.size();
	cout << "KeyPoints SIZE: " << SIZE << endl;
	if(SIZE > 0) {

		for(int i = 0; i < SIZE; ++i) {
			Point2f position = keyPoints[i].pt;
			Scalar  color 	 = hsvImage.at<Vec3b>(position);
			ledPoints.push_back(LedDescriptor(position,color,keyPoints[i].size));
		}

		// Remove points too far from the centroid of the detected points set
		// compute the mean distance from the centroid
		Point2f centr = LedDescriptor::centroid(ledPoints);
		float meanDist = 0;
		float *distances = new float[SIZE];	// TODO: try to use another way, dangerous pointer
		for (int i = 0; i < SIZE; i++) {
			float dist = GenPurpFunc::distPoint2Point(centr, ledPoints[i].position);
			meanDist += dist;
			distances[i] = dist;
		}
		meanDist = meanDist / SIZE;

		// remove points too far away points
		int size = SIZE;
		for (int i = 0; i < size; ) {
			if (distances[i] > 2 * meanDist) {
				ledPoints[i] = ledPoints[size - 1];
				distances[i] = distances[size - 1];
				ledPoints.pop_back();
			}
			else i++;
			size = ledPoints.size();
		}

		delete [] distances;
	}

	return ledPoints.size();
}

