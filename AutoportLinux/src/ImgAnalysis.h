//Copyright (c) 2016 Alberto Cenzato. All rights reserved.

#include <iostream>
#include <set>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;
using namespace cv;

#ifndef IMGANALYSIS_H
#define IMGANALISIS_H

class ImgAnalysis {

public:

	static Mat filterByColor(Mat &img, Scalar &, Scalar &);
	static vector<Point2f> findBlobs(Mat &img, SimpleBlobDetector::Params &);
	static vector<Point2f> pattern1(vector<Point2f> &, Mat &);
	static vector<Point2f> pattern3(vector<Point2f> &, Mat &);
	static vector<Point2f> patternMirko(vector<Point2f> &, Mat &, int);

private:

	static inline float distancePointToPoint(Point2f &p1, Point2f &p2) {
		return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
	}

	static inline void drawDetectedLed(Mat &image, Point2f &keyPoint, string &number) {
		putText(image, number, keyPoint, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255),4);
		imshow("Thresholded Image", image);
		waitKey(25);
	}

	static inline Point2f centroid(vector<Point2f> &points) {
		float x = 0;
		float y = 0;
		for (uint i = 0; i < points.size(); i++) {
			Point2f p = points[i];
			x += p.x;
			y += p.y;
		}
		return Point2f(x / points.size(), y / points.size());
	}

};

#endif
