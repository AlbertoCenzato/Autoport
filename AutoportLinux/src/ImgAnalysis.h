//Copyright (c) 2016 Alberto Cenzato. All rights reserved.

#include <opencv2/features2d/features2d.hpp>

namespace ImgAnalysis {
	//---Callback functions for sliders in GUI---
	extern inline void tbColorCallback(int, void*);
	extern inline void tbBlobCallback(int, void*);

	//---Functions---
	cv::Mat filterByColor(cv::Mat &, cv::Scalar &, cv::Scalar &);
	std::vector<cv::Point2f> findBlobs(cv::Mat &, cv::SimpleBlobDetector::Params &);
	extern std::vector<cv::Point2f> imgLedDetection(cv::Mat &,cv::Mat &);

	extern std::vector<cv::Point2f> pattern1(std::vector<cv::Point2f>&, cv::Mat&);
	extern std::vector<cv::Point2f> pattern3(std::vector<cv::Point2f>&, cv::Mat&);
	extern std::vector<cv::Point2f> patternMirko(std::vector<cv::Point2f>&, cv::Mat&, int tolerance);

	//---Inline functions---
	inline float myDistance(Point2f &p1, Point2f &p2) {
		return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
	}

	inline void drawDetectedLed(Mat &image, Point2f &keyPoint, string &number) {
		putText(image, number, keyPoint, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255),4);
		imshow("Thresholded Image", image);
		waitKey(25);
	}

	inline Point2f centroid(vector<Point2f> &points) {
		float x = 0;
		float y = 0;
		for (int i = 0; i < points.size(); i++) {
			Point2f p = points[i];
			x += p.x;
			y += p.y;
		}
		return Point2f(x / points.size(), y / points.size());
	}
}


