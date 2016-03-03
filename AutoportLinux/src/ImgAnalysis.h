//Copyright (c) 2016 Alberto Cenzato. All rights reserved.

#include <opencv2/features2d/features2d.hpp>

namespace ImgAnalysis {
	//---Callback functions for sliders in GUI---
	extern inline void tbColorCallback(int, void*);
	extern inline void tbBlobCallback(int, void*);

	//---Private functions---
	extern float myDistance(cv::Point2f &, cv::Point2f &);

	//---Functions---
	cv::Mat filterByColor(cv::Mat &, cv::Scalar &, cv::Scalar &);
	std::vector<cv::Point2f> findBlobs(cv::Mat &, cv::SimpleBlobDetector::Params &);
	extern std::vector<cv::Point2f> imgLedDetection(cv::Mat &,cv::Mat &);

	extern std::vector<cv::Point2f> pattern1(std::vector<cv::Point2f>&, cv::Mat&);
	extern std::vector<cv::Point2f> pattern3(std::vector<cv::Point2f>&, cv::Mat&);
	extern std::vector<cv::Point2f> patternMirko(std::vector<cv::Point2f>&, cv::Mat&, int tolerance);
}


