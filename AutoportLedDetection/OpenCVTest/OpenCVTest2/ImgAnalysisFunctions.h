#include <math.h>
#include <opencv2/core/core.hpp>

//---Callback functions for sliders in GUI---
extern inline void tbColorCallback(int, void*);
extern inline void tbBlobCallback(int, void*);

//---Private functions---
extern float myDistance(cv::Point2f &, cv::Point2f &);

//---Functions---
extern std::vector<cv::KeyPoint> imgLedDetection(std::string&,cv::Mat&);
extern std::vector<cv::KeyPoint> vidLedDetection(std::string&);

extern std::vector<cv::Point2f> pattern1(std::vector<cv::KeyPoint>&, cv::Mat&);
extern std::vector<cv::Point2f> pattern3(std::vector<cv::KeyPoint>&, cv::Mat&);



