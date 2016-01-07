#include <math.h>
#include <opencv2/core/core.hpp>

//---Callback functions for sliders in GUI---
extern inline void tbColorCallback(int, void*);
extern inline void tbBlobCallback(int, void*);

//---Private functions---
extern float myDistance(cv::KeyPoint*, cv::KeyPoint*);

//---Functions---
extern std::vector<cv::KeyPoint> imgLedDetection(std::string&);
extern std::vector<cv::KeyPoint> vidLedDetection(std::string&);

extern std::vector<cv::KeyPoint> pattern1(std::vector<cv::KeyPoint>&);



