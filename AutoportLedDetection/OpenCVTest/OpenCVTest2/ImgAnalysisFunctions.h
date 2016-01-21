//#include <math.h>

//---Callback functions for sliders in GUI---
extern inline void tbColorCallback(int, void*);
extern inline void tbBlobCallback(int, void*);

//---Private functions---
extern float myDistance(cv::Point2f &, cv::Point2f &);

//---Functions---
extern std::vector<cv::Point2f> imgLedDetection(cv::Mat,cv::Mat);

extern std::vector<cv::Point2f> pattern1(std::vector<cv::Point2f>&, cv::Mat&);
extern std::vector<cv::Point2f> pattern3(std::vector<cv::Point2f>&, cv::Mat&);
extern std::vector<cv::Point2f> patternMirko(std::vector<cv::Point2f>&, cv::Mat&, int tolerance);



