
#include <math.h>
#include <opencv2/core/core.hpp>

extern std::vector<cv::KeyPoint> imgLedDetection(std::string &);
extern std::vector<cv::KeyPoint> vidLedDetection(std::string &);

extern std::vector<POINT> pattern1(std::vector<cv::KeyPoint> &);