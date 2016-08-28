#pragma once

#include <stdlib.h>
#include "GenPurpFunc.hpp"

using namespace std;

class Settings {

public:

	static const char* HEADER;
	static const char* VALUE;
	static const char* LOW;
	static const char* HIGH;

	static Interval<int> hue;
	static Interval<int> saturation;
	static Interval<int> value;

	static int colorTolerance;
	static int ROITolerance;
	static int sizeTolerance;
	static int sizeSupTolerance;

	static Position_XYZ_YPR initialPosition;
	static double focalX;
	static double focalY;
	static double pixelDimension;


	Settings();
	virtual ~Settings();

	static bool loadConfiguration(string configFilePath);

	static string toString();
};
