/*
 * XMLReader.h
 *
 *  Created on: Aug 20, 2016
 *      Author: alberto
 */

#pragma once

#include <stdlib.h>
#include "GenPurpFunc.h"

using namespace std;

class Settings {

public:

	static const char* HEADER;

	static Interval<int> hue;
	static Interval<int> saturation;
	static Interval<int> value;

	static int colorTolerance;
	static int ROITolerance;
	static int sizeTolerance;
	static int sizeSupTolerance;

	static double focalX;
	static double focalY;
	static double pixelDimension;


	Settings();
	virtual ~Settings();

	static bool loadConfiguration(string configFilePath);

	static string toString();
};
