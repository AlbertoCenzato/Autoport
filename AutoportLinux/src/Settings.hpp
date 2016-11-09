#ifndef SETTINGS_HPP_
#define SETTINGS_HPP_

#include <stdlib.h>

#include "pugixml.hpp"
#include "GenPurpFunc.hpp"

using namespace std;
using namespace pugi;

class Settings {

private:

	string workingDir;
	char *filePath = nullptr;
	xml_document doc;
	bool docOpen = false;

	static Settings *singleton;

	Settings(const string &configFilePath);

public:

	static const char* FILE_NAME;
	static const char* BLUE;
	static const char* RED;
	static const char* HEADER;
	static const char* VALUE;
	static const char* LOW;
	static const char* HIGH;

	LedColor patternColor 	 = LedColor::BLUE;
	Interval<int> hue		 = Interval<int>(105,135);
	Interval<int> saturation = Interval<int>(150,255);
	Interval<int> value		 = Interval<int>(  0,255);

	int colorTolerance   = 20;
	int ROITolerance     = 100;
	int sizeTolerance    = 20;
	int sizeSupTolerance = 128;

	Position_XYZ_YPR initialPosition = Position_XYZ_YPR(0,0,2000,0,0,0);

	// TODO: change default led positions
	vector<Point3f> realWorldPoints = {{90, 70,0},
	                                   {90, 30,0},
	                                   {90,-90,0},
	                                   {50, 70,0},
	                                   {50, 30,0},
	                                   {50,-90,0},
	                                  {-90, 70,0},
	                                  {-90, 90,0}};

	double focalX = 3.59;
	double focalY = 3.59;
	double pixelDimension = 1.4e-3;

	virtual ~Settings();

	static Settings& getInstance();
	//static Settings& getInstance(const string& configFilePath);

	bool loadConfiguration(const string &configFilePath);

	bool modifyConfigParam(const string &paramName, const string &attributeName, double value);

	bool saveConfig();

	string toString();

};

#endif
