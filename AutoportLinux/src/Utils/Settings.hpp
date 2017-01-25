/*==============================================================================
Software for Autoport project

// Copyright   : Copyright (c) 2016, Alberto Cenzato
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the FreeBSD Project.
//============================================================================ */

#ifndef SETTINGS_HPP_
#define SETTINGS_HPP_

#include <opencv2/opencv.hpp>
#include "pugixml.hpp"
#include "GenPurpFunc.hpp"

using namespace std;

class Settings {

private:

	string workingDir;
	char *filePath = nullptr;	// TODO: this static pointer is a bit dangerous,
	pugi::xml_document doc;	 	//	     wrap it in a std::auto_ptr or in a manager class
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
	vector<cv::Point3f> realWorldPoints = {{90, 70,0},
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

	static Settings* getInstance();
	//static Settings& getInstance(const string& configFilePath);

	bool loadConfiguration(const string &configFilePath);

	bool modifyConfigParam(const string &paramName, const string &attributeName, double value);

	bool saveConfig();

	string toString();

};

#endif
