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
#include "../../extern_libs/pugixml.hpp"
#include "GenPurpFunc.hpp"

/**
 * This class loads all configurable project settings from an xml file
 * and stores them in its member variables. It can also save
 * modified settings to a file. To change permanently the parameters
 * use Settings::modifyConfigParam(), don't use member variables, changes made
 * to member variables are not written to file.
 *
 * FIXME: it's ambiguous to have public member variables and
 * 		 Settings::modifyConfigParam() function
 * FIXME: this class should be split in multiple inner classes
 * 		  following the hierarchy of the xml configuration file
 */
class Settings {

private:

	std::string workingDir;
	char *filePath = nullptr;
	pugi::xml_document doc;
	bool docOpen = false;

	static Settings *singleton; // TODO: this static pointer is a bit dangerous,
                                //	     wrap it in a std::auto_ptr or in a manager class

	/**
	 * Private class constructor.
	 */
	Settings(const std::string &configFilePath);

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
	std::vector<cv::Point3f> realWorldPoints = {{90, 70,0},
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

	/**
	 * This function is the only way to instantiate an object of this class.
	 * Projects settings are loaded from file the first time this function is called
	 * and kept in memory until the execution ends; therefore subsequent calls to
	 * this function do not open the file again, they simply and quickly return a pointer.
	 *
	 * @return: a pointer to the static Settings object.
	 */
	static Settings* getInstance();

	/**
	 * Function called by the constructor to load configuration
	 * settings from file.
	 *
	 * @configFilePath: path to configuration file
	 * @return: false if errors occur
	 */
	bool loadConfiguration(const std::string &configFilePath);

	/**
	 * Change value to a parameter.
	 *
	 * @paramName: name of the parameter to change.
	 * @attributeName: name of the attribute of the parameter to change.
	 * @value: new attribute value.
	 * @return: false if errors occur
	 */
	bool modifyConfigParam(const std::string &paramName, const std::string &attributeName, double value);

	/**
	 * Saves changes done by Settings::modifyConfigParam()
	 *
	 * @return: false if errors occur
	 */
	bool saveConfig();

	/**
	 * Returns a string containing parameters values
	 */
	std::string toString();

};

#endif
