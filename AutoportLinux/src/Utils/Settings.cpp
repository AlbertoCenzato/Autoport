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

#include "Settings.hpp"

#include <string.h>
#include <fstream>
#include <algorithm>
#include <unistd.h>
#include <stdlib.h>

using namespace pugi;

Settings* Settings::singleton = nullptr;

const char* Settings::FILE_NAME = "autoport.config";

const char* Settings::BLUE 	 = "BLUE";
const char* Settings::RED    = "RED";
const char* Settings::HEADER = "AutoportXMLConfigFile";
const char*	Settings::VALUE  = "value";
const char* Settings::LOW 	 = "low";
const char* Settings::HIGH 	 = "high";

Settings::Settings(const string &configFilePath) {
		loadConfiguration(configFilePath);
	}

Settings::~Settings() {
	delete [] filePath;
}

Settings* Settings::getInstance() {
	if(singleton == nullptr) {
		const int bufferSize = 500;
		char workDir[bufferSize];
		getcwd(workDir,bufferSize);
		string configFilePath(workDir);
		string fileName(FILE_NAME);
		configFilePath += "/" + fileName;
		singleton = new Settings(configFilePath);
		singleton->workingDir = workDir;
	}
	return singleton;
}

bool Settings::loadConfiguration(const string &configFilePath) {

	const char* path = configFilePath.c_str();
	if(!doc.load_file(path)) {
		cerr << "Error loading file " << configFilePath << endl;
		return false;
	}

	docOpen = true;

	if(strcmp(doc.child(HEADER).name(), HEADER) != 0) {
		cerr << "Wrong file! The header is not \"" << HEADER << "\"" << endl;
		return false;
	}

	filePath = new char[configFilePath.size()];
	configFilePath.copy(filePath, 0, configFilePath.size());

	xml_node imageAnalysis		= doc.child(HEADER).child("ImageAnalysis"	  ).child("DefaultValues");
	xml_node positionEstimation = doc.child(HEADER).child("PositionEstimation").child("DefaultValues");

	// load ImageAnalysis settings

	xml_node node = imageAnalysis.child("pattern_color");
	string color = node.attribute(VALUE).as_string();
	transform(color.begin(), color.end(), color.begin(), ::toupper);
	patternColor = color.compare(string(RED)) == 0 ? LedColor::RED : LedColor::BLUE;

	node = imageAnalysis.child("hue");
	int low  = node.attribute(LOW).as_int();
	int high = node.attribute(HIGH).as_int();
	hue = Interval<int>(low, high);

	node = imageAnalysis.child("saturation");
	low  = node.attribute(LOW).as_int();
	high = node.attribute(HIGH).as_int();
	saturation = Interval<int>(low, high);

	node = imageAnalysis.child(VALUE);
	low  = node.attribute(LOW).as_int();
	high = node.attribute(HIGH).as_int();
	value = Interval<int>(low, high);

	node = imageAnalysis.child("color_tolerance");
	colorTolerance = node.attribute(VALUE).as_int();

	node = imageAnalysis.child("roi_tolerance");
	ROITolerance = node.attribute(VALUE).as_int();

	node = imageAnalysis.child("size_tolerance");
	sizeTolerance = node.attribute(VALUE).as_int();

	node = imageAnalysis.child("size_sup_tolerance");
	sizeSupTolerance = node.attribute(VALUE).as_int();

	// load default PositionEstimation settings

	node = positionEstimation.child("initial_position");
	initialPosition = Position_XYZ_YPR(node.attribute("x").as_double(),
									   node.attribute("y").as_double(),
									   node.attribute("z").as_double(),
									   node.attribute("yaw").as_double(),
									   node.attribute("pitch").as_double(),
									   node.attribute("roll").as_double());

	realWorldPoints.clear();
	for (xml_node iterNode = positionEstimation.child("real_world_points").first_child(); iterNode; iterNode = iterNode.next_sibling()) {
		float x = iterNode.attribute("x").as_float();
		float y = iterNode.attribute("y").as_float();
		float z = iterNode.attribute("z").as_float();
		realWorldPoints.push_back({x,y,z});
	}

	node = positionEstimation.child("focal_x");
	focalX = node.attribute(VALUE).as_double();

	node = positionEstimation.child("focal_y");
	focalY = node.attribute(VALUE).as_double();

	node = positionEstimation.child("pixel_dimension");
	pixelDimension = node.attribute(VALUE).as_double();

	//TODO: doc must be closed? don't think so...

	return true;

}

bool Settings::modifyConfigParam(const string &paramName, const string &attributeName, double value) {

	if(!docOpen) {
		if(!doc.load_file(filePath)) {
			cerr << "Error loading file " << filePath << endl;
			return false;
		}
		docOpen = true;
	}

	if(strcmp(doc.child(HEADER).name(), HEADER) != 0) {
		cerr << "Wrong file!" << endl;
		return false;
	}

	const char* param = paramName.c_str();
	xml_node node = doc.findChild(param);

	const char* attribute = attributeName.c_str();
	if(!node.attribute(attribute).set_value(value)) {
		cerr << "Failed in modifying " << attribute << " to " << value << endl;
		return false;
	}

	return true;
}

bool Settings::saveConfig() {
	if(!docOpen) {
		cerr << "Trying to save a closed document!" << endl;
		return false;
	}

	if(!doc.save_file(filePath)) {
		cerr << "Can't save file " << filePath << endl;;
		return false;
	}

	return doc.save_file(FILE_NAME);;
}

string Settings::toString() {
	string config = "COLOR " + (patternColor == LedColor::RED ? string(RED) : string(BLUE));
	config += "\nHUE " + hue.toString();
	config += "\nSATURATION " + saturation.toString();
	config += "\nVALUE " + value.toString();
	config += "\nCOLOR TOLERANCE " + to_string(colorTolerance);
	config += "\nREGION OF INTEREST TOLERANCE " + to_string(ROITolerance);
	config += "\nSIZE TOLERANCE " + to_string(sizeTolerance);
	config += "\nSIZE SUP TOLERANACE " + to_string(sizeSupTolerance);
	config += "\nINITIAL POSITION " + initialPosition.toString();
	config += "\nREAL WORLD POINTS " + GenPurpFunc::pointVectorToString(realWorldPoints);
	config += "\nFOCAL X " + to_string(focalX);
	config += "\nFOCAL Y " + to_string(focalY);
	config += "\nPIXEL_DIMENSION " + to_string(pixelDimension);
	return config;
}
