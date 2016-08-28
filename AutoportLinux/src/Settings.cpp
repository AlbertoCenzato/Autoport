#include "Settings.hpp"

#include <stdlib.h>
#include <string.h>
#include <fstream>
#include "pugixml.hpp"
#include "GenPurpFunc.hpp"

using namespace std;
using namespace pugi;

const char* Settings::HEADER = "AutoportXMLConfigFile";
const char*	Settings::VALUE = "value";
const char* Settings::LOW = "low";
const char* Settings::HIGH = "high";

Interval<int> Settings::hue 	   = Interval<int>(105,135);
Interval<int> Settings::saturation = Interval<int>(150,255);
Interval<int> Settings::value 	   = Interval<int>(  0,255);

int Settings::colorTolerance   = 20;
int Settings::ROITolerance     = 100;
int Settings::sizeTolerance    = 20;
int Settings::sizeSupTolerance = 128;

double Settings::focalX = 3.59;
double Settings::focalY = 3.59;
double Settings::pixelDimension = 1.4e-3;

Settings::Settings() {

}

Settings::~Settings() {
	// TODO Auto-generated destructor stub
}

bool Settings::loadConfiguration(string configFilePath) {

	xml_document doc;
	const char* path = configFilePath.c_str();
	if(!doc.load_file(path))
		cout << "Error loading file " << configFilePath << endl;

	if(strcmp(doc.child(HEADER).name(), HEADER) != 0)
		cout << "Wrong file!" << endl;

	xml_node imageAnalysis		= doc.child(HEADER).child("ImageAnalysis"	  ).child("DefaultValues");
	xml_node positionEstimation = doc.child(HEADER).child("PositionEstimation").child("DefaultValues");

	// load ImageAnalysis settings

	xml_node node = imageAnalysis.child("hue");
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

	node = positionEstimation.child("focal_x");
	focalX = node.attribute(VALUE).as_double();

	node = positionEstimation.child("focal_y");
	focalY = node.attribute(VALUE).as_double();

	node = positionEstimation.child("pixel_dimension");
	pixelDimension = node.attribute(VALUE).as_double();

	return true;

}

string Settings::toString() {
	string config = "HUE " + hue.toString();
	config += "\nSATURATION " + saturation.toString();
	config += "\nVALUE " + value.toString();
	config += "\nCOLOR TOLERANCE " + to_string(colorTolerance);
	config += "\nREGION OF INTEREST TOLERANCE " + to_string(ROITolerance);
	config += "\nSIZE TOLERANCE " + to_string(sizeTolerance);
	config += "\nSIZE SUP TOLERANACE " + to_string(sizeSupTolerance);
	config += "\nFOCAL X " + to_string(focalX);
	config += "\nFOCAL Y " + to_string(focalY);
	config += "\nPIXEL_DIMENSION " + to_string(pixelDimension);
	return config;
}
