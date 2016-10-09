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

string Settings::filePath = "";

Interval<int> Settings::hue 	   = Interval<int>(105,135);
Interval<int> Settings::saturation = Interval<int>(150,255);
Interval<int> Settings::value 	   = Interval<int>(  0,255);

int Settings::colorTolerance   = 20;
int Settings::ROITolerance     = 100;
int Settings::sizeTolerance    = 20;
int Settings::sizeSupTolerance = 128;

Position_XYZ_YPR Settings::initialPosition(0,0,2000,0,0,0);
vector<Point3d> Settings::realWorldPoints= {{90, 70,0},
		   	   	   	   	   	   	   	   	    {90, 30,0},
											{90,-90,0},
											{50, 70,0},
											{50, 30,0},
											{50,-90,0},
										   {-90, 70,0},
										   {-90, 90,0}};
double Settings::focalX = 3.59;
double Settings::focalY = 3.59;
double Settings::pixelDimension = 1.4e-3;

Settings::Settings() {

}

Settings::~Settings() {
	// TODO Auto-generated destructor stub
}

bool Settings::loadConfiguration(string &configFilePath) {

	xml_document doc;
	const char* path = configFilePath.c_str();
	if(!doc.load_file(path)) {
		cout << "Error loading file " << configFilePath << endl;
		return false;
	}

	if(strcmp(doc.child(HEADER).name(), HEADER) != 0) {
		cout << "Wrong file!" << endl;
		return false;
	}

	Settings::filePath = configFilePath;

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

	node = positionEstimation.child("initial_position");
	initialPosition = Position_XYZ_YPR(node.attribute("x").as_double(),
									   node.attribute("y").as_double(),
									   node.attribute("z").as_double(),
									   node.attribute("yaw").as_double(),
									   node.attribute("pitch").as_double(),
									   node.attribute("roll").as_double());

	realWorldPoints.empty();
	int count = 0;
	for (xml_node iterNode = positionEstimation.child("real_world_points").first_child(); iterNode; iterNode = iterNode.next_sibling()) {
		double x = iterNode.attribute("x").as_double();
		double y = iterNode.attribute("y").as_double();
		double z = iterNode.attribute("z").as_double();
		realWorldPoints.at(count++) = {x,y,z};
	}

	node = positionEstimation.child("focal_x");
	focalX = node.attribute(VALUE).as_double();

	node = positionEstimation.child("focal_y");
	focalY = node.attribute(VALUE).as_double();

	node = positionEstimation.child("pixel_dimension");
	pixelDimension = node.attribute(VALUE).as_double();

	//FIXME: doc must be closed?

	return true;

}

bool Settings::saveConfigParam(const string &paramName, const string &attributeName, double value) {

	xml_document doc;
	const char* path = Settings::filePath.c_str();
	if(!doc.load_file(path)) {
		cout << "Error loading file " << filePath << endl;
		return false;
	}

	if(strcmp(doc.child(HEADER).name(), HEADER) != 0) {
		cout << "Wrong file!" << endl;
		return false;
	}

	const char* param = paramName.c_str();
	//cout << "Looking for " << param << " node" << endl;
	xml_node node = doc.findChild(param);

	const char* attribute = attributeName.c_str();
	//cout << "Setting " << param << " to value " << value << endl;
	//cout << "Setting " << node.name() << " to value " << value << endl;
	if(!node.attribute(attribute).set_value(value)) {
		cout << "Failed" << endl;
		return false;
	}

	//cout << "Saving file to " << path << endl;
	if(!doc.save_file(path)) {
		cout << "Can't save file " << path << endl;;
		return false;
	}

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
	config += "\nINITIAL POSITION " + initialPosition.toString();
	config += "\nREAL WORLD POINTS " + GenPurpFunc::pointVectorToString(realWorldPoints);
	config += "\nFOCAL X " + to_string(focalX);
	config += "\nFOCAL Y " + to_string(focalY);
	config += "\nPIXEL_DIMENSION " + to_string(pixelDimension);
	return config;
}
