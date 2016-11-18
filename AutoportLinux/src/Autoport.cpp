//============================================================================
// Name        : Autoport.cpp
// Author      : Alberto Cenzato
// Version     : 2.0
// Copyright   : Copyright (c) 2016 Alberto Cenzato. All rights reserved.
// Description : Software for Autoport project
//============================================================================

#include "Settings.hpp"
#include "Test.hpp"

using namespace std;
using namespace cv;

string workingDir;
const string configFileName = "autoport.config";
Status status = Status::LOOKING_FOR_TARGET;
//Settings settings;

int main() {

	cout << "****** AUTOPORT SOFTWARE ******\n" << endl;

	cout << "\n*** Settings ***" << endl;
	Settings *settings = Settings::getInstance();
	cout << settings->toString() << "\n\n" << endl;

	cout << "Enter the path of the file to analyze [d for camera capture]" << endl;
	string path;
	cin >> path;

	Test::ippAnalysis(path);

	getchar();
	return 0;
}

