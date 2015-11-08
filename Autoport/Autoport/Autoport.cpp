// Autoport.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdio.h>
#include <Eigen/Eigen/Dense>
#include "P3p.h"

int main()
{
	printf("Test program for modified P3p.cpp");

	double focal = 3.46031; // Focale camera[mm]

	Eigen::Vector3d f1 = {  0.08650775,  0.08650775, focal };
	Eigen::Vector3d f2 = {  0.08650775, -0.08650775, focal };
	Eigen::Vector3d f3 = { -0.08650775, -0.08650775, focal };
	f1.normalize();
	f2.normalize();
	f3.normalize();

	Eigen::Matrix3d featureVectors;
	featureVectors.col(0) = f1;
	featureVectors.col(1) = f2;
	featureVectors.col(2) = f3;

	Eigen::Matrix3d worldPoints;
	worldPoints.col(0) <<  50,  50, 0;
	worldPoints.col(1) << -50,  50, 0;
	worldPoints.col(2) << -50, -50, 0;

	Eigen::Matrix<double, 3, 16> solutions = Eigen::MatrixXd::Zero(3,16);
	
	P3p p3p;
	p3p.computePoses(featureVectors, worldPoints, solutions);
	for (int i = 0; i < 3; i++) {
		printf("\n");
		for (int j = 0; j < 16; j++)
			printf("%f\n", solutions(i, j));
	}
	getchar();
    return 0;
}

