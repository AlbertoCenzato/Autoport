// Autoport.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdio.h>
#include "..\Eigen\Eigen\Dense"
#include "P3p.h"


/*
This function finds the angles (in RADIANS) of the yaw - pitch - roll sequence
R1(gamma)*R2(beta)*R3(alpha) from the direction cosine matrix
Q - direction cosine matrix
yaw - yaw angle(rad)
pitch - pitch angle(rad)
roll - roll angle(rad)
*/

inline double* dcm_to_ypr(Eigen::Matrix3d &R) {
	double ypr[3];
	ypr[0] = atan(R(0, 1) / R(0, 0));
	ypr[1] = asin(-R(0, 2));
	ypr[2] = atan(R(1, 2) / R(2, 2));
	return ypr;
}


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

	printf("-------------------------------------------------------\n");
	printf("\nTEST PROGRAM FOR dcm_to_ypr.cpp");
	Eigen::Matrix3d R;
	R << 0.7922, 0.0357, 0.6787, 0.9595, 0.8491, 0.7577, 0.6557, 0.9340, 0.7431;
	double *ypr = dcm_to_ypr(R);
	printf("\n\nYaw: %f\nPitch: %f\nRoll: %f", ypr[0], ypr[1], ypr[2]);
	getchar();
    return 0;
}


