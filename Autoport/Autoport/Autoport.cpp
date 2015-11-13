// Autoport.cpp : Defines the entry point for the console application.
// Test program for computetions and algorithms of Autoport project

#include "stdafx.h"
#include <stdio.h>
#include "..\Eigen\Eigen\Dense"
#include "P3p.h"
#include "Functions.h"

using Eigen::Vector3d;
using Eigen::Matrix3d;

int main()
{
	printf("Test program for modified P3p.cpp");

	double focal = 3.46031; // Focale camera[mm]

	Vector3d f1 = {  0.08650775,  0.08650775, focal };
	Vector3d f2 = {  0.08650775, -0.08650775, focal };
	Vector3d f3 = { -0.08650775, -0.08650775, focal };
	f1.normalize();
	f2.normalize();
	f3.normalize();

	Matrix3d featureVectors;
	featureVectors.col(0) = f1;
	featureVectors.col(1) = f2;
	featureVectors.col(2) = f3;

	Matrix3d worldPoints;
	worldPoints.col(0) <<  50,  50, 0;
	worldPoints.col(1) << -50,  50, 0;
	worldPoints.col(2) << -50, -50, 0;

	Eigen::Matrix<double, 3, 16> solutions;
	P3p p3p;
	solutions = p3p.computePoses(featureVectors, worldPoints);
	for (int i = 0; i < 3; i++) {
		printf("\n");
		for (int j = 0; j < 16; j++)
			printf("%f\n", solutions(i, j));
	}
	getchar();

	printf("-------------------------------------------------------\n");
	printf("\nTEST PROGRAM FOR dcm_to_ypr.cpp");
	Eigen::Matrix3d R;
	R << 0.8147,0.9134,0.2785,0.9058,0.6324,0.5469,0.1270,0.0975,0.9575;
	double *ypr = dcm_to_ypr(R);
	printf("\n\nYaw: %f\nPitch: %f\nRoll: %f", ypr[0], ypr[1], ypr[2]);
	getchar();

	printf("---------------------------------------------------------\n");
	printf("\nTEST PROGRAM FOR p3p_solver.cpp");
	Eigen::Matrix<double, 3, 4> P;
	P << 0.1626 ,   0.9597 ,   0.2238  ,  0.5060,
		0.1190  ,  0.3404  ,  0.7513   , 0.6991 ,
		0.4984  ,  0.5853  ,  0.2551   , 0.8909;
	Eigen::Matrix<double, 3, 4> f;
	f << 0.3816, 0.1869, 0.6463, 0.2760,
		0.7655, 0.4898, 0.7094, 0.6797,
		0.7952, 0.4456, 0.7547, 0.6551;
	Eigen::Matrix<double, 3, 4> solution = p3p_solver(P, f);
	
	for (int i = 0; i < 3; i++) {
		printf("\n");
		for (int j = 0; j < 4; j++)
			printf("%f ", solution(i, j));
	}
	getchar();

    return 0;
}


