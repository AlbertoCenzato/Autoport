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
	/*
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
	printf("\nSolutions: ");
	printMatrix(solutions, 3, 16);
	getchar();
	*/
	/*
	printf("-------------------------------------------------------\n");
	printf("\nTEST PROGRAM FOR dcm_to_ypr.cpp");
	Eigen::Matrix3d R;
	R << 0.8147,0.9134,0.2785,0.9058,0.6324,0.5469,0.1270,0.0975,0.9575;
	double *ypr = dcm_to_ypr(R);
	printf("\n\nYaw: %f\nPitch: %f\nRoll: %f", ypr[0], ypr[1], ypr[2]);
	getchar();
	*/
	/*
	printf("---------------------------------------------------------\n");
	printf("\nTEST PROGRAM FOR simulazioneCompleta\n");
	simulazioneCompleta();
	*/
	/*
	printf("---------------------------------------------------------\n");
	printf("\nTEST PROGRAM FOR p3p_solver.cpp\n");
	Eigen::Matrix<double, 3, 4> P;
	P << 500, 500,   0, -500,
		-500, 500, 500,  500,
		   0,   0,   0,    0;
	Eigen::Matrix<double, 3, 4> f;
	f << -0.235765117474178,0.236050371549753,0.241941876133016,0,
		0.235765117474178,0.235437865595631,0.000970290259205868,- 0.235754658236770,
		0.942777608328058,0.942788010920269,0.970290259206001,0;
	Eigen::Matrix<double, 3, 4> solution = p3p_solver(P, f);
	printf("\nsolution: ");
	printMatrix(solution, 3, 4);
	getchar();
	*/
	
	printf("---------------------------------------------------------\n");
	printf("\nTEST PROGRAM FOR pinHoleFSolve");
	Eigen::Matrix<double, 6, 1> variables = {};
	
    return 0;
}


