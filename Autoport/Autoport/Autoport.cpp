// Autoport.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdio.h>
#include "..\Eigen\Eigen\Dense"
#include "P3p.h"

using Eigen::Vector3d;
using Eigen::Matrix3d;



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
	ypr[0] = 360 * ypr[0] / 2 / M_PI;
	ypr[1] = 360 * ypr[1] / 2 / M_PI;
	ypr[2] = 360 * ypr[2] / 2 / M_PI;
	return ypr;
}

/* Function che risolve il prp con il metodo di Laurent Kneip
% Input:
%   P - Matrice 3x4 dei 4 punti nello spazio assoluto.Le colonne sono i
%       punti
%   f - Matrice 3x4 dei 4 versori nel sistema camera che individuano le
%       direzioni dei quattro punti fissi.Le colonne sono i versori
% Output :
%   C - Centro del sistema camera nel sistema di riferimento assoluto
%   R - Matrice di rotazione dal sistema assoluto a quello camera */

Eigen::Matrix<double, 3, 4> p3p_solver(Eigen::Matrix<double, 3, 4> &P, Eigen::Matrix<double, 3, 4> &f) {
	//Fixed points at the base station (in millimeters)
	Vector3d P1 = P.col(0);
	Vector3d P2 = P.col(1);
	Vector3d P3 = P.col(2);
	Vector3d P4 = P.col(3);

	//Versori normati nel riferimento della camera
	Vector3d f1 = f.col(0);
	Vector3d f2 = f.col(1);
	Vector3d f3 = f.col(2);
	Vector3d f4 = f.col(3);

	//Primo calcolo
	//Input al codice di Kneip:
	Eigen::Matrix3d wP;
	wP.col(0) = P1
	wP.col(1) = P2
	wP.col(2) = P4; //CHECK IF WORKS LIKE THIS
	Eigen::Matrix3d iV;
	iV.col(0) = f1
	iV.col(1) = f2
	iV.col(2) = f4; //SAME HERE

	//risoluzione del p3p
	P3p p3p;
	Eigen::Matrix<double, 3, 16> poses = p3p.computePoses(iV, wP);	//set n and m dimension
	Vector3d C1 = poses.col(0);
	Vector3d C2 = poses.col(4);
	Vector3d C3 = poses.col(8);
	Vector3d C4 = poses.col(12);
	Matrix3d R1;
	R1.col(0) = poses.col(1);
	R1.col(1) = poses.col(2);
	R1.col(2) = poses.col(3);
	Matrix3d R2;
	R2.col(0) = poses.col(5);
	R2.col(1) = poses.col(6);
	R2.col(2) = poses.col(7);
	Matrix3d R3;
	R3.col(0) = poses.col(9);
	R3.col(1) = poses.col(10);
	R3.col(2) = poses.col(11);
	Matrix3d R4;
	R4.col(0) = poses.col(13);
	R4.col(1) = poses.col(14);
	R4.col(2) = poses.col(15);
	Eigen::Matrix<double, 3, 4> C;
	C.col(0) = C1;
	C.col(1) = C2;
	C.col(2) = C3;
	C.col(3) = C4;
	Eigen::Matrix<double, 3, 12> R;
	R << R1, R2, R3, R4;

	//Discriminazione della soluzione esatta
	Vector3d F31 = (R1*(P3 - C1));
	Vector3d F32 = (R2*(P3 - C2));
	Vector3d F33 = (R3*(P3 - C3));
	Vector3d F34 = (R4*(P3 - C4));
	F31.norm();
	F32.norm();
	F33.norm();
	F34.norm();

	Eigen::Matrix<double, 4, 1> dF;
	dF << (F31 - f3).norm(), (F32 - f3).norm(), (F33 - f3).norm(), (F34 - f3).norm();
	dF = dF.cwiseAbs();

	//find a better algorithm for min finding
	int min = dF(0);
	int index = 0;
	for (int i = 1; i < 4; i++) {
		if (dF(i) < min) {
			min = dF(i);
			index = i;
		}
	}

	Vector3d c = C.col(index);
	Matrix3d r;
	r.col(0) = R.col(3 * index);
	r.col(1) = R.col(3 * index + 1);
	r.col(2) = R.col(3 * index + 2);

	Eigen::Matrix<double, 3, 4> solution;
	solution.col(0) = c;
	solution.col(1) = r.col(0);
	solution.col(2) = r.col(1);
	solution.col(3) = r.col(2);

	//CHECK IF RETURNS BY VALUE OR BY REFERENCE!
	return solution;
}

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


