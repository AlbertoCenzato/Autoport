//#pragma once
#include "..\Eigen\Eigen\Dense"

using Eigen::Vector2d;
using Eigen::Vector3d;

	/* This function finds the angles (in RADIANS) of the yaw - pitch - roll sequence
	R1(gamma)*R2(beta)*R3(alpha) from the direction cosine matrix
	Q - direction cosine matrix
	yaw - yaw angle(rad)
	pitch - pitch angle(rad)
	roll - roll angle(rad) */

extern double* dcm_to_ypr(Eigen::Matrix3d &R);

	/* Function che risolve il prp con il metodo di Laurent Kneip
	% Input:
	%   P - Matrice 3x4 dei 4 punti nello spazio assoluto.Le colonne sono i
	%       punti
	%   f - Matrice 3x4 dei 4 versori nel sistema camera che individuano le
	%       direzioni dei quattro punti fissi.Le colonne sono i versori
	% Output :
	%   C - Centro del sistema camera nel sistema di riferimento assoluto
	%   R - Matrice di rotazione dal sistema assoluto a quello camera */

extern Eigen::Matrix<double, 3, 4> p3p_solver(Eigen::Matrix<double, 3, 4> &P, Eigen::Matrix<double, 3, 4> &f);


extern int pinHoleFSolve(Eigen::Matrix<double, 6, 1> &variables, Eigen::VectorXd &fvec, double* q_d, double* v, double* PXL1, double* PXL2, double* PXL3, double* PXL4, double* P1_T, double* P2_T, double* P3_T, double* P4_T, double focal, double d_pxl);

extern void printMatrix(Eigen::MatrixXd mtrx, int n, int m);

extern long simulazioneCompleta();