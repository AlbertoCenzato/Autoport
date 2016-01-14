//#pragma once
#include "..\Eigen\Eigen\Dense"

using namespace Eigen;
using namespace cv;

	/* This function finds the angles (in RADIANS) of the yaw - pitch - roll sequence
	R1(gamma)*R2(beta)*R3(alpha) from the direction cosine matrix
	Q - direction cosine matrix
	yaw - yaw angle(rad)
	pitch - pitch angle(rad)
	roll - roll angle(rad) */

extern void dcm_to_ypr(Eigen::Matrix3d &R, double* ypr);

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

extern long pinHoleFSolve(Eigen::Matrix<double, 6, 1> &, double *, cv::Point2f *, cv::Point3f *, double focal, double d_pxl);

extern void printMatrix(Eigen::MatrixXd mtrx, int n, int m);

cv::Point2f multiply2f(Matrix2d &, cv::Point2f &);
cv::Point2d multiply2d(Matrix2d &, cv::Point2d &);
cv::Point3f multiply3f(Matrix3d &, cv::Point3f &);
cv::Point3d multiply3d(Matrix3d &, cv::Point3d &);

cv::Point2f normalize(cv::Point2f &);
cv::Point2d normalize(cv::Point2d &);
cv::Point3f normalize(cv::Point3f &);
cv::Point3d normalize(cv::Point3d &);