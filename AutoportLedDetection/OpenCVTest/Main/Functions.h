//#pragma once
#include "..\Eigen\Eigen\Dense"

using namespace Eigen;
using namespace cv;
using namespace std;

	/* This function finds the angles (in RADIANS) of the yaw - pitch - roll sequence
	R1(gamma)*R2(beta)*R3(alpha) from the direction cosine matrix
	Q - direction cosine matrix
	yaw - yaw angle(rad)
	pitch - pitch angle(rad)
	roll - roll angle(rad) */

extern void dcm_to_ypr(Matrix3d &R, double* ypr);

/* Function che risolve il prp con il metodo di Laurent Kneip
% Input:
%   P - Matrice 3x4 dei 4 punti nello spazio assoluto.Le colonne sono i
%       punti
%   f - Matrice 3x4 dei 4 versori nel sistema camera che individuano le
%       direzioni dei quattro punti fissi.Le colonne sono i versori
% Output :
%   C - Centro del sistema camera nel sistema di riferimento assoluto
%   R - Matrice di rotazione dal sistema assoluto a quello camera */
extern Matrix<double, 3, 4> p3p_solver(Matrix<double, 3, 4> &P, Matrix<double, 3, 4> &f);

extern long pinHoleFSolve(Matrix<double, 6, 1> &, double *, Point2f *, Point3f *, double focal, double d_pxl);

extern void printMatrix(MatrixXd mtrx, int n, int m);

Point2f multiply2f(Matrix2d &, Point2f &);
Point2d multiply2d(Matrix2d &, Point2d &);
Point3f multiply3f(Matrix3d &, Point3f &);
Point3d multiply3d(Matrix3d &, Point3d &);

Point2f normalize(Point2f &);
Point2d normalize(Point2d &);
Point3f normalize(Point3f &);
Point3d normalize(Point3d &);

Point2f* findMaxXInVec(vector<Point2f> &);
Point2f* findMaxYInVec(vector<Point2f> &);
Point2f* findMinXInVec(vector<Point2f> &);
Point2f* findMinYInVec(vector<Point2f> &);
