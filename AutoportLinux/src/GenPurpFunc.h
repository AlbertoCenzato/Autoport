//Copyright (c) 2016 Alberto Cenzato. All rights reserved.

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

using namespace Eigen;
using namespace cv;
using namespace std;

namespace GenPurpFunc {
	/* This function finds the angles (in RADIANS) of the yaw - pitch - roll sequence
	R1(gamma)*R2(beta)*R3(alpha) from the direction cosine matrix
	Q - direction cosine matrix
	yaw - yaw angle(rad)
	pitch - pitch angle(rad)
	roll - roll angle(rad) */
	inline void dcm_to_ypr(Matrix3d &R, double* ypr) {
		ypr[0] = atan2((float)R(0, 1) , (float)R(0, 0));
		ypr[1] = asin((float)-R(0, 2));
		ypr[2] = atan2((float)R(1, 2) , (float)R(2, 2));
		ypr[0] = 180 * ypr[0] / M_PI;
		ypr[1] = 180 * ypr[1] / M_PI;
		ypr[2] = 180 * ypr[2] / M_PI;
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
	extern Matrix<double, 3, 4> p3p_solver(Matrix<double, 3, 4> &P, Matrix<double, 3, 4> &f);

	extern long pinHoleFSolve(Matrix<double, 6, 1> &, double *, Point2f *, Point3f *, double focal, double d_pxl);

	inline void printMatrix(MatrixXd mtrx, int n, int m) {
		for (int i = 0; i < n; i++) {
			printf("\n");
			for (int j = 0; j < m; j++) {
				printf("%f ", mtrx(i, j));
			}
		}
		printf("\n");
	}

	inline Point2f multiply2f(Matrix2d &R, Point2f &point) {
		Point2f ret;
		ret.x = R.row(0)[0] * point.x + R.row(0)[1] * point.y;
		ret.y = R.row(1)[0] * point.x + R.row(1)[1] * point.y;
		return ret;
	}
	inline Point2d multiply2d(Matrix2d &R, Point2d &point) {
		Point2d ret;
		ret.x = R.row(0)[0] * point.x + R.row(0)[1] * point.y;
		ret.y = R.row(1)[0] * point.x + R.row(1)[1] * point.y;
		return ret;
	}
	inline Point3f multiply3f(Matrix3d &R, Point3f &point) {
		Point3f ret;
		ret.x = R.row(0)[0] * point.x + R.row(0)[1] * point.y + R.row(0)[2] * point.z;
		ret.y = R.row(1)[0] * point.x + R.row(1)[1] * point.y + R.row(1)[2] * point.z;
		ret.z = R.row(2)[0] * point.x + R.row(2)[1] * point.y + R.row(2)[2] * point.z;
		return ret;
	}
	inline Point3d multiply3d(Matrix3d &R, Point3d &point) {
		Point3d ret;
		ret.x = R.row(0)[0] * point.x + R.row(0)[1] * point.y + R.row(0)[2] * point.z;
		ret.y = R.row(1)[0] * point.x + R.row(1)[1] * point.y + R.row(1)[2] * point.z;
		ret.z = R.row(2)[0] * point.x + R.row(2)[1] * point.y + R.row(2)[2] * point.z;
		return ret;
	}

	inline Point2f normalize(Point2f &p) {
		double norm = sqrt(pow(p.x, 2) + pow(p.y, 2));
		return Point2f(p.x / norm, p.y / norm);
	}
	inline Point2d normalize(Point2d &p) {
		double norm = sqrt(pow(p.x, 2) + pow(p.y, 2));
		return Point2d(p.x / norm, p.y / norm);
	}
	inline Point3f normalize(Point3f &p) {
		double norm = sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z,2));
		return Point3f(p.x / norm, p.y / norm, p.z/norm);
	}
	inline Point3d normalize(Point3d &p) {
		double norm = sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));
		return Point3d(p.x / norm, p.y / norm, p.z / norm);
	}

	inline Point2f* findMaxXInVec(vector<Point2f> &vec) {
		Point2f *max = &vec[0];
		for (uint i = 1; i < vec.size(); i++)
			if (max->x < vec[i].x)
				max = &vec[i];
		return max;
	}
	inline Point2f* findMaxYInVec(vector<Point2f> &vec) {
		Point2f *max = &vec[0];;
		for (uint i = 1; i < vec.size(); i++)
			if (max->y < vec[i].y)
				max = &vec[i];
		return max;
	}
	inline Point2f* findMinXInVec(vector<Point2f> &vec) {
		Point2f *min = &vec[0];;
		for (uint i = 1; i < vec.size(); i++)
			if (min->x > vec[i].x)
				min = &vec[i];
		return min;
	}
	inline Point2f* findMinYInVec(vector<Point2f> &vec) {
		Point2f *min = &vec[0];;
		for (uint i = 1; i < vec.size(); i++)
			if (min->y > vec[i].y)
				min = &vec[i];
		return min;
	}
}