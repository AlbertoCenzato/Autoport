#include "stdafx.h"
#include <stdio.h>
#include <Eigen\Dense>
#include "..\Eigen\unsupported\Eigen\NonLinearOptimization"
#include "..\Eigen\unsupported\Eigen\NumericalDiff"
#include "P3p.h"
#include "Functions.h"

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector2d;

/* This function finds the angles (in RADIANS) of the yaw - pitch - roll sequence
R1(gamma)*R2(beta)*R3(alpha) from the direction cosine matrix
Q - direction cosine matrix
yaw - yaw angle(rad)
pitch - pitch angle(rad)
roll - roll angle(rad) */
inline double* dcm_to_ypr(Matrix3d &R) {
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
	Matrix3d wP;
	wP.col(0) = P1;
	wP.col(1) = P2;
	wP.col(2) = P4; //CHECK IF WORKS LIKE THIS
	Matrix3d iV;
	iV.col(0) = f1;
	iV.col(1) = f2;
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
	double min = dF(0);
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


// Generic functor
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
	typedef _Scalar Scalar;
	enum {
		InputsAtCompileTime = NX,
		ValuesAtCompileTime = NY
	};
	typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

	int m_inputs, m_values;

	Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
	Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

	int inputs() const { return m_inputs; }
	int values() const { return m_values; }
};


struct PinHoleEquations : Functor<double> /*TODO: inputs and values missing! Add them!*/{
	PinHoleEquations(double* q_d, double* v, double* PXL1, double* PXL2, double* PXL3, double* PXL4, double* P1_T, double* P2_T, double* P3_T, double* P4_T, double focal, double d_pxl)
		: q_d(q_d), v(v), PXL1(PXL1), PXL2(PXL2), PXL3(PXL3), PXL4(PXL4), P1_T(P1_T), P2_T(P2_T), P3_T(P3_T), P4_T(P4_T) {}

	int operator()(Eigen::VectorXd &variables, Eigen::VectorXd &fvec) const {

		double x = variables(0);
		double Y = variables(1);
		double Z = variables(2);
		double Yaw = variables(3);
		double Pitch = variables(4);
		double Roll = variables(5);

		double cosYaw = cos(Yaw);
		double sinYaw = sin(Yaw);
		double cosPitch = cos(Pitch);
		double sinPitch = sin(Pitch);
		double cosRoll = cos(Roll);
		double sinRoll = sin(Roll);

		//#define aaa(a,b)  (cos(a)*sin(b)+a*a)

		double x_pxl_1 = PXL1[0];
		double x_pxl_2 = PXL2[0];
		double x_pxl_3 = PXL3[0];
		double x_pxl_4 = PXL4[0];
						 	 
		double y_pxl_1 = PXL1[1];
		double y_pxl_2 = PXL2[1];
		double y_pxl_3 = PXL3[1];
		double y_pxl_4 = PXL4[1];

		double Px_1 = P1_T[0];
		double Px_2 = P2_T[0];
		double Px_3 = P3_T[0];
		double Px_4 = P4_T[0];
					  	  
		double Py_1 = P1_T[1];
		double Py_2 = P2_T[1];
		double Py_3 = P3_T[1];
		double Py_4 = P4_T[1];
					  	  
		double Pz_1 = P1_T[2];
		double Pz_2 = P2_T[2];
		double Pz_3 = P3_T[2];
		double Pz_4 = P4_T[2];

		double vx = v[0];
		double vy = v[1];
		double vz = v[2];
					
		double qx = q_d[0];
		double qy = q_d[1];
		double qz = q_d[2];

		fvec(0) = x_pxl_1 - (focal*(cosPitch*cosYaw*(Px_1 - x) - sinPitch*(Pz_1 - Z) + cosPitch*sinYaw*(Py_1 - Y))) / (d_pxl*((Px_1 - x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_1 - Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_1 - Z)));
		fvec(2) = x_pxl_2 - (focal*(cosPitch*cosYaw*(Px_2 - x) - sinPitch*(Pz_2 - Z) + cosPitch*sinYaw*(Py_2 - Y))) / (d_pxl*((Px_2 - x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_2 - Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_2 - Z)));
		fvec(4) = x_pxl_3 - (focal*(cosPitch*cosYaw*(Px_3 - x) - sinPitch*(Pz_3 - Z) + cosPitch*sinYaw*(Py_3 - Y))) / (d_pxl*((Px_3 - x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_3 - Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_3 - Z)));
		fvec(6) = x_pxl_4 - (focal*(cosPitch*cosYaw*(Px_4 - x) - sinPitch*(Pz_4 - Z) + cosPitch*sinYaw*(Py_4 - Y))) / (d_pxl*((Px_4 - x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_4 - Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_4 - Z)));

		fvec(1) = y_pxl_1 - (focal*((Py_1 - Y)*(cosRoll*cosYaw + sinPitch*sinRoll*sinYaw) - (Px_1 - x)*(cosRoll*sinYaw - cosYaw*sinPitch*sinRoll) + cosPitch*sinRoll*(Pz_1 - Z))) / (d_pxl*((Px_1 - x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_1 - Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_1 - Z)));
		fvec(3) = y_pxl_2 - (focal*((Py_2 - Y)*(cosRoll*cosYaw + sinPitch*sinRoll*sinYaw) - (Px_2 - x)*(cosRoll*sinYaw - cosYaw*sinPitch*sinRoll) + cosPitch*sinRoll*(Pz_2 - Z))) / (d_pxl*((Px_2 - x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_2 - Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_2 - Z)));
		fvec(5) = y_pxl_3 - (focal*((Py_3 - Y)*(cosRoll*cosYaw + sinPitch*sinRoll*sinYaw) - (Px_3 - x)*(cosRoll*sinYaw - cosYaw*sinPitch*sinRoll) + cosPitch*sinRoll*(Pz_3 - Z))) / (d_pxl*((Px_3 - x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_3 - Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_3 - Z)));
		fvec(7) = y_pxl_4 - (focal*((Py_4 - Y)*(cosRoll*cosYaw + sinPitch*sinRoll*sinYaw) - (Px_4 - x)*(cosRoll*sinYaw - cosYaw*sinPitch*sinRoll) + cosPitch*sinRoll*(Pz_4 - Z))) / (d_pxl*((Px_4 - x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_4 - Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_4 - Z)));

		//TODO: change variable name, it sucks
		double denominator = sqrt(pow(abs(qx - Z*sinPitch + x*cosPitch*cosYaw + Y*cosPitch*sinYaw), 2) + pow(abs(qz + x*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - Y*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + Z*cosPitch*cosRoll), 2) + pow(abs(qy - x*(cosRoll*sinYaw - cosYaw*sinPitch*sinRoll) + Y*(cosRoll*cosYaw + sinPitch*sinRoll*sinYaw) + Z*cosPitch*sinRoll), 2));

		fvec(8) = vx + (qx - Z*sinPitch + x*cosPitch*cosYaw + Y*cosPitch*sinYaw) / denominator;
		fvec(9) = vy + (qy - x*(cosRoll*sinYaw - cosYaw*sinPitch*sinRoll) + Y*(cosRoll*cosYaw + sinPitch*sinRoll*sinYaw) + Z*cosPitch*sinRoll) / denominator;
		fvec(10) = vz + (qz + x*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - Y*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + Z*cosPitch*cosRoll) / denominator;

		return 1;
	}

	double *q_d;
	double *v;
	double *PXL1;
	double *PXL2;
	double *PXL3;
	double *PXL4;
	double *P1_T;
	double *P2_T;
	double *P3_T;
	double *P4_T;
	double focal;
	double d_pxl;
};


//TODO: fix the size of VectorXd variables
int pinHoleFSolve(Eigen::VectorXd &variables, Eigen::VectorXd &fvec, double* q_d, double* v, double* PXL1, double* PXL2, double* PXL3, double* PXL4, double* P1_T, double* P2_T, double* P3_T, double* P4_T, double focal, double d_pxl) {

	PinHoleEquations pinHoleFunctor(q_d, v, PXL1, PXL2, PXL3, PXL4, P1_T, P2_T, P3_T, P4_T, focal, d_pxl);
	Eigen::NumericalDiff<PinHoleEquations> numDiff(pinHoleFunctor);
	Eigen::LevenbergMarquardt<Eigen::NumericalDiff<PinHoleEquations>, double> levMarq(numDiff);
	//levMarq.parameters.maxfev = 2000;
	//levMarq.parameters.xtol = 1.0e-10;
	return levMarq.minimize(variables);
}
