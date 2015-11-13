#include "stdafx.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <complex>
#include "../Eigen/Eigen/Dense"
#include "../Eigen/unsupported/Eigen/NonLinearOptimization"
#include "../Eigen/unsupported/Eigen/NumericalDiff"

using Eigen::Vector3d;
using Eigen::Vector2d;

double* pinHoleFSolve(Eigen::Matrix<double, 6, 1> variables, Vector3d q_d, Vector3d v, Vector2d PXL1, Vector2d PXL2, Vector2d PXL3, Vector2d PXL4, Vector3d P1_T, Vector3d P2_T, Vector3d P3_T, Vector3d P4_T, double focal, double d_pxl) {
	
	PinHoleEquations pinHoleFunctor;
	Eigen::NumericalDiff<PinHoleEquations> numDiff(pinHoleFunctor);
	Eigen::LevenbergMarquardt<Eigen::NumericalDiff<PinHoleEquations>, double> levMarq(numDiff);
	//levMarq.parameters.maxfev = 2000;
	//levMarq.parameters.xtol = 1.0e-10;
	int ret = levMarq.minimize(variables);


}


struct PinHoleEquations {
	PinHoleEquations() {}
	Eigen::Matrix<double,11,1> operator()(Eigen::Matrix<double, 6, 1> variables, Vector3d q_d, Vector3d v, Vector2d PXL1, Vector2d PXL2, Vector2d PXL3, Vector2d PXL4, Vector3d P1_T, Vector3d P2_T, Vector3d P3_T, Vector3d P4_T, double focal, double d_pxl) const {
		
		double x = variables(0);
		double Y = variables(1);
		double Z = variables(2);
		double Yaw	 = variables(3);
		double Pitch = variables(4);
		double Roll	 = variables(5);

		double cosYaw = cos(Yaw);
		double sinYaw = sin(Yaw);
		double cosPitch = cos(Pitch);
		double sinPitch = sin(Pitch);
		double cosRoll = cos(Roll);
		double sinRoll = sin(Roll);

//#define aaa(a,b)  (cos(a)*sin(b)+a*a)

		double x_pxl_1 = PXL1(0);
		double x_pxl_2 = PXL2(0);
		double x_pxl_3 = PXL3(0);
		double x_pxl_4 = PXL4(0);

		double y_pxl_1 = PXL1(1);
		double y_pxl_2 = PXL2(1);
		double y_pxl_3 = PXL3(1);
		double y_pxl_4 = PXL4(1);

		double Px_1 = P1_T(0);
		double Px_2 = P2_T(0);
		double Px_3 = P3_T(0);
		double Px_4 = P4_T(0);

		double Py_1 = P1_T(1);
		double Py_2 = P2_T(1);
		double Py_3 = P3_T(1);
		double Py_4 = P4_T(1);

		double Pz_1 = P1_T(2);
		double Pz_2 = P2_T(2);
		double Pz_3 = P3_T(2);
		double Pz_4 = P4_T(2);

		double vx = v(0);
		double vy = v(1);
		double vz = v(2);

		double qx = q_d(0);
		double qy = q_d(1);
		double qz = q_d(2);

		Eigen::Matrix<double, 11, 1> k;
		k(0) = x_pxl_1 - (focal*(cosPitch*cosYaw*(Px_1 - x) - sinPitch*(Pz_1 - Z) + cosPitch*sinYaw*(Py_1 - Y))) / (d_pxl*((Px_1 - x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_1 - Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_1 - Z)));
		k(2) = x_pxl_2 - (focal*(cosPitch*cosYaw*(Px_2 - x) - sinPitch*(Pz_2 - Z) + cosPitch*sinYaw*(Py_2 - Y))) / (d_pxl*((Px_2 - x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_2 - Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_2 - Z)));
		k(4) = x_pxl_3 - (focal*(cosPitch*cosYaw*(Px_3 - x) - sinPitch*(Pz_3 - Z) + cosPitch*sinYaw*(Py_3 - Y))) / (d_pxl*((Px_3 - x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_3 - Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_3 - Z)));
		k(6) = x_pxl_4 - (focal*(cosPitch*cosYaw*(Px_4 - x) - sinPitch*(Pz_4 - Z) + cosPitch*sinYaw*(Py_4 - Y))) / (d_pxl*((Px_4 - x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_4 - Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_4 - Z)));

		k(1) = y_pxl_1 - (focal*((Py_1 - Y)*(cosRoll*cosYaw + sinPitch*sinRoll*sinYaw) - (Px_1 - x)*(cosRoll*sinYaw - cosYaw*sinPitch*sinRoll) + cosPitch*sinRoll*(Pz_1 - Z))) / (d_pxl*((Px_1 - x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_1 - Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_1 - Z)));			  
		k(3) = y_pxl_2 - (focal*((Py_2 - Y)*(cosRoll*cosYaw + sinPitch*sinRoll*sinYaw) - (Px_2 - x)*(cosRoll*sinYaw - cosYaw*sinPitch*sinRoll) + cosPitch*sinRoll*(Pz_2 - Z))) / (d_pxl*((Px_2 - x*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_2 - Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_2 - Z)));
		k(5) = y_pxl_3 - (focal*((Py_3 - Y)*(cosRoll*cosYaw + sinPitch*sinRoll*sinYaw) - (Px_3 - x)*(cosRoll*sinYaw - cosYaw*sinPitch*sinRoll) + cosPitch*sinRoll*(Pz_3 - Z))) / (d_pxl*((Px_3 - x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_3 - Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_3 - Z)));
		k(7) = y_pxl_4 - (focal*((Py_4 - Y)*(cosRoll*cosYaw + sinPitch*sinRoll*sinYaw) - (Px_4 - x)*(cosRoll*sinYaw - cosYaw*sinPitch*sinRoll) + cosPitch*sinRoll*(Pz_4 - Z))) / (d_pxl*((Px_4 - x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_4 - Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_4 - Z)));
		
		//change variable name, it sucks
		double denominator = sqrt(pow(abs(qx - Z*sinPitch + x*cosPitch*cosYaw + Y*cosPitch*sinYaw), 2) + pow(abs(qz + x*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - Y*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + Z*cosPitch*cosRoll), 2) + pow(abs(qy - x*(cosRoll*sinYaw - cosYaw*sinPitch*sinRoll) + Y*(cosRoll*cosYaw + sinPitch*sinRoll*sinYaw) + Z*cosPitch*sinRoll), 2));
		
		k(8)  = vx + (qx - Z*sinPitch + x*cosPitch*cosYaw + Y*cosPitch*sinYaw) / denominator;
		k(9)  = vy + (qy - x*(cosRoll*sinYaw - cosYaw*sinPitch*sinRoll) + Y*(cosRoll*cosYaw + sinPitch*sinRoll*sinYaw) + Z*cosPitch*sinRoll) / denominator;
		k(10) = vz + (qz + x*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - Y*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + Z*cosPitch*cosRoll) / denominator;

		return k; 
	}
};
