#include "stdafx.h"
#include <stdio.h>
#include <time.h>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen\Dense>
#include <unsupported\Eigen\NumericalDiff>
#include <unsupported\Eigen\NonLinearOptimization>
#include "P3p.h"
#include "Functions.h"

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector2d;
using namespace cv;
using namespace std;


/* This function finds the angles (in RADIANS) of the yaw - pitch - roll sequence
R1(gamma)*R2(beta)*R3(alpha) from the direction cosine matrix
Q - direction cosine matrix
yaw - yaw angle(rad)
pitch - pitch angle(rad)
roll - roll angle(rad) */
void dcm_to_ypr(Matrix3d &R, double* ypr) {
	ypr[0] = atan2(R(0, 1) , R(0, 0));
	ypr[1] = asin(-R(0, 2));
	ypr[2] = atan2(R(1, 2) , R(2, 2));
	ypr[0] = 180 * ypr[0] / M_PI;
	ypr[1] = 180 * ypr[1] / M_PI;
	ypr[2] = 180 * ypr[2] / M_PI;
}


/* Function che risolve il p3p con il metodo di Laurent Kneip
% Input:
%   P - Matrice 3x4 dei 4 punti nello spazio assoluto.Le colonne sono i
%       punti
%   f - Matrice 3x4 dei 4 versori nel sistema camera che individuano le
%       direzioni dei quattro punti fissi.Le colonne sono i versori
% Output :
%   C - Centro del sistema camera nel sistema di riferimento assoluto
%   R - Matrice di rotazione dal sistema assoluto a quello camera */

Eigen::Matrix<double, 3, 4> p3p_solver(Eigen::Matrix<double, 3, 4> &P, Eigen::Matrix<double, 3, 4> &f) {
	
	//auto begin = std::chrono::high_resolution_clock::now();
	
	//Fixed points at the base station (in millimeters)
	//Vector3d P1 = P.col(0);
	//Vector3d P2 = P.col(1);
	Vector3d P3 = P.col(2);
	//Vector3d P4 = P.col(3);

	//Versori normati nel riferimento della camera
	//Vector3d f1 = f.col(0);
	//Vector3d f2 = f.col(1);
	Vector3d f3 = f.col(2);
	//Vector3d f4 = f.col(3);
	
	//Primo calcolo
	//Input al codice di Kneip:
	Matrix3d wP;
	wP.col(0) = P.col(0); //wP.col(0) = P1;
	wP.col(1) = P.col(1); //wP.col(1) = P2;
	wP.col(2) = P.col(3); //wP.col(2) = P4;
	Matrix3d iV;
	iV.col(0) = f.col(0); //iV.col(0) = f1;
	iV.col(1) = f.col(1); //iV.col(1) = f2;
	iV.col(2) = f.col(3); //iV.col(2) = f4; 
	/*
	printf("\niV:");
	printMatrix(iV, 3, 3);
	printf("\nwP;");
	printMatrix(wP, 3, 3);
	*/
	//risoluzione del p3p
	P3p p3p;
	Eigen::Matrix<double, 3, 16> poses = p3p.computePoses(iV, wP);	//set n and m dimension
	/*
	printf("poses: \n");
	printMatrix(poses, 3, 16);
	*/
	//Vector3d C1 = poses.col(0);
	//Vector3d C2 = poses.col(4);
	//Vector3d C3 = poses.col(8);
	//Vector3d C4 = poses.col(12);
	Matrix3d R1;
	R1.col(0) = poses.col(1);
	R1.col(1) = poses.col(2);
	R1.col(2) = poses.col(3);
	//printf("R1: ");
	//printMatrix(R1, 3, 3);

	Matrix3d R2;
	R2.col(0) = poses.col(5);
	R2.col(1) = poses.col(6);
	R2.col(2) = poses.col(7);
	//printf("R2: ");
	//printMatrix(R2, 3, 3);

	Matrix3d R3;
	R3.col(0) = poses.col(9);
	R3.col(1) = poses.col(10);
	R3.col(2) = poses.col(11);
	//printf("R3: ");
	//printMatrix(R3, 3, 3);

	Matrix3d R4;
	R4.col(0) = poses.col(13);
	R4.col(1) = poses.col(14);
	R4.col(2) = poses.col(15);
	//printf("R4: ");
	//printMatrix(R4, 3, 3);

	//Eigen::Matrix<double, 3, 4> C;
	//C.col(0) = C1;
	//C.col(1) = C2;
	//C.col(2) = C3;
	//C.col(3) = C4;
	//printf("C: ");
	//printMatrix(C, 3, 4);

	Eigen::Matrix<double, 3, 12> R;
	R << R1, R2, R3, R4;
	//printf("R: ");
	//printMatrix(R, 3, 12);

	//Discriminazione della soluzione esatta
	Vector3d F31 = (R1*(P3 - poses.col(0)));	//Vector3d F31 = (R1*(P3 - C1));
	Vector3d F32 = (R2*(P3 - poses.col(4)));	//Vector3d F32 = (R2*(P3 - C2));
	Vector3d F33 = (R3*(P3 - poses.col(8)));	//Vector3d F33 = (R3*(P3 - C3));
	Vector3d F34 = (R4*(P3 - poses.col(12)));	//Vector3d F34 = (R4*(P3 - C4));
	F31.normalize();
	F32.normalize();
	F33.normalize();
	F34.normalize();
	//printf("F31: ");
	//printMatrix(F31, 3, 1);
	//printf("F32: ");
	//printMatrix(F32, 3, 1);
	//printf("F33: ");
	//printMatrix(F33, 3, 1);
	//printf("F34: ");
	//printMatrix(F34, 3, 1);

	Eigen::Matrix<double, 4, 1> dF;
	dF << abs((F31 - f3).norm()), abs((F32 - f3).norm()), abs((F33 - f3).norm()), abs((F34 - f3).norm());
	//printf("dF: ");
	//printMatrix(dF, 4, 1);


	//find a better algorithm for min finding
	double min = dF(0);
	int index = 0;
	for (int i = 1; i < 4; i++) {
		if (dF(i) < min) {
			min = dF(i);
			index = i;
		}
	}

	//Vector3d c = poses.col(4*index); //Vector3d c = C.col(index);
	//Matrix3d r;
	//r.col(0) = R.col(3 * index);
	//r.col(1) = R.col(3 * index + 1);
	//r.col(2) = R.col(3 * index + 2);

	Eigen::Matrix<double, 3, 4> solution;
	solution.col(0) = poses.col(4 * index); //solution.col(0) = c;
	solution.col(1) = R.col(3 * index);		//solution.col(1) = r.col(0);
	solution.col(2) = R.col(3 * index + 1); //solution.col(2) = r.col(1);
	solution.col(3) = R.col(3 * index + 2); //solution.col(3) = r.col(2);

	//auto end = std::chrono::high_resolution_clock::now();
	//long total = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
	//double timeInMillis = total / pow(10, 6);
	//printf("\nP3P_solver: %f millisecondi", timeInMillis);

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


struct PinHoleEquationsQD : Functor<double> /*TODO: inputs and values missing! Add them!*/{
	
	PinHoleEquationsQD(double* q_d, double* v, double* PXL1, double* PXL2, double* PXL3, double* PXL4, double* P1_T, double* P2_T, double* P3_T, double* P4_T, double focal, double d_pxl) 
		: Functor(6,11), q_d(q_d), v(v), PXL1(PXL1), PXL2(PXL2), PXL3(PXL3), PXL4(PXL4), P1_T(P1_T), P2_T(P2_T), P3_T(P3_T), P4_T(P4_T), focal(focal), d_pxl(d_pxl) {}

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

		double x_pxl_1 = PXL1[0];	//printf("\nx_pxl_1: %f",x_pxl_1);
		double x_pxl_2 = PXL2[0];	//printf("\nx_pxl_2: %f",x_pxl_2);
		double x_pxl_3 = PXL3[0];	//printf("\nx_pxl_3: %f",x_pxl_3);
		double x_pxl_4 = PXL4[0];	//printf("\nx_pxl_4: %f",x_pxl_4);
						 	 		//
		double y_pxl_1 = PXL1[1];	//printf("\ny_pxl_1: %f",y_pxl_1);
		double y_pxl_2 = PXL2[1];	//printf("\ny_pxl_2: %f",y_pxl_2);
		double y_pxl_3 = PXL3[1];	//printf("\ny_pxl_3: %f",y_pxl_3);
		double y_pxl_4 = PXL4[1];	//printf("\ny_pxl_4: %f",y_pxl_4);
									//
		double Px_1 = P1_T[0];		//printf("\nPx_1: %f",Px_1);
		double Px_2 = P2_T[0];		//printf("\nPx_2: %f",Px_2);
		double Px_3 = P3_T[0];		//printf("\nPx_3: %f",Px_3);
		double Px_4 = P4_T[0];		//printf("\nPx_4: %f",Px_4);
					  	  			//
		double Py_1 = P1_T[1];		//printf("\nPy_1: %f",Py_1);
		double Py_2 = P2_T[1];		//printf("\nPy_2: %f",Py_2);
		double Py_3 = P3_T[1];		//printf("\nPy_3: %f",Py_3);
		double Py_4 = P4_T[1];		//printf("\nPy_4: %f",Py_4);
					  	  			//
		double Pz_1 = P1_T[2];		//printf("\nPz_1: %f",Pz_1);
		double Pz_2 = P2_T[2];		//printf("\nPz_2: %f",Pz_2);
		double Pz_3 = P3_T[2];		//printf("\nPz_3: %f",Pz_3);
		double Pz_4 = P4_T[2];		//printf("\nPz_4: %f",Pz_4);
									//
		double vx = v[0];			//printf("\nvx: %f",vx);
		double vy = v[1];			//printf("\nvy: %f",vy);
		double vz = v[2];			//printf("\nvz: %f",vz);
									//
		double qx = q_d[0];			//printf("\nqx: %f",qx);
		double qy = q_d[1];			//printf("\nqy: %f",qy);
		double qz = q_d[2];			//printf("\nqz: %f",qz);

		double cosPcosY     = cosPitch*cosYaw;
		double cosPsinY     = cosPitch*sinYaw;
		double sinRsinY     = sinRoll*sinYaw;
		double cosRcosYsinP = cosRoll*cosYaw*sinPitch;
		double cosYsinR		= cosYaw*sinRoll;
		double cosRsinPsinY = cosRoll*sinPitch*sinYaw;
		double cosPcosR     = cosPitch*cosRoll;
		double cosRcosY		= cosRoll*cosYaw;
		double sinPsinRsinY = sinPitch*sinRoll*sinYaw;
		double cosRsinY		= cosRoll*sinYaw;
		double cosYsinPsinR = cosYaw*sinPitch*sinRoll;
		double cosPsinR		= cosPitch*sinRoll;
		/*
#define fx(xPXL,pX,pY,pZ)  xPXL - (focal*(cosPcosY*(pX + x) - sinPitch*(pZ + Z) + cosPsinY*(pY + Y)));
#define fy(yPXL,pX,pY,pZ)  yPXL - (focal*((pY + Y)*(cosRcosY + sinPsinRsinY) - (pX + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(pZ + Z)));
#define den(pX,pY,pZ)  (d_pxl*((pX + x)*(sinRsinY + cosRcosYsinP) - (pY + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(pZ + Z)));
*/
		fvec(0) = x_pxl_1 - (focal*(cosPcosY*(Px_1 + x) - sinPitch*(Pz_1 + Z) + cosPsinY*(Py_1 + Y))) / (d_pxl*((Px_1 + x)*(sinRsinY + cosRcosYsinP) - (Py_1 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_1 + Z)));
		fvec(2) = x_pxl_2 - (focal*(cosPcosY*(Px_2 + x) - sinPitch*(Pz_2 + Z) + cosPsinY*(Py_2 + Y))) / (d_pxl*((Px_2 + x)*(sinRsinY + cosRcosYsinP) - (Py_2 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_2 + Z)));
		fvec(4) = x_pxl_3 - (focal*(cosPcosY*(Px_3 + x) - sinPitch*(Pz_3 + Z) + cosPsinY*(Py_3 + Y))) / (d_pxl*((Px_3 + x)*(sinRsinY + cosRcosYsinP) - (Py_3 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_3 + Z)));
		fvec(6) = x_pxl_4 - (focal*(cosPcosY*(Px_4 + x) - sinPitch*(Pz_4 + Z) + cosPsinY*(Py_4 + Y))) / (d_pxl*((Px_4 + x)*(sinRsinY + cosRcosYsinP) - (Py_4 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_4 + Z)));
		/*
		double den1 = den(Px_1, Py_1, Pz_1);
		double den2 = den(Px_2, Py_2, Pz_2);
		double den3 = den(Px_3, Py_3, Pz_3);
		double den4 = den(Px_4, Py_4, Pz_4);
		
		fvec(0) = fx(x_pxl_1,Px_1,Py_1,Pz_1) / den1;
		fvec(2) = fx(x_pxl_2,Px_2,Py_2,Pz_2) / den2;
		fvec(4) = fx(x_pxl_3,Px_3,Py_3,Pz_3) / den3;
		fvec(6) = fx(x_pxl_4,Px_4,Py_4,Pz_4) / den4;
		*/
		fvec(1) = y_pxl_1 - (focal*((Py_1 + Y)*(cosRcosY + sinPsinRsinY) - (Px_1 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_1 + Z))) / (d_pxl*((Px_1 + x)*(sinRsinY + cosRcosYsinP) - (Py_1 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_1 + Z)));
		fvec(3) = y_pxl_2 - (focal*((Py_2 + Y)*(cosRcosY + sinPsinRsinY) - (Px_2 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_2 + Z))) / (d_pxl*((Px_2 + x)*(sinRsinY + cosRcosYsinP) - (Py_2 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_2 + Z)));
		fvec(5) = y_pxl_3 - (focal*((Py_3 + Y)*(cosRcosY + sinPsinRsinY) - (Px_3 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_3 + Z))) / (d_pxl*((Px_3 + x)*(sinRsinY + cosRcosYsinP) - (Py_3 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_3 + Z)));
		fvec(7) = y_pxl_4 - (focal*((Py_4 + Y)*(cosRcosY + sinPsinRsinY) - (Px_4 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_4 + Z))) / (d_pxl*((Px_4 + x)*(sinRsinY + cosRcosYsinP) - (Py_4 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_4 + Z)));
		/*
		fvec(1) = fy(y_pxl_1,Px_1,Py_1,Pz_1) / den1;
		fvec(3) = fy(y_pxl_2,Px_2,Py_2,Pz_2) / den2;
		fvec(5) = fy(y_pxl_3,Px_3,Py_3,Pz_3) / den3;
		fvec(7) = fy(y_pxl_4,Px_4,Py_4,Pz_4) / den4;
		*/
		//TODO: change variable name, it sucks
		double denominator = sqrt(pow(abs(qx - Z*sinPitch + x*cosPitch*cosYaw + Y*cosPitch*sinYaw), 2) + pow(abs(qz + x*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - Y*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + Z*cosPitch*cosRoll), 2) + pow(abs(qy - x*(cosRoll*sinYaw - cosYaw*sinPitch*sinRoll) + Y*(cosRoll*cosYaw + sinPitch*sinRoll*sinYaw) + Z*cosPitch*sinRoll), 2));

		fvec(8) = vx + (qx - Z*sinPitch + x*cosPcosY + Y*cosPsinY) / denominator;
		fvec(9) = vy + (qy - x*(cosRsinY - cosYsinPsinR) + Y*(cosRcosY + sinPsinRsinY) + Z*cosPsinR) / denominator;
		fvec(10) = vz + (qz + x*(sinRsinY + cosRcosYsinP) - Y*(cosYsinR - cosRsinPsinY) + Z*cosPcosR) / denominator;
		//printf("\nfvec:");
		//printMatrix(fvec, 11, 1);
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

struct PinHoleEquationsUS : Functor<double> /*TODO: inputs and values missing! Add them!*/ {

	PinHoleEquationsUS(double* q_d, double* PXL1, double* PXL2, double* PXL3, double* PXL4, double* P1_T, double* P2_T, double* P3_T, double* P4_T, double* delta, double* d1, double* d2, double* d3, double* d4, double focal, double d_pxl, double v)
		: Functor(6, 11), q_d(q_d), PXL1(PXL1), PXL2(PXL2), PXL3(PXL3), PXL4(PXL4), P1_T(P1_T), P2_T(P2_T), P3_T(P3_T), P4_T(P4_T), delta(delta), d1(d1), d2(d2), d3(d3), d4(d4), focal(focal), d_pxl(d_pxl), v(v) {}

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

		double x_pxl_1 = PXL1[0];	//printf("\nx_pxl_1: %f",x_pxl_1);
		double x_pxl_2 = PXL2[0];	//printf("\nx_pxl_2: %f",x_pxl_2);
		double x_pxl_3 = PXL3[0];	//printf("\nx_pxl_3: %f",x_pxl_3);
		double x_pxl_4 = PXL4[0];	//printf("\nx_pxl_4: %f",x_pxl_4);
									//
		double y_pxl_1 = PXL1[1];	//printf("\ny_pxl_1: %f",y_pxl_1);
		double y_pxl_2 = PXL2[1];	//printf("\ny_pxl_2: %f",y_pxl_2);
		double y_pxl_3 = PXL3[1];	//printf("\ny_pxl_3: %f",y_pxl_3);
		double y_pxl_4 = PXL4[1];	//printf("\ny_pxl_4: %f",y_pxl_4);
									//
		double Px_1 = P1_T[0];		//printf("\nPx_1: %f",Px_1);
		double Px_2 = P2_T[0];		//printf("\nPx_2: %f",Px_2);
		double Px_3 = P3_T[0];		//printf("\nPx_3: %f",Px_3);
		double Px_4 = P4_T[0];		//printf("\nPx_4: %f",Px_4);
									//
		double Py_1 = P1_T[1];		//printf("\nPy_1: %f",Py_1);
		double Py_2 = P2_T[1];		//printf("\nPy_2: %f",Py_2);
		double Py_3 = P3_T[1];		//printf("\nPy_3: %f",Py_3);
		double Py_4 = P4_T[1];		//printf("\nPy_4: %f",Py_4);
									//
		double Pz_1 = P1_T[2];		//printf("\nPz_1: %f",Pz_1);
		double Pz_2 = P2_T[2];		//printf("\nPz_2: %f",Pz_2);
		double Pz_3 = P3_T[2];		//printf("\nPz_3: %f",Pz_3);
		double Pz_4 = P4_T[2];		//printf("\nPz_4: %f",Pz_4);
									//
		//double vx = v[0];			//printf("\nvx: %f",vx);
		//double vy = v[1];			//printf("\nvy: %f",vy);
		//double vz = v[2];			//printf("\nvz: %f",vz);
									//
		double qx = q_d[0];			//printf("\nqx: %f",qx);
		double qy = q_d[1];			//printf("\nqy: %f",qy);
		double qz = q_d[2];			//printf("\nqz: %f",qz);

		double Delta_t12 = delta[0];
		double Delta_t13 = delta[1];
		double Delta_t14 = delta[2];
		double dx1 = d1[0];
		double dy1 = d1[1];
		double dz1 = d1[2];
		double dx2 = d2[0];
		double dy2 = d2[1];
		double dz2 = d2[2];
		double dx3 = d3[0];
		double dy3 = d3[1];
		double dz3 = d3[2];
		double dx4 = d4[0];
		double dy4 = d4[1];
		double dz4 = d4[2];


		double cosPcosY = cosPitch*cosYaw;
		double cosPsinY = cosPitch*sinYaw;
		double sinRsinY = sinRoll*sinYaw;
		double cosRcosYsinP = cosRoll*cosYaw*sinPitch;
		double cosYsinR = cosYaw*sinRoll;
		double cosRsinPsinY = cosRoll*sinPitch*sinYaw;
		double cosPcosR = cosPitch*cosRoll;
		double cosRcosY = cosRoll*cosYaw;
		double sinPsinRsinY = sinPitch*sinRoll*sinYaw;
		double cosRsinY = cosRoll*sinYaw;
		double cosYsinPsinR = cosYaw*sinPitch*sinRoll;
		double cosPsinR = cosPitch*sinRoll;
		
		fvec(0) = x_pxl_1 - (focal*(cosPcosY*(Px_1 + x) - sinPitch*(Pz_1 + Z) + cosPsinY*(Py_1 + Y))) / (d_pxl*((Px_1 + x)*(sinRsinY + cosRcosYsinP) - (Py_1 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_1 + Z)));
		fvec(2) = x_pxl_2 - (focal*(cosPcosY*(Px_2 + x) - sinPitch*(Pz_2 + Z) + cosPsinY*(Py_2 + Y))) / (d_pxl*((Px_2 + x)*(sinRsinY + cosRcosYsinP) - (Py_2 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_2 + Z)));
		fvec(4) = x_pxl_3 - (focal*(cosPcosY*(Px_3 + x) - sinPitch*(Pz_3 + Z) + cosPsinY*(Py_3 + Y))) / (d_pxl*((Px_3 + x)*(sinRsinY + cosRcosYsinP) - (Py_3 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_3 + Z)));
		fvec(6) = x_pxl_4 - (focal*(cosPcosY*(Px_4 + x) - sinPitch*(Pz_4 + Z) + cosPsinY*(Py_4 + Y))) / (d_pxl*((Px_4 + x)*(sinRsinY + cosRcosYsinP) - (Py_4 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_4 + Z)));
		
		fvec(1) = y_pxl_1 - (focal*((Py_1 + Y)*(cosRcosY + sinPsinRsinY) - (Px_1 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_1 + Z))) / (d_pxl*((Px_1 + x)*(sinRsinY + cosRcosYsinP) - (Py_1 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_1 + Z)));
		fvec(3) = y_pxl_2 - (focal*((Py_2 + Y)*(cosRcosY + sinPsinRsinY) - (Px_2 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_2 + Z))) / (d_pxl*((Px_2 + x)*(sinRsinY + cosRcosYsinP) - (Py_2 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_2 + Z)));
		fvec(5) = y_pxl_3 - (focal*((Py_3 + Y)*(cosRcosY + sinPsinRsinY) - (Px_3 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_3 + Z))) / (d_pxl*((Px_3 + x)*(sinRsinY + cosRcosYsinP) - (Py_3 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_3 + Z)));
		fvec(7) = y_pxl_4 - (focal*((Py_4 + Y)*(cosRcosY + sinPsinRsinY) - (Px_4 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_4 + Z))) / (d_pxl*((Px_4 + x)*(sinRsinY + cosRcosYsinP) - (Py_4 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_4 + Z)));
		
		//TODO: change variable name, it sucks
		double denominator = sqrt(pow(abs(qx - Z*sinPitch + x*cosPitch*cosYaw + Y*cosPitch*sinYaw), 2) + pow(abs(qz + x*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - Y*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + Z*cosPitch*cosRoll), 2) + pow(abs(qy - x*(cosRoll*sinYaw - cosYaw*sinPitch*sinRoll) + Y*(cosRoll*cosYaw + sinPitch*sinRoll*sinYaw) + Z*cosPitch*sinRoll), 2));

		fvec(8)  = Delta_t12 - (sqrt(pow(abs(Z - (dx1*sinPitch) / (pow(cosPitch, 2) + pow(sinPitch, 2)) + (dz1*cosPitch*cosRoll) / (pow(cosPitch, 2)*pow(cosRoll, 2) + pow(cosPitch, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)) + (dy1*cosPitch*sinRoll) / (pow(cosPitch, 2)*pow(cosRoll, 2) + pow(cosPitch, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinRoll, 2))), 2) + pow(abs(Y + (dy1*(sinPitch*sinRoll*sinYaw + pow(cosPitch, 2)*cosRoll*cosYaw + cosRoll*cosYaw*pow(sinPitch, 2))) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) - (dz1*(cosYaw*pow(sinPitch, 2)*sinRoll - cosRoll*sinPitch*sinYaw + pow(cosPitch, 2)*cosYaw*sinRoll)) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) + (dx1*cosPitch*sinYaw) / (pow(cosPitch, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinYaw, 2))), 2) + pow(abs(x - (dy1*(cosRoll*pow(sinPitch, 2)*sinYaw - cosYaw*sinPitch*sinRoll + pow(cosPitch, 2)*cosRoll*sinYaw)) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) + (dz1*(pow(cosPitch, 2)*sinRoll*sinYaw + pow(sinPitch, 2)*sinRoll*sinYaw + cosRoll*cosYaw*sinPitch)) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) + (dx1*cosPitch*cosYaw) / (pow(cosPitch, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinYaw, 2))), 2)) - sqrt(pow(abs(Z - (dx2*sinPitch) / (pow(cosPitch, 2) + pow(sinPitch, 2)) + (dz2*cosPitch*cosRoll) / (pow(cosPitch, 2)*pow(cosRoll, 2) + pow(cosPitch, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)) + (dy2*cosPitch*sinRoll) / (pow(cosPitch, 2)*pow(cosRoll, 2) + pow(cosPitch, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinRoll, 2))), 2) + pow(abs(Y + (dy2*(sinPitch*sinRoll*sinYaw + pow(cosPitch, 2)*cosRoll*cosYaw + cosRoll*cosYaw*pow(sinPitch, 2))) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) - (dz2*(cosYaw*pow(sinPitch, 2)*sinRoll - cosRoll*sinPitch*sinYaw + pow(cosPitch, 2)*cosYaw*sinRoll)) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) + (dx2*cosPitch*sinYaw) / (pow(cosPitch, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinYaw, 2))), 2) + pow(abs(x - (dy2*(cosRoll*pow(sinPitch, 2)*sinYaw - cosYaw*sinPitch*sinRoll + pow(cosPitch, 2)*cosRoll*sinYaw)) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) + (dz2*(pow(cosPitch, 2)*sinRoll*sinYaw + pow(sinPitch, 2)*sinRoll*sinYaw + cosRoll*cosYaw*sinPitch)) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) + (dx2*cosPitch*cosYaw) / (pow(cosPitch, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinYaw, 2))), 2))) / v;
		fvec(9)  = Delta_t14 - (sqrt(pow(abs(Z - (dx1*sinPitch) / (pow(cosPitch, 2) + pow(sinPitch, 2)) + (dz1*cosPitch*cosRoll) / (pow(cosPitch, 2)*pow(cosRoll, 2) + pow(cosPitch, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)) + (dy1*cosPitch*sinRoll) / (pow(cosPitch, 2)*pow(cosRoll, 2) + pow(cosPitch, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinRoll, 2))), 2) + pow(abs(Y + (dy1*(sinPitch*sinRoll*sinYaw + pow(cosPitch, 2)*cosRoll*cosYaw + cosRoll*cosYaw*pow(sinPitch, 2))) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) - (dz1*(cosYaw*pow(sinPitch, 2)*sinRoll - cosRoll*sinPitch*sinYaw + pow(cosPitch, 2)*cosYaw*sinRoll)) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) + (dx1*cosPitch*sinYaw) / (pow(cosPitch, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinYaw, 2))), 2) + pow(abs(x - (dy1*(cosRoll*pow(sinPitch, 2)*sinYaw - cosYaw*sinPitch*sinRoll + pow(cosPitch, 2)*cosRoll*sinYaw)) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) + (dz1*(pow(cosPitch, 2)*sinRoll*sinYaw + pow(sinPitch, 2)*sinRoll*sinYaw + cosRoll*cosYaw*sinPitch)) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) + (dx1*cosPitch*cosYaw) / (pow(cosPitch, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinYaw, 2))), 2)) - sqrt(pow(abs(Z - (dx4*sinPitch) / (pow(cosPitch, 2) + pow(sinPitch, 2)) + (dz4*cosPitch*cosRoll) / (pow(cosPitch, 2)*pow(cosRoll, 2) + pow(cosPitch, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)) + (dy4*cosPitch*sinRoll) / (pow(cosPitch, 2)*pow(cosRoll, 2) + pow(cosPitch, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinRoll, 2))), 2) + pow(abs(Y + (dy4*(sinPitch*sinRoll*sinYaw + pow(cosPitch, 2)*cosRoll*cosYaw + cosRoll*cosYaw*pow(sinPitch, 2))) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) - (dz4*(cosYaw*pow(sinPitch, 2)*sinRoll - cosRoll*sinPitch*sinYaw + pow(cosPitch, 2)*cosYaw*sinRoll)) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) + (dx4*cosPitch*sinYaw) / (pow(cosPitch, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinYaw, 2))), 2) + pow(abs(x - (dy4*(cosRoll*pow(sinPitch, 2)*sinYaw - cosYaw*sinPitch*sinRoll + pow(cosPitch, 2)*cosRoll*sinYaw)) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) + (dz4*(pow(cosPitch, 2)*sinRoll*sinYaw + pow(sinPitch, 2)*sinRoll*sinYaw + cosRoll*cosYaw*sinPitch)) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) + (dx4*cosPitch*cosYaw) / (pow(cosPitch, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinYaw, 2))), 2))) / v;
		fvec(10) = Delta_t13 - (sqrt(pow(abs(Z - (dx1*sinPitch) / (pow(cosPitch, 2) + pow(sinPitch, 2)) + (dz1*cosPitch*cosRoll) / (pow(cosPitch, 2)*pow(cosRoll, 2) + pow(cosPitch, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)) + (dy1*cosPitch*sinRoll) / (pow(cosPitch, 2)*pow(cosRoll, 2) + pow(cosPitch, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinRoll, 2))), 2) + pow(abs(Y + (dy1*(sinPitch*sinRoll*sinYaw + pow(cosPitch, 2)*cosRoll*cosYaw + cosRoll*cosYaw*pow(sinPitch, 2))) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) - (dz1*(cosYaw*pow(sinPitch, 2)*sinRoll - cosRoll*sinPitch*sinYaw + pow(cosPitch, 2)*cosYaw*sinRoll)) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) + (dx1*cosPitch*sinYaw) / (pow(cosPitch, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinYaw, 2))), 2) + pow(abs(x - (dy1*(cosRoll*pow(sinPitch, 2)*sinYaw - cosYaw*sinPitch*sinRoll + pow(cosPitch, 2)*cosRoll*sinYaw)) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) + (dz1*(pow(cosPitch, 2)*sinRoll*sinYaw + pow(sinPitch, 2)*sinRoll*sinYaw + cosRoll*cosYaw*sinPitch)) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) + (dx1*cosPitch*cosYaw) / (pow(cosPitch, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinYaw, 2))), 2)) - sqrt(pow(abs(Z - (dx3*sinPitch) / (pow(cosPitch, 2) + pow(sinPitch, 2)) + (dz3*cosPitch*cosRoll) / (pow(cosPitch, 2)*pow(cosRoll, 2) + pow(cosPitch, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)) + (dy3*cosPitch*sinRoll) / (pow(cosPitch, 2)*pow(cosRoll, 2) + pow(cosPitch, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinRoll, 2))), 2) + pow(abs(Y + (dy3*(sinPitch*sinRoll*sinYaw + pow(cosPitch, 2)*cosRoll*cosYaw + cosRoll*cosYaw*pow(sinPitch, 2))) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) - (dz3*(cosYaw*pow(sinPitch, 2)*sinRoll - cosRoll*sinPitch*sinYaw + pow(cosPitch, 2)*cosYaw*sinRoll)) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) + (dx3*cosPitch*sinYaw) / (pow(cosPitch, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinYaw, 2))), 2) + pow(abs(x - (dy3*(cosRoll*pow(sinPitch, 2)*sinYaw - cosYaw*sinPitch*sinRoll + pow(cosPitch, 2)*cosRoll*sinYaw)) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) + (dz3*(pow(cosPitch, 2)*sinRoll*sinYaw + pow(sinPitch, 2)*sinRoll*sinYaw + cosRoll*cosYaw*sinPitch)) / (pow(cosPitch, 2)*pow(cosRoll, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(cosRoll, 2)*pow(sinYaw, 2) + pow(cosPitch, 2)*pow(cosYaw, 2)*pow(sinRoll, 2) + pow(cosRoll, 2)*pow(cosYaw, 2)*pow(sinPitch, 2) + pow(cosPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2) + pow(cosRoll, 2)*pow(sinPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2)*pow(sinRoll, 2) + pow(sinPitch, 2)*pow(sinRoll, 2)*pow(sinYaw, 2)) + (dx3*cosPitch*cosYaw) / (pow(cosPitch, 2)*pow(cosYaw, 2) + pow(cosPitch, 2)*pow(sinYaw, 2) + pow(cosYaw, 2)*pow(sinPitch, 2) + pow(sinPitch, 2)*pow(sinYaw, 2))), 2))) / v;
		//printf("\nfvec:");
		//printMatrix(fvec, 11, 1);
		return 1;
	}

	double *q_d;
	double *PXL1;
	double *PXL2;
	double *PXL3;
	double *PXL4;
	double *P1_T;
	double *P2_T;
	double *P3_T;
	double *P4_T;
	double *delta;
	double *d1;
	double *d2;
	double *d3;
	double *d4;
	double focal;
	double d_pxl;
	double v;
};

long pinHoleFSolveQD(Eigen::Matrix<double,6,1> &variables, double* q_d, double* v, double* PXL1, double* PXL2, double* PXL3, double* PXL4, double* P1_T, double* P2_T, double* P3_T, double* P4_T, double focal, double d_pxl) {
	
	auto begin = std::chrono::high_resolution_clock::now();

	PinHoleEquationsQD pinHoleFunctor(q_d, v, PXL1, PXL2, PXL3, PXL4, P1_T, P2_T, P3_T, P4_T, focal, d_pxl);
	Eigen::NumericalDiff<PinHoleEquationsQD> numDiff(pinHoleFunctor);
	Eigen::LevenbergMarquardt<Eigen::NumericalDiff<PinHoleEquationsQD>, double> levMarq(numDiff);
	levMarq.parameters.maxfev = 2000;
	levMarq.parameters.xtol = 1.0e-10;
	Eigen::VectorXd dynVar(6);
	dynVar(0) = variables(0);
	dynVar(1) = variables(1);
	dynVar(2) = variables(2);
	dynVar(3) = variables(3);
	dynVar(4) = variables(4);
	dynVar(5) = variables(5);

	//walking inside the world of black magic...
	int ret = levMarq.minimize(dynVar);
	variables = dynVar;

	auto end = std::chrono::high_resolution_clock::now();
	long total = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
	//double timeInMillis = total / pow(10, 6);
	//printf("\nFsolve: %f millisecondi", timeInMillis);

	return total;
}

long pinHoleFSolveUS(Eigen::Matrix<double, 6, 1> &variables, double* q_d, double* PXL1, double* PXL2, double* PXL3, double* PXL4, double* P1_T, double* P2_T, double* P3_T, double* P4_T, double* delta, double* d1, double* d2, double* d3, double* d4, double focal, double d_pxl, double v) {

	auto begin = std::chrono::high_resolution_clock::now();

	PinHoleEquationsUS pinHoleFunctor(q_d, PXL1, PXL2, PXL3, PXL4, P1_T, P2_T, P3_T, P4_T, delta, d1, d2, d3, d4, focal, d_pxl, v);
	Eigen::NumericalDiff<PinHoleEquationsUS> numDiff(pinHoleFunctor);
	Eigen::LevenbergMarquardt<Eigen::NumericalDiff<PinHoleEquationsUS>, double> levMarq(numDiff);
	levMarq.parameters.maxfev = 2000;
	levMarq.parameters.xtol = 1.0e-10;
	Eigen::VectorXd dynVar(6);
	dynVar(0) = variables(0);
	dynVar(1) = variables(1);
	dynVar(2) = variables(2);
	dynVar(3) = variables(3);
	dynVar(4) = variables(4);
	dynVar(5) = variables(5);

	//walking inside the world of black magic...
	int ret = levMarq.minimize(dynVar);
	variables = dynVar;

	auto end = std::chrono::high_resolution_clock::now();
	long total = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
	//double timeInMillis = total / pow(10, 6);
	//printf("\nFsolve: %f millisecondi", timeInMillis);

	return total;
}

void printMatrix(Eigen::MatrixXd mtrx, int n, int m) {
	for (int i = 0; i < n; i++) {
		printf("\n");
		for (int j = 0; j < m; j++) {
			printf("%f ", mtrx(i, j));
		}
	}
	printf("\n");
}

