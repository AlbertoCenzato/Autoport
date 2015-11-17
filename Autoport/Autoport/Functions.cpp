#include "stdafx.h"
#include <stdio.h>
#include <Eigen\Dense>
#include "..\Eigen\unsupported\Eigen\NonLinearOptimization"
#include "..\Eigen\unsupported\Eigen\NumericalDiff"
#include "P3p.h"
#include "Functions.h"
#include <time.h>
#include <chrono>

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector2d;

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
	
	auto begin = std::chrono::high_resolution_clock::now();
	
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
	Vector3d C1 = poses.col(0);
	Vector3d C2 = poses.col(4);
	Vector3d C3 = poses.col(8);
	Vector3d C4 = poses.col(12);
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

	Eigen::Matrix<double, 3, 4> C;
	C.col(0) = C1;
	C.col(1) = C2;
	C.col(2) = C3;
	C.col(3) = C4;
	//printf("C: ");
	//printMatrix(C, 3, 4);

	Eigen::Matrix<double, 3, 12> R;
	R << R1, R2, R3, R4;
	//printf("R: ");
	//printMatrix(R, 3, 12);

	//Discriminazione della soluzione esatta
	Vector3d F31 = (R1*(P3 - C1));
	Vector3d F32 = (R2*(P3 - C2));
	Vector3d F33 = (R3*(P3 - C3));
	Vector3d F34 = (R4*(P3 - C4));
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

	auto end = std::chrono::high_resolution_clock::now();
	long total = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
	double timeInMillis = total / pow(10, 6);
	printf("\nP3P_solver: %f millisecondi", timeInMillis);

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

		fvec(0) = x_pxl_1 - (focal*(cosPitch*cosYaw*(Px_1 + x) - sinPitch*(Pz_1 + Z) + cosPitch*sinYaw*(Py_1 + Y))) / (d_pxl*((Px_1 + x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_1 + Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_1 + Z)));
		fvec(2) = x_pxl_2 - (focal*(cosPitch*cosYaw*(Px_2 + x) - sinPitch*(Pz_2 + Z) + cosPitch*sinYaw*(Py_2 + Y))) / (d_pxl*((Px_2 + x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_2 + Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_2 + Z)));
		fvec(4) = x_pxl_3 - (focal*(cosPitch*cosYaw*(Px_3 + x) - sinPitch*(Pz_3 + Z) + cosPitch*sinYaw*(Py_3 + Y))) / (d_pxl*((Px_3 + x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_3 + Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_3 + Z)));
		fvec(6) = x_pxl_4 - (focal*(cosPitch*cosYaw*(Px_4 + x) - sinPitch*(Pz_4 + Z) + cosPitch*sinYaw*(Py_4 + Y))) / (d_pxl*((Px_4 + x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_4 + Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_4 + Z)));

		fvec(1) = y_pxl_1 - (focal*((Py_1 + Y)*(cosRoll*cosYaw + sinPitch*sinRoll*sinYaw) - (Px_1 + x)*(cosRoll*sinYaw - cosYaw*sinPitch*sinRoll) + cosPitch*sinRoll*(Pz_1 + Z))) / (d_pxl*((Px_1 + x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_1 + Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_1 + Z)));
		fvec(3) = y_pxl_2 - (focal*((Py_2 + Y)*(cosRoll*cosYaw + sinPitch*sinRoll*sinYaw) - (Px_2 + x)*(cosRoll*sinYaw - cosYaw*sinPitch*sinRoll) + cosPitch*sinRoll*(Pz_2 + Z))) / (d_pxl*((Px_2 + x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_2 + Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_2 + Z)));
		fvec(5) = y_pxl_3 - (focal*((Py_3 + Y)*(cosRoll*cosYaw + sinPitch*sinRoll*sinYaw) - (Px_3 + x)*(cosRoll*sinYaw - cosYaw*sinPitch*sinRoll) + cosPitch*sinRoll*(Pz_3 + Z))) / (d_pxl*((Px_3 + x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_3 + Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_3 + Z)));
		fvec(7) = y_pxl_4 - (focal*((Py_4 + Y)*(cosRoll*cosYaw + sinPitch*sinRoll*sinYaw) - (Px_4 + x)*(cosRoll*sinYaw - cosYaw*sinPitch*sinRoll) + cosPitch*sinRoll*(Pz_4 + Z))) / (d_pxl*((Px_4 + x)*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - (Py_4 + Y)*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + cosPitch*cosRoll*(Pz_4 + Z)));

		//TODO: change variable name, it sucks
		double denominator = sqrt(pow(abs(qx - Z*sinPitch + x*cosPitch*cosYaw + Y*cosPitch*sinYaw), 2) + pow(abs(qz + x*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - Y*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + Z*cosPitch*cosRoll), 2) + pow(abs(qy - x*(cosRoll*sinYaw - cosYaw*sinPitch*sinRoll) + Y*(cosRoll*cosYaw + sinPitch*sinRoll*sinYaw) + Z*cosPitch*sinRoll), 2));

		fvec(8) = vx + (qx - Z*sinPitch + x*cosPitch*cosYaw + Y*cosPitch*sinYaw) / denominator;
		fvec(9) = vy + (qy - x*(cosRoll*sinYaw - cosYaw*sinPitch*sinRoll) + Y*(cosRoll*cosYaw + sinPitch*sinRoll*sinYaw) + Z*cosPitch*sinRoll) / denominator;
		fvec(10) = vz + (qz + x*(sinRoll*sinYaw + cosRoll*cosYaw*sinPitch) - Y*(cosYaw*sinRoll - cosRoll*sinPitch*sinYaw) + Z*cosPitch*cosRoll) / denominator;
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


//TODO: fix the size of VectorXd variables
int pinHoleFSolve(Eigen::Matrix<double,6,1> &variables, double* q_d, double* v, double* PXL1, double* PXL2, double* PXL3, double* PXL4, double* P1_T, double* P2_T, double* P3_T, double* P4_T, double focal, double d_pxl) {
	
	auto begin = std::chrono::high_resolution_clock::now();

	PinHoleEquations pinHoleFunctor(q_d, v, PXL1, PXL2, PXL3, PXL4, P1_T, P2_T, P3_T, P4_T, focal, d_pxl);
	Eigen::NumericalDiff<PinHoleEquations> numDiff(pinHoleFunctor);
	Eigen::LevenbergMarquardt<Eigen::NumericalDiff<PinHoleEquations>, double> levMarq(numDiff);
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
	double timeInMillis = total / pow(10, 6);
	printf("\nFsolve: %f millisecondi", timeInMillis);

	return ret;
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



void simulazioneCompleta() {
	Vector3d q_d(400, 0, 0);			//Coordinate del Quadrant Detector nel sistema drone
	Vector3d P1_T(500, -500, 0);	//Coordinate del led 1 nel sistema target
	Vector3d P2_T(500, 500, 0);	//Coordinate del led 2 nel sistema target
	Vector3d P3_T(0, 500, 0);	//Coordinate del led 3 nel sistema target
	Vector3d P4_T(-500, 500, 0);	//Coordinate del led 4 nel sistema target

	Vector3d q_dn = q_d + Vector3d(0.1, 0.1, 0);		 //Coordinate con RUMORE del Quadrant Detector nel sistema drone
	Vector3d P1_Tn = P1_T + Vector3d(0.1, -0.1, 0.2); //Coordinate con RUMORE del led 1 nel sistema target
	Vector3d P2_Tn = P2_T + Vector3d(-0.3, 1, -1); //Coordinate con RUMORE del led 2 nel sistema target
	Vector3d P3_Tn = P3_T + Vector3d(2, -1.3, 0); //Coordinate con RUMORE del led 3 nel sistema target
	Vector3d P4_Tn = P4_T + Vector3d(0.1, -0.5, 0.8); //Coordinate con RUMORE del led 4 nel sistema target

	//Istante iniziale
	Vector3d T_0(0, 0, 2000);
	double yaw0 = 90 * M_PI / 180;		//in RAD
	double pitch0 = 0 * M_PI / 180;		//in RAD
	double roll0 = 180 * M_PI / 180;	//in RAD

	//temporary variables for R_0 matrix computing
	double cosYaw0 = cos(yaw0);
	double sinYaw0 = sin(yaw0);
	double cosPitch0 = cos(pitch0);
	double sinPitch0 = sin(pitch0);
	double cosRoll0 = cos(roll0);
	double sinRoll0 = sin(roll0);

	double R11_0 = cosYaw0*cosPitch0;
	double R12_0 = sinYaw0*cosPitch0;
	double R13_0 = -sinPitch0;
	double R21_0 = cosYaw0*sinPitch0*sinRoll0 - sinYaw0*cosRoll0;
	double R22_0 = sinYaw0*sinPitch0*sinRoll0 + cosYaw0*cosRoll0;
	double R23_0 = cosPitch0*sinRoll0;
	double R31_0 = cosYaw0*sinPitch0*cosRoll0 + sinYaw0*sinRoll0;
	double R32_0 = sinYaw0*sinPitch0*cosRoll0 - cosYaw0*sinRoll0;
	double R33_0 = cosPitch0*cosRoll0;

	//TODO: rename R_0 to R0, it will be overwritten after calling p3p_solver but it doesn't matter
	Matrix3d R0;
	R0 << R11_0, R12_0, R13_0, R21_0, R22_0, R23_0, R31_0, R32_0, R33_0;
	/*
	printf("\nR_0:");
	printMatrix(R_0, 3, 3);
	*/
	Vector3d f1d_0 = R0*(P1_Tn - T_0); // Versori nel sistema camera
	f1d_0.normalize();
	Vector3d f2d_0 = R0*(P2_Tn - T_0);
	f2d_0.normalize();
	Vector3d f3d_0 = R0*(P3_Tn - T_0);
	f3d_0.normalize();
	Vector3d f4d_0 = R0*(P4_Tn - T_0);
	f4d_0.normalize();

	//Risoluzione del punto iniziale tramite p3p
	Eigen::Matrix<double, 3, 4> P;
	P.col(0) = P1_T;
	P.col(1) = P2_T;
	P.col(2) = P3_T;
	P.col(3) = P4_T;

	Eigen::Matrix<double, 3, 4> fd_0;
	fd_0.col(0) = f1d_0;
	fd_0.col(1) = f2d_0;
	fd_0.col(2) = f3d_0;
	fd_0.col(3) = f4d_0;
	/*
	printf("\nP:");
	printMatrix(P, 3, 4);
	printf("\nfd_0:");
	printMatrix(fd_0, 3, 4);
	*/

	Eigen::Matrix<double, 3, 4> sol = p3p_solver(P, fd_0);	
	T_0 = sol.col(0);
	R0.col(0) = sol.col(1);
	R0.col(1) = sol.col(2);
	R0.col(2) = sol.col(3);
	/*
	printf("\nT_0: ");
	printMatrix(T_0, 3, 1);
	printf("\nR0 :");
	printMatrix(R0, 3, 3);
	*/
	double ypr[3];
	dcm_to_ypr(R0,ypr);	//ypr = vector containing yaw, pitch, roll in DEGREES!!!
	/*
	printf("\nyaw: %f", (double)ypr[0]);
	printf("\npitch: %f", (double)ypr[1]);
	printf("\nroll: %f", (double)ypr[2]);
	*/

	Vector3d T_s = T_0;
	double Yaw_s = yaw0;
	double Pitch_s = pitch0;
	double Roll_s = roll0;

	//Moto
	double Yaw_nt[1001];
	double Pitch_nt[1001];
	double Roll_nt[1001];
	Eigen::Matrix<double, 3, 1001> T_out;
	Eigen::Matrix<double, 1, 1001> Yaw_out;
	Eigen::Matrix<double, 1, 1001> Pitch_out;
	Eigen::Matrix<double, 1, 1001> Roll_out;
	Eigen::Matrix<double, 1, 1001> Time_out;
	//int iter = 0;
	for (int i = 0; i <= 1000; i++) {

		double t = i / 0.01;
		int Periodo = 2; //secondi
		double w = 2 * M_PI / Periodo;

		double oscillation = (2 * M_PI / 180)*cos(w*t);

		double yaw = ypr[0] * M_PI / 180;
		double pitch = 0 * M_PI / 180;
		double roll = ypr[2] * M_PI / 180 + oscillation;

		Yaw_nt[i] = yaw;				//in DEG
		Pitch_nt[i] = pitch;					//in DEG 
		Roll_nt[i] = roll;	//in DEG
		/*
		printf("\nyaw: %f", (double)Yaw_nt[iter-1]);
		printf("\npitch: %f", (double)Pitch_nt[iter - 1]);
		printf("\nroll: %f", (double)Roll_nt[iter - 1]);
		printf("\noscillation: %f", oscillation);
		*/
		Eigen::Matrix<double, 3, 1001> T_nt;
		T_nt.col(i) = T_0;

		double cosYaw = cos(yaw);
		double sinYaw = sin(yaw);
		double cosPitch = cos(pitch);
		double sinPitch = sin(pitch);
		double cosRoll = cos(roll);
		double sinRoll = sin(roll);

		double R_nt11 = cosYaw*cosPitch;
		double R_nt12 = sinYaw*cosPitch;
		double R_nt13 = -sinPitch;
		double R_nt21 = cosYaw*sinPitch*sinRoll - sinYaw*cosRoll;
		double R_nt22 = sinYaw*sinPitch*sinRoll + cosYaw*cosRoll;
		double R_nt23 = cosPitch*sinRoll;
		double R_nt31 = cosYaw*sinPitch*cosRoll + sinYaw*sinRoll;
		double R_nt32 = sinYaw*sinPitch*cosRoll - cosYaw*sinRoll;
		double R_nt33 = cosPitch*cosRoll;

		Eigen::Matrix<double, 3, 3> R_nt;
		R_nt << R_nt11, R_nt12, R_nt13, R_nt21, R_nt22, R_nt23, R_nt31, R_nt32, R_nt33;

		//printf("\R_nt:");
		//printMatrix(R_nt, 3, 3);

		// Parametri:
		double focal = 3.46031;
		double d_pxl = 1.4e-3;
		double qx = q_d[0];
		double qy = q_d[1];
		double qz = q_d[2];

		//printf("\nq_d:");
		//printMatrix(q_d, 3, 1);

		Vector3d v = -(R_nt*T_nt.col(i) + q_dn);
		v.normalize();
		double vx = v(0);
		double vy = v(1);
		double vz = v(2);

		//printf("\nv:");
		//printMatrix(v, 3, 1);
		/*
		printf("\nT_nt.col:");
		printMatrix(T_nt.col(i), 3, 1);
		printf("\nP1_Tn:");
		printMatrix(P1_Tn, 3, 1);
		*/

		//TODO: computes three entire vectors only to keep three values, change
		Vector3d z1 = (R_nt*(T_nt.col(i) + P1_Tn)); // Riferimento drone
		double Z1 = z1(2);			  
		Vector3d z2 = (R_nt*(T_nt.col(i) + P2_Tn)); // Riferimento drone
		double Z2 = z2(2);			  
		Vector3d z3 = (R_nt*(T_nt.col(i) + P3_Tn)); // Riferimento drone
		double Z3 = z3(2);			  
		Vector3d z4 = (R_nt*(T_nt.col(i) + P4_Tn)); // Riferimento drone
		double Z4 = z4(2);
		/*
		if (i == 0) {
			printf("\nZ1: %f",Z1);
			printf("\nZ2: %f",Z2);
			printf("\nZ3: %f",Z3);
			printf("\nZ4: %f",Z4);
		}
		*/
		Eigen::Matrix<double, 2, 3> sparseFocal;
		sparseFocal << focal, 0, 0, 0, focal, 0;
		Eigen::Vector2d PXL1 = (1 / (Z1*d_pxl)) * sparseFocal * (R_nt*(T_nt.col(i) + P1_Tn));
		Eigen::Vector2d PXL2 = (1 / (Z2*d_pxl)) * sparseFocal * (R_nt*(T_nt.col(i) + P2_Tn));
		Eigen::Vector2d PXL3 = (1 / (Z3*d_pxl)) * sparseFocal * (R_nt*(T_nt.col(i) + P3_Tn));
		Eigen::Vector2d PXL4 = (1 / (Z4*d_pxl)) * sparseFocal * (R_nt*(T_nt.col(i) + P4_Tn));

		/*
		printf("\nPXL1:");
		printMatrix(PXL1, 2, 1);
		printf("\nPXL2:");
		printMatrix(PXL2, 2, 1);
		printf("\nPXL3:");
		printMatrix(PXL3, 2, 1);
		printf("\nPXL4:");
		printMatrix(PXL4, 2, 1);
		*/
		double x_pxl_1 = PXL1(0);
		double y_pxl_1 = PXL1(1);
		double x_pxl_2 = PXL2(0);
		double y_pxl_2 = PXL2(1);
		double x_pxl_3 = PXL3(0);
		double y_pxl_3 = PXL3(1);
		double x_pxl_4 = PXL4(0);
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

		Eigen::Matrix<double,6,1> X0;
		X0 << T_s(0), T_s(1), T_s(2), Yaw_s, Pitch_s, Roll_s;  // Condizioni iniziali PinHole;
		/*
		if (i == 0) {
			printf("\nX0 input:");
			printMatrix(X0, 6, 1);
		}
		*/
		pinHoleFSolve(X0, q_d.data(), v.data(), PXL1.data(), PXL2.data(), PXL3.data(), PXL4.data(), P1_T.data(), P2_T.data(), P3_T.data(), P4_T.data(), focal, d_pxl);
		/*
		if (i == 0) {
			printf("\nX0 output:");
			printMatrix(X0, 6, 1);
		}
		*/
		T_s << X0(0),X0(1),X0(2);
		Yaw_s = X0(3);
		Pitch_s = X0(4);
		Roll_s = X0(5);

		// Output:
		T_out.col(i) = T_s;
		Yaw_out(i) = Yaw_s;
		Pitch_out(i) = Pitch_s;
		Roll_out(i) = Roll_s;
		Time_out(i) = t;
	}

	//plots missing!
}
