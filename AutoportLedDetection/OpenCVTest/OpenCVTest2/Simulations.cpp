#include "stdafx.h"
#include <stdio.h>
#include <time.h>
#include <chrono>
#include <Eigen\Dense>
#include <unsupported\Eigen\NonLinearOptimization>
#include <unsupported\Eigen\NumericalDiff>
#include "P3p.h"
#include "Functions.h"

using namespace Eigen;

double simulazioneCompleta() {

	//Vector3d q_d(400, 0, 0);		//Coordinate del Quadrant Detector nel sistema drone
	Vector3d P1_T(500, -500, 0);	//Coordinate del led 1 nel sistema target
	Vector3d P2_T(500,  500, 0);	//Coordinate del led 2 nel sistema target
	Vector3d P3_T(  0,  500, 0);	//Coordinate del led 3 nel sistema target
	Vector3d P4_T(-500, 500, 0);	//Coordinate del led 4 nel sistema target

	//Vector3d q_dn = q_d + Vector3d(0.1, 0.1, 0);		//Coordinate con RUMORE del Quadrant Detector nel sistema drone
	Vector3d P1_Tn = P1_T + Vector3d( 0.1, -0.1, 0.2);	//Coordinate con RUMORE del led 1 nel sistema target
	Vector3d P2_Tn = P2_T + Vector3d(-0.3,    1,  -1);	//Coordinate con RUMORE del led 2 nel sistema target
	Vector3d P3_Tn = P3_T + Vector3d(   2, -1.3,   0);	//Coordinate con RUMORE del led 3 nel sistema target
	Vector3d P4_Tn = P4_T + Vector3d( 0.1, -0.5, 0.8);	//Coordinate con RUMORE del led 4 nel sistema target

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

	Matrix<double, 3, 4> fd_0;
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

	Matrix<double, 3, 4> sol = p3p_solver(P, fd_0);
	//T_0 = sol.col(0);
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
	dcm_to_ypr(R0, ypr);	//ypr = vector containing yaw, pitch, roll in DEGREES!!!
	/*
	printf("\nyaw: %f", (double)ypr[0]);
	printf("\npitch: %f", (double)ypr[1]);
	printf("\nroll: %f", (double)ypr[2]);
	*/

	Vector3d T_s = sol.col(0); //Vector3d T_s = T_0;
	double Yaw_s = yaw0;
	double Pitch_s = pitch0;
	double Roll_s = roll0;

	//Moto
	double Yaw_nt[1001];
	double Pitch_nt[1001];
	double Roll_nt[1001];
	Matrix<double, 3, 1001> T_out;
	Matrix<double, 1, 1001> Yaw_out;
	Matrix<double, 1, 1001> Pitch_out;
	Matrix<double, 1, 1001> Roll_out;
	Matrix<double, 1, 1001> Time_out;
	//int iter = 0;
	double totalTime = 0;
	for (int i = 0; i <= 1000; i++) {

		double t = i / 0.01;
		int Periodo = 2; //secondi
		double w = 2 * M_PI / Periodo;

		double oscillation = (2 * M_PI / 180)*cos(w*t);

		double yaw = ypr[0] * M_PI / 180;
		double pitch = 0 * M_PI / 180;
		double roll = ypr[2] * M_PI / 180 + oscillation;

		Yaw_nt[i] = yaw;		//in DEG
		Pitch_nt[i] = pitch;	//in DEG 
		Roll_nt[i] = roll;		//in DEG
		/*
		printf("\nyaw: %f", (double)Yaw_nt[iter-1]);
		printf("\npitch: %f", (double)Pitch_nt[iter - 1]);
		printf("\nroll: %f", (double)Roll_nt[iter - 1]);
		printf("\noscillation: %f", oscillation);
		*/
		Matrix<double, 3, 1001> T_nt;
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

		Matrix<double, 3, 3> R_nt;
		R_nt << R_nt11, R_nt12, R_nt13, R_nt21, R_nt22, R_nt23, R_nt31, R_nt32, R_nt33;

		//printf("\R_nt:");
		//printMatrix(R_nt, 3, 3);

		// Parametri:
		double focal = 3.46031;
		double d_pxl = 1.4e-3;
		//double qx = q_d[0];
		//double qy = q_d[1];
		//double qz = q_d[2];

		//printf("\nq_d:");
		//printMatrix(q_d, 3, 1);

		Vector3d v = -(R_nt*T_nt.col(i)); //+ q_dn);
		v.normalize();
		//double vx = v(0);
		//double vy = v(1);
		//double vz = v(2);

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
		Matrix<double, 2, 3> sparseFocal;
		sparseFocal << focal, 0, 0, 0, focal, 0;
		Vector2d PXL1 = (1 / (Z1*d_pxl)) * sparseFocal * (R_nt*(T_nt.col(i) + P1_Tn));
		Vector2d PXL2 = (1 / (Z2*d_pxl)) * sparseFocal * (R_nt*(T_nt.col(i) + P2_Tn));
		Vector2d PXL3 = (1 / (Z3*d_pxl)) * sparseFocal * (R_nt*(T_nt.col(i) + P3_Tn));
		Vector2d PXL4 = (1 / (Z4*d_pxl)) * sparseFocal * (R_nt*(T_nt.col(i) + P4_Tn));
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

		Matrix<double, 6, 1> X0;
		X0 << T_s(0), T_s(1), T_s(2), Yaw_s, Pitch_s, Roll_s;  // Condizioni iniziali PinHole;
		/*
		if (i == 0) {
		printf("\nX0 input:");
		printMatrix(X0, 6, 1);
		}
		*/

		long time = pinHoleFSolve(X0, v.data(), PXL1.data(), PXL2.data(), PXL3.data(), PXL4.data(), P1_T.data(), P2_T.data(), P3_T.data(), P4_T.data(), focal, d_pxl);
		totalTime += (double)time;
		/*
		if (i == 0) {
		printf("\nX0 output:");
		printMatrix(X0, 6, 1);
		}
		*/
		T_s << X0(0), X0(1), X0(2);
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
	double meanTime = totalTime / 1001;
	return meanTime;//plots missing!
}


/*
long simulazioneUS() {
	Eigen::Matrix<double, 6, 1> X0 = Eigen::Matrix<double, 6, 1>::Random();
	Vector3d q_d = Vector3d::Random();
	Vector2d PXL1 = Vector2d::Random();
	Vector2d PXL2 = Vector2d::Random();
	Vector2d PXL3 = Vector2d::Random();
	Vector2d PXL4 = Vector2d::Random();
	Vector3d P1_T = Vector3d::Random();
	Vector3d P2_T = Vector3d::Random();
	Vector3d P3_T = Vector3d::Random();
	Vector3d P4_T = Vector3d::Random();
	Vector3d delta = Vector3d::Random();
	Vector3d d1 = Vector3d::Random();
	Vector3d d2 = Vector3d::Random();
	Vector3d d3 = Vector3d::Random();
	Vector3d d4 = Vector3d::Random();
	double focal = 3.46031;
	double d_pxl = 1.4e-3;
	double v = 343800; // mm / s
					   //double* q_d, double* PXL1, double* PXL2, double* PXL3, double* PXL4, double* P1_T, double* P2_T, double* P3_T, double* P4_T, double* delta, double* d1, double* d2, double* d3, double* d4, double focal, double d_pxl, double v
	long time = pinHoleFSolveUS(X0, q_d.data(), PXL1.data(), PXL2.data(), PXL3.data(), PXL4.data(), P1_T.data(), P2_T.data(), P3_T.data(), P4_T.data(), delta.data(), d1.data(), d2.data(), d3.data(), d4.data(), focal, d_pxl, v);
	return time;

}
*/