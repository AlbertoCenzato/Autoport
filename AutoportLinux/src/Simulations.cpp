//Copyright (c) 2016 Alberto Cenzato. All rights reserved.

#include <stdio.h>
#include <time.h>
#include <chrono>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

#include "GenPurpFunc.h"
#include "P3p.h"

using namespace Eigen;
using namespace cv;

/*
double simulazioneCompleta() {

	//Vector3d q_d(400, 0, 0);		//Coordinate del Quadrant Detector nel sistema drone
	Point3d P1_T(500, -500, 0);	//Coordinate del led 1 nel sistema target
	Point3d P2_T(500,  500, 0);	//Coordinate del led 2 nel sistema target
	Point3d P3_T(  0,  500, 0);	//Coordinate del led 3 nel sistema target
	Point3d P4_T(-500, 500, 0);	//Coordinate del led 4 nel sistema target

	//Vector3d q_dn = q_d + Vector3d(0.1, 0.1, 0);		//Coordinate con RUMORE del Quadrant Detector nel sistema drone
	Point3d P1_Tn = P1_T + Point3d( 0.1, -0.1, 0.2);	//Coordinate con RUMORE del led 1 nel sistema target
	Point3d P2_Tn = P2_T + Point3d(-0.3,    1,  -1);	//Coordinate con RUMORE del led 2 nel sistema target
	Point3d P3_Tn = P3_T + Point3d(   2, -1.3,   0);	//Coordinate con RUMORE del led 3 nel sistema target
	Point3d P4_Tn = P4_T + Point3d( 0.1, -0.5, 0.8);	//Coordinate con RUMORE del led 4 nel sistema target

	//Istante iniziale
	Point3d T_0(0, 0, 2000);
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

	Matrix3d R0;
	R0 << R11_0, R12_0, R13_0, R21_0, R22_0, R23_0, R31_0, R32_0, R33_0;
	
	Point3d f0d_0 = multiply3d(R0, P1_Tn - T_0); // Versori nel sistema camera
	Point3d f1d_0 = multiply3d(R0, P2_Tn - T_0);
	Point3d f2d_0 = multiply3d(R0, P3_Tn - T_0);
	Point3d f3d_0 = multiply3d(R0, P4_Tn - T_0);
	f0d_0 = normalize(f0d_0);
	f1d_0 = normalize(f1d_0);
	f2d_0 = normalize(f2d_0);
	f3d_0 = normalize(f3d_0);

	//Risoluzione del punto iniziale tramite p3p
	Point3d points[4];
	points[0] = P1_T;
	points[1] = P2_T;
	points[2] = P3_T;
	points[3] = P4_T;

	Point3d vers[4];
	vers[0] = f0d_0;
	vers[1] = f1d_0;
	vers[2] = f2d_0;
	vers[3] = f3d_0;

	Matrix<double, 3, 4> sol = p3p_solver(points, vers);
	//T_0 = sol.col(0);
	R0.col(0) = sol.col(1);
	R0.col(1) = sol.col(2);
	R0.col(2) = sol.col(3);
	
	double ypr[3];
	dcm_to_ypr(R0, ypr);	//ypr = vector containing yaw, pitch, roll in DEGREES!!!

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

		Point3d v = -multiply3d(R_nt,T_0);//-(R_nt*T_nt.col(i)); //+ q_dn);
		v = normalize(v);
		//double vx = v(0);
		//double vy = v(1);
		//double vz = v(2);

		//printf("\nv:");
		//printMatrix(v, 3, 1);

		//TODO: computes three entire vectors only to keep three values, change
		Point3d z0 = multiply3d(R_nt,(T_0 + P1_Tn)); // Riferimento drone
		Point3d z1 = multiply3d(R_nt,(T_0 + P2_Tn)); // Riferimento drone
		Point3d z2 = multiply3d(R_nt,(T_0 + P3_Tn)); // Riferimento drone
		Point3d z3 = multiply3d(R_nt,(T_0 + P4_Tn)); // Riferimento drone
		double Z0 = z0.z;
		double Z1 = z1.z;
		double Z2 = z2.z;
		double Z3 = z3.z;
		
		Matrix<double, 2, 3> sparseFocal;
		sparseFocal << focal, 0, 0, 0, focal, 0;
		Vector2d PXL0 = (1 / (Z0*d_pxl)) * sparseFocal * (R_nt*(T_nt.col(i) + P0_Tn));
		Vector2d PXL1 = (1 / (Z1*d_pxl)) * sparseFocal * (R_nt*(T_nt.col(i) + P1_Tn));
		Vector2d PXL2 = (1 / (Z2*d_pxl)) * sparseFocal * (R_nt*(T_nt.col(i) + P2_Tn));
		Vector2d PXL3 = (1 / (Z3*d_pxl)) * sparseFocal * (R_nt*(T_nt.col(i) + P3_Tn));
		Vector2d PXL4 = (1 / (Z4*d_pxl)) * sparseFocal * (R_nt*(T_nt.col(i) + P4_Tn));
		Vector2d PXL5 = (1 / (Z5*d_pxl)) * sparseFocal * (R_nt*(T_nt.col(i) + P5_Tn));
		Vector2d PXL6 = (1 / (Z6*d_pxl)) * sparseFocal * (R_nt*(T_nt.col(i) + P6_Tn));
		Vector2d PXL7 = (1 / (Z7*d_pxl)) * sparseFocal * (R_nt*(T_nt.col(i) + P7_Tn));

		Point2f leds[8];
		Point2f leds[0] = Point2f(PXL0(0),PXL0(1));
		Point2f leds[1] = Point2f(PXL1(0),PXL1(1));
		Point2f leds[2] = Point2f(PXL2(0),PXL2(1));
		Point2f leds[3] = Point2f(PXL3(0),PXL3(1));
		Point2f leds[4] = Point2f(PXL4(0),PXL4(1));
		Point2f leds[5] = Point2f(PXL5(0),PXL5(1));
		Point2f leds[6] = Point2f(PXL6(0),PXL6(1));
		Point2f leds[7] = Point2f(PXL7(0),PXL7(1));
		
		Point3f points[8];
		Point3f points[0] = Point3f(P0_T(0), P0_T(1), P0_T(2));
		Point3f points[1] = Point3f(P1_T(0), P1_T(1), P1_T(2));
		Point3f points[2] = Point3f(P2_T(0), P2_T(1), P2_T(2));
		Point3f points[3] = Point3f(P3_T(0), P3_T(1), P3_T(2));
		Point3f points[4] = Point3f(P4_T(0), P4_T(1), P4_T(2));
		Point3f points[5] = Point3f(P5_T(0), P5_T(1), P5_T(2));
		Point3f points[6] = Point3f(P6_T(0), P6_T(1), P6_T(2));
		Point3f points[7] = Point3f(P7_T(0), P7_T(1), P7_T(2));

		Matrix<double, 6, 1> X0;
		X0 << T_s(0), T_s(1), T_s(2), Yaw_s, Pitch_s, Roll_s;  // Condizioni iniziali PinHole;

		long time = pinHoleFSolve(X0, v.data(), leds, points, focal, d_pxl);
		totalTime += (double)time;

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
*/
