
#include "stdafx.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <complex>
#include "../Eigen/Eigen/Dense"

#include "P3p.h"
#include "Functions.h"

using Eigen::Vector3d;
using Eigen::Matrix3d;

void SimulazioneCompleta() {
	Vector3d q_d( 400, 0, 0 );			//Coordinate del Quadrant Detector nel sistema drone
	Vector3d P1_T( 500, -500, 0);	//Coordinate del led 1 nel sistema target
	Vector3d P2_T( 500,  500, 0);	//Coordinate del led 2 nel sistema target
	Vector3d P3_T(   0,  500, 0);	//Coordinate del led 3 nel sistema target
	Vector3d P4_T(-500,  500, 0);	//Coordinate del led 4 nel sistema target
	
	Vector3d q_dn = q_d + Vector3d(0, 0, 0);		 //Coordinate con RUMORE del Quadrant Detector nel sistema drone
	Vector3d P1_Tn = P1_T + Vector3d(0, 0, 0); //Coordinate con RUMORE del led 1 nel sistema target
	Vector3d P2_Tn = P2_T + Vector3d(0, 0, 0); //Coordinate con RUMORE del led 2 nel sistema target
	Vector3d P3_Tn = P3_T + Vector3d(0, 0, 0); //Coordinate con RUMORE del led 3 nel sistema target
	Vector3d P4_Tn = P4_T + Vector3d(0, 0, 0); //Coordinate con RUMORE del led 4 nel sistema target

	//Istante iniziale
	Vector3d T_0(0, 0, 200);
	double yaw0   = 90  * M_PI / 180;
	double pitch0 = 0   * M_PI / 180;
	double roll0 = 180  * M_PI / 180;

	double R11_0 = cos(yaw0)*cos(pitch0);
	double R12_0 = sin(yaw0)*cos(pitch0);
	double R13_0 = -sin(pitch0);
	double R21_0 = cos(yaw0)*sin(pitch0)*sin(roll0) - sin(yaw0)*cos(roll0);
	double R22_0 = sin(yaw0)*sin(pitch0)*sin(roll0) + cos(yaw0)*cos(roll0);
	double R23_0 = cos(pitch0)*sin(roll0);
	double R31_0 = cos(yaw0)*sin(pitch0)*cos(roll0) + sin(yaw0)*sin(roll0);
	double R32_0 = sin(yaw0)*sin(pitch0)*cos(roll0) - cos(yaw0)*sin(roll0);
	double R33_0 = cos(pitch0)*cos(roll0);

	Matrix3d R_0;
	R_0 << R11_0, R12_0, R13_0, R21_0, R22_0, R23_0, R31_0, R32_0, R33_0;

	Vector3d f1d_0 = R_0*(P1_Tn - T_0); // Versori nel sistema camera
	f1d_0.normalize();
	Vector3d f2d_0 = R_0*(P2_Tn - T_0);
	f2d_0.normalize();
	Vector3d f3d_0 = R_0*(P3_Tn - T_0);
	f3d_0.normalize();
	Vector3d f4d_0 = R_0*(P4_Tn - T_0);
	f4d_0.normalize();

	//Risoluzione del punto iniziale tramite p3p
	Eigen::Matrix<double, 3, 4> P;
	P.col(0) = P1_T;
	P.col(1) = P2_T;
	P.col(3) = P3_T;
	P.col(4) = P4_T;

	Eigen::Matrix<double, 3, 4> fd_0;
	fd_0.col(0) = f1d_0;
	fd_0.col(1) = f2d_0;
	fd_0.col(2) = f3d_0;
	fd_0.col(3) = f4d_0;

	//TODO define p3p_solver_new
	Eigen::Matrix<double, 3, 4> sol = p3p_solver(P, fd_0);	//T_0 is a 3x1 vector, for R see matlab file "p3p_solver_new.m" (should be 3x3)
	T_0 = sol.col(0);
	Eigen::Matrix3d R0;
	R0.col(0) = sol.col(1);
	R0.col(1) = sol.col(2);
	R0.col(2) = sol.col(3);

	//Written like this is a bit confusing... Find a better way
	double *ypr;	//array of length 3
	ypr = dcm_to_ypr(R_0);	//ypr = vector containing yaw, pitch, roll IN RADIANS!!!

	Vector3d T_s = T_0;
	double Yaw_s = yaw0 * M_PI / 180;
	double Pitch_s = pitch0 *M_PI / 180;
	double Roll_s = roll0 *M_PI / 180;

	//Moto
	double Yaw_nt[1001];
	double Pitch_nt[1001];
	double Roll_nt[1001];
	Eigen::Matrix<double, 3, 1001> T_out;
	Eigen::Matrix<double, 1, 1001> Yaw_out;
	Eigen::Matrix<double, 1, 1001> Pitch_out;
	Eigen::Matrix<double, 1, 1001> Roll_out;
	Eigen::Matrix<double, 1, 1001> Time_out;
	int iter = 0;
	for (int i = 0; i <= 1000; i++) {
		double t = i / 0.01;

		//what can we do to replace these global variables?
		/*
		global qx qy qz vx vy vz ...
			x_pxl_1 y_pxl_1 ... // Coordinate dei led nei pixel della cam
			x_pxl_2 y_pxl_2 ...
			x_pxl_3 y_pxl_3 ...
			x_pxl_4 y_pxl_4 ...
			Px_1 Py_1 Pz_1 ... // Coordinate del led nel sistema target
			Px_2 Py_2 Pz_2 ...
			Px_3 Py_3 Pz_3 ...
			Px_4 Py_4 Pz_4 ...
			focal d_pxl // focale cam
		*/

		iter++;
		int Periodo = 2; //secondi
		double w = 2 * M_PI / Periodo;

		//ypr is already in radians!! modify these lines
		Yaw_nt[iter-1]   = ypr[0] * M_PI / 180; // rad
		Pitch_nt[iter - 1] = 0;					 // rad
		Roll_nt[iter - 1]  = ypr[2] * M_PI / 180 + (2 * M_PI / 180)*cos(w*t); // rad

		Eigen::Matrix<double, 3, 1001> T_nt;
		T_nt.col(iter) = T_0;

		double R_nt11 =  cos(Yaw_nt[iter-1])*cos(Pitch_nt[iter-1]);
		double R_nt12 =  sin(Yaw_nt[iter-1])*cos(Pitch_nt[iter-1]);
		double R_nt13 = -sin(Pitch_nt[iter - 1]);
		double R_nt21 =  cos(Yaw_nt[iter-1])*sin(Pitch_nt[iter-1])*sin(Roll_nt[iter-1]) - sin(Yaw_nt[iter-1])*cos(Roll_nt[iter-1]);
		double R_nt22 =  sin(Yaw_nt[iter-1])*sin(Pitch_nt[iter-1])*sin(Roll_nt[iter-1]) + cos(Yaw_nt[iter-1])*cos(Roll_nt[iter-1]);
		double R_nt23 =  cos(Pitch_nt[iter - 1])*sin(Roll_nt[iter - 1]);
		double R_nt31 =  cos(Yaw_nt[iter-1])*sin(Pitch_nt[iter-1])*cos(Roll_nt[iter-1]) + sin(Yaw_nt[iter-1])*sin(Roll_nt[iter-1]);
		double R_nt32 =  sin(Yaw_nt[iter-1])*sin(Pitch_nt[iter-1])*cos(Roll_nt[iter-1]) - cos(Yaw_nt[iter-1])*sin(Roll_nt[iter-1]);
		double R_nt33 =  cos(Pitch_nt[iter - 1])*cos(Roll_nt[iter - 1]);

		Eigen::Matrix<double, 3, 3> R_nt;
		R_nt << R_nt11, R_nt12, R_nt13, R_nt21, R_nt22, R_nt23, R_nt31, R_nt32, R_nt33;

		// Parametri:
		double focal = 3.46031;
		double d_pxl = 1.4e-3;
		double qx = q_d[0]; 
		double qy = q_d[1]; 
		double qz = q_d[2];

		Vector3d v = -(R_nt*T_nt.col(iter - 1) + q_dn);
		v.normalize();
		double vx = v(0); 
		double vy = v(1); 
		double vz = v(2);

		//TODO: computes three entire vectors only to keep three values, change
		Vector3d z1 = (R_nt*(T_nt.col(iter-1) + P1_Tn)); // Riferimento drone
		double Z1 = z1(2);				  
		Vector3d z2 = (R_nt*(T_nt.col(iter-1) + P2_Tn)); // Riferimento drone
		double Z2 = z2(2);				  
		Vector3d z3 = (R_nt*(T_nt.col(iter-1) + P3_Tn)); // Riferimento drone
		double Z3 = z3(2);				  
		Vector3d z4 = (R_nt*(T_nt.col(iter-1) + P4_Tn)); // Riferimento drone
		double Z4 = z4(2);

		Eigen::Matrix<double, 2, 3> sparseFocal;
		sparseFocal << focal, 0, 0, 0, focal, 0;
		Eigen::Vector2d PXL1 = (1 / (Z1*d_pxl)) * sparseFocal * (R_nt*(T_nt.col(iter-1) + P1_Tn));
		Eigen::Vector2d PXL2 = (1 / (Z2*d_pxl)) * sparseFocal * (R_nt*(T_nt.col(iter-1) + P2_Tn));
		Eigen::Vector2d PXL3 = (1 / (Z3*d_pxl)) * sparseFocal * (R_nt*(T_nt.col(iter-1) + P3_Tn));
		Eigen::Vector2d PXL4 = (1 / (Z4*d_pxl)) * sparseFocal * (R_nt*(T_nt.col(iter-1) + P4_Tn));
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

		//TODO write fsolve
		Eigen::VectorXd X0;
		Eigen::VectorXd Y0;
		X0 << T_s(0), T_s(1), T_s(2), Yaw_s, Pitch_s, Roll_s;  // Condizioni iniziali PinHole;
		pinHoleFSolve(X0, Y0, q_d.data(), v.data(), PXL1.data(), PXL2.data(), PXL3.data(), PXL4.data(), P1_T.data(), P2_T.data(), P3_T.data(), P4_T.data(), focal, d_pxl);
		T_s = X0.block(0,0,2,0);
		Yaw_s = X0(3);
		Pitch_s = X0(4);
		Roll_s = X0(5);

		// Output:
		T_out.col(iter-1) = T_s;
		Yaw_out(iter-1)   = Yaw_s;
		Pitch_out(iter-1) = Pitch_s;
		Roll_out(iter-1)  = Roll_s;
		Time_out(iter-1)  = t;
	}

	//plots missing!
}
