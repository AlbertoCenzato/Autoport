#include "stdafx.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <complex>
#include "../Eigen/Eigen/Dense"
#include "../Eigen/Eigen/Geometry"

#include "P3p.h"

using Eigen::Vector3d;
using Eigen::Matrix3d;

const double PI = 3.141592653589793;

void SimulazioneCompleta() {
	Vector3d q_d(400, 0, 0);			//Coordinate del Quadrant Detector nel sistema drone
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
	double yaw0   = 90  * PI / 180;
	double pitch0 = 0   * PI / 180;
	double roll0 = 180  * PI / 180;

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
	[T_0, R_0] = p3p_solver_new(P, fd_0);	//T_0 is a 3x4 matrix, for R see matlab file "p3p_solver_new.m" (should be 3x3)

	//TODO define dcm_to_ypr
	double ypr[3];
	ypr = dcm_to_ypr(R_0);	//ypr = vector containing yaw, pitch, roll
	//[Yaw_0, Pitch_0, Roll_0] = dcm_to_ypr(R_0);	original line of code

	Eigen::Matrix<double,3,4> T_s = T_0;
	double Yaw_s = yaw0 * PI / 180;
	double Pitch_s = pitch0 *PI / 180;
	double Roll_s = roll0 *PI / 180;

	//Moto
	double Yaw_nt[1001];
	double Pitch_nt[1001];
	double Roll_nt[1001];
	int iter = 0;
	for (int i = 0; i <= 1000; i++) {
		float t = i / 0.01;
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

		iter++;
		int Periodo = 2; //secondi
		int w = 2 * PI / Periodo;

		Yaw_nt[iter]   = ypr[0] * PI / 180; // rad
		Pitch_nt[iter] = 0;					 // rad
		Roll_nt[iter]  = ypr[2] * PI / 180 + (2 * PI / 180)*cos(w*t); // rad

		T_nt(:, iter) = T_0;

		R_nt11 = cos(Yaw_nt(iter))*cos(Pitch_nt(iter));
		R_nt12 = sin(Yaw_nt(iter))*cos(Pitch_nt(iter));
		R_nt13 = -sin(Pitch_nt(iter));
		R_nt21 = cos(Yaw_nt(iter))*sin(Pitch_nt(iter))*sin(Roll_nt(iter)) - sin(Yaw_nt(iter))*cos(Roll_nt(iter));
		R_nt22 = sin(Yaw_nt(iter))*sin(Pitch_nt(iter))*sin(Roll_nt(iter)) + cos(Yaw_nt(iter))*cos(Roll_nt(iter));
		R_nt23 = cos(Pitch_nt(iter))*sin(Roll_nt(iter));
		R_nt31 = cos(Yaw_nt(iter))*sin(Pitch_nt(iter))*cos(Roll_nt(iter)) + sin(Yaw_nt(iter))*sin(Roll_nt(iter));
		R_nt32 = sin(Yaw_nt(iter))*sin(Pitch_nt(iter))*cos(Roll_nt(iter)) - cos(Yaw_nt(iter))*sin(Roll_nt(iter));
		R_nt33 = cos(Pitch_nt(iter))*cos(Roll_nt(iter));

		R_nt = [R_nt11 R_nt12 R_nt13; R_nt21 R_nt22 R_nt23; R_nt31 R_nt32 R_nt33];

		% Parametri:

		focal = 3.46031;
		d_pxl = 1.4e-3;
		qx = q_d(1); qy = q_d(2); qz = q_d(3);

		v = -(R_nt*T_nt(:, iter) + q_dn) / norm(R_nt*T_nt(:, iter) + q_dn);
		vx = v(1); vy = v(2); vz = v(3);

		Z1 = (R_nt*(T_nt(:, iter) + P1_Tn))'*[0 0 1]'; % Riferimento drone
			Z2 = (R_nt*(T_nt(:, iter) + P2_Tn))'*[0 0 1]'; % Riferimento drone
			Z3 = (R_nt*(T_nt(:, iter) + P3_Tn))'*[0 0 1]'; % Riferimento drone
			Z4 = (R_nt*(T_nt(:, iter) + P4_Tn))'*[0 0 1]'; % Riferimento drone
			PXL1 = (1 / (Z1*d_pxl))*[focal 0 0; 0 focal 0] * (R_nt*(T_nt(:, iter) + P1_Tn));
		PXL2 = (1 / (Z2*d_pxl))*[focal 0 0; 0 focal 0] * (R_nt*(T_nt(:, iter) + P2_Tn));
		PXL3 = (1 / (Z3*d_pxl))*[focal 0 0; 0 focal 0] * (R_nt*(T_nt(:, iter) + P3_Tn));
		PXL4 = (1 / (Z4*d_pxl))*[focal 0 0; 0 focal 0] * (R_nt*(T_nt(:, iter) + P4_Tn));
		x_pxl_1 = PXL1(1);
		y_pxl_1 = PXL1(2);
		x_pxl_2 = PXL2(1);
		y_pxl_2 = PXL2(2);
		x_pxl_3 = PXL3(1);
		y_pxl_3 = PXL3(2);
		x_pxl_4 = PXL4(1);
		y_pxl_4 = PXL4(2);

		Px_1 = P1_T(1); Py_1 = P1_T(2); Pz_1 = P1_T(3);
		Px_2 = P2_T(1); Py_2 = P2_T(2); Pz_2 = P2_T(3);
		Px_3 = P3_T(1); Py_3 = P3_T(2); Pz_3 = P3_T(3);
		Px_4 = P4_T(1); Py_4 = P4_T(2); Pz_4 = P4_T(3);

		X0 = [T_s(1) T_s(2) T_s(3) Yaw_s Pitch_s Roll_s];  % Condizioni iniziali PinHole;
		options = optimset('Algorithm', 'levenberg-marquardt');
		[X] = fsolve(@PinHole,X0, options);
		T_s = X(1:3);
		Yaw_s = X(4);
		Pitch_s = X(5);
		Roll_s = X(6);

		% Output:
		T_out(:, iter) = T_s' ;
			Yaw_out(iter) = Yaw_s;
		Pitch_out(iter) = Pitch_s;
		Roll_out(iter) = Roll_s;
		Time_out(iter) = t;
	}
}



// Simulazione completa di QD
/*
clear all, clc, close all
q_d = [400; 0; 0];		% Coordinate del Quadrant Detector nel sitema drone
P1_T = [500; -500; 0];	% Coordinate del led 1 nel sistema target
P2_T = [500; 500; 0];	% Coordinate del led 2 nel sistema target
P3_T = [0; 500; 0];		% Coordinate del led 3 nel sistema target
P4_T = [-500; 500; 0];	% Coordinate del led 4 nel sistema target

q_dn = q_d + [.1; .1; 0]; % Coordinate con RUMORE del Quadrant Detector nel sitema drone
P1_Tn = P1_T + [.1; -.1; .2]; % Coordinate con RUMORE del led 1 nel sistema target
P2_Tn = P2_T + [-.3; 1; -1]; % Coordinate con RUMORE del led 2 nel sistema target
P3_Tn = P3_T + [2; -1.3; 0]; % Coordinate con RUMORE del led 3 nel sistema target
P4_Tn = P4_T + [.1; -.5; .8]; % Coordinate con RUMORE del led 4 nel sistema target



%% Istante iniziale

T_0 = [0; 0; 2000];

yaw0 = 90 * pi / 180;
pitch0 = 0 * pi / 180;
roll0 = 180 * pi / 180;

R11_0 = cos(yaw0)*cos(pitch0);
R12_0 = sin(yaw0)*cos(pitch0);
R13_0 = -sin(pitch0);
R21_0 = cos(yaw0)*sin(pitch0)*sin(roll0) - sin(yaw0)*cos(roll0);
R22_0 = sin(yaw0)*sin(pitch0)*sin(roll0) + cos(yaw0)*cos(roll0);
R23_0 = cos(pitch0)*sin(roll0);
R31_0 = cos(yaw0)*sin(pitch0)*cos(roll0) + sin(yaw0)*sin(roll0);
R32_0 = sin(yaw0)*sin(pitch0)*cos(roll0) - cos(yaw0)*sin(roll0);
R33_0 = cos(pitch0)*cos(roll0);



R_0 = [R11_0 R12_0 R13_0; ...
R21_0 R22_0 R23_0; ...
R31_0 R32_0 R33_0]; % Matrice di rotazione

f1d_0 = (R_0*(P1_Tn - T_0)) / norm(R_0*(P1_Tn - T_0)); % Versori nel sistema cam
f2d_0 = (R_0*(P2_Tn - T_0)) / norm(R_0*(P2_Tn - T_0));
f3d_0 = (R_0*(P3_Tn - T_0)) / norm(R_0*(P3_Tn - T_0));
f4d_0 = (R_0*(P4_Tn - T_0)) / norm(R_0*(P4_Tn - T_0));



% Risoluzione del punto iniziale tramite p3p
P = [P1_T P2_T P3_T P4_T];
fd_0 = [f1d_0 f2d_0 f3d_0 f4d_0];
[T_0, R_0] = p3p_solver_new(P, fd_0);
[Yaw_0, Pitch_0, Roll_0] = dcm_to_ypr(R_0);
T_s = T_0;
Yaw_s = yaw0 *pi / 180;
Pitch_s = pitch0 *pi / 180;
Roll_s = roll0 *pi / 180;
*/

%% Moto
iter = 0;
for t = 0 : .01 : 10
global qx qy qz vx vy vz ...
x_pxl_1 y_pxl_1 ... % Coordinate dei led nei pixel della cam
x_pxl_2 y_pxl_2 ...
x_pxl_3 y_pxl_3 ...
x_pxl_4 y_pxl_4 ...
Px_1 Py_1 Pz_1 ... % Coordinate del led nel sistema target
Px_2 Py_2 Pz_2 ...
Px_3 Py_3 Pz_3 ...
Px_4 Py_4 Pz_4 ...
focal d_pxl % focale cam
iter = iter + 1;
Periodo = 2; % secondi
w = 2 * pi / Periodo;

Yaw_nt(iter) = Yaw_0  *pi / 180; % rad
Pitch_nt(iter) = 0; % rad
Roll_nt(iter) = Roll_0  *pi / 180 + (2 * pi / 180)*cos(w*t); % rad

T_nt(:, iter) = T_0;

R_nt11 = cos(Yaw_nt(iter))*cos(Pitch_nt(iter));
R_nt12 = sin(Yaw_nt(iter))*cos(Pitch_nt(iter));
R_nt13 = -sin(Pitch_nt(iter));
R_nt21 = cos(Yaw_nt(iter))*sin(Pitch_nt(iter))*sin(Roll_nt(iter)) - sin(Yaw_nt(iter))*cos(Roll_nt(iter));
R_nt22 = sin(Yaw_nt(iter))*sin(Pitch_nt(iter))*sin(Roll_nt(iter)) + cos(Yaw_nt(iter))*cos(Roll_nt(iter));
R_nt23 = cos(Pitch_nt(iter))*sin(Roll_nt(iter));
R_nt31 = cos(Yaw_nt(iter))*sin(Pitch_nt(iter))*cos(Roll_nt(iter)) + sin(Yaw_nt(iter))*sin(Roll_nt(iter));
R_nt32 = sin(Yaw_nt(iter))*sin(Pitch_nt(iter))*cos(Roll_nt(iter)) - cos(Yaw_nt(iter))*sin(Roll_nt(iter));
R_nt33 = cos(Pitch_nt(iter))*cos(Roll_nt(iter));

R_nt = [R_nt11 R_nt12 R_nt13; R_nt21 R_nt22 R_nt23; R_nt31 R_nt32 R_nt33];

% Parametri:

focal = 3.46031;
d_pxl = 1.4e-3;
qx = q_d(1); qy = q_d(2); qz = q_d(3);

v = -(R_nt*T_nt(:, iter) + q_dn) / norm(R_nt*T_nt(:, iter) + q_dn);
vx = v(1); vy = v(2); vz = v(3);

Z1 = (R_nt*(T_nt(:, iter) + P1_Tn))'*[0 0 1]'; % Riferimento drone
Z2 = (R_nt*(T_nt(:, iter) + P2_Tn))'*[0 0 1]'; % Riferimento drone
Z3 = (R_nt*(T_nt(:, iter) + P3_Tn))'*[0 0 1]'; % Riferimento drone
Z4 = (R_nt*(T_nt(:, iter) + P4_Tn))'*[0 0 1]'; % Riferimento drone
PXL1 = (1 / (Z1*d_pxl))*[focal 0 0; 0 focal 0] * (R_nt*(T_nt(:, iter) + P1_Tn));
PXL2 = (1 / (Z2*d_pxl))*[focal 0 0; 0 focal 0] * (R_nt*(T_nt(:, iter) + P2_Tn));
PXL3 = (1 / (Z3*d_pxl))*[focal 0 0; 0 focal 0] * (R_nt*(T_nt(:, iter) + P3_Tn));
PXL4 = (1 / (Z4*d_pxl))*[focal 0 0; 0 focal 0] * (R_nt*(T_nt(:, iter) + P4_Tn));
x_pxl_1 = PXL1(1);
y_pxl_1 = PXL1(2);
x_pxl_2 = PXL2(1);
y_pxl_2 = PXL2(2);
x_pxl_3 = PXL3(1);
y_pxl_3 = PXL3(2);
x_pxl_4 = PXL4(1);
y_pxl_4 = PXL4(2);

Px_1 = P1_T(1); Py_1 = P1_T(2); Pz_1 = P1_T(3);
Px_2 = P2_T(1); Py_2 = P2_T(2); Pz_2 = P2_T(3);
Px_3 = P3_T(1); Py_3 = P3_T(2); Pz_3 = P3_T(3);
Px_4 = P4_T(1); Py_4 = P4_T(2); Pz_4 = P4_T(3);

X0 = [T_s(1) T_s(2) T_s(3) Yaw_s Pitch_s Roll_s];  % Condizioni iniziali PinHole;
options = optimset('Algorithm', 'levenberg-marquardt');
[X] = fsolve(@PinHole,X0, options);
T_s = X(1:3);
Yaw_s = X(4);
Pitch_s = X(5);
Roll_s = X(6);

% Output:
T_out(:, iter) = T_s' ;
Yaw_out(iter) = Yaw_s;
Pitch_out(iter) = Pitch_s;
Roll_out(iter) = Roll_s;
Time_out(iter) = t;
end

figure(1)
plot(Time_out, Roll_out * 180 / pi)
hold on
plot(Time_out, Roll_nt * 180 / pi)

figure(2)
plot(Time_out, abs(Roll_out - Roll_nt) * 180 / pi)

