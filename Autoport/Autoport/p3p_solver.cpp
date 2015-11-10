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

/* Function che risolve il prp con il metodo di Laurent Kneip
% Input:
%   P - Matrice 3x4 dei 4 punti nello spazio assoluto.Le colonne sono i
%       punti
%   f - Matrice 3x4 dei 4 versori nel sistema camera che individuano le
%       direzioni dei quattro punti fissi.Le colonne sono i versori
% Output :
	%   C - Centro del sistema camera nel sistema di riferimento assoluto
	%   R - Matrice di rotazione dal sistema assoluto a quello camera */

Eigen::Matrix<double, dynamic, dynamic>* p3p_solver(Eigen::Matrix<double, 3, 4> &P, Eigen::Matrix<double, 3, 4> &f) {
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
	Eigen::Matrix3d wP;
	wP << P1, P2, P4; //CHECK IF WORKS LIKE THIS
	Eigen::Matrix3d iV;
	iV << f1, f2, f4; //SAME HERE

	//risoluzione del p3p
	P3p p3p;
	Eigen::Matrix<double, dynamic, dynamic> poses;	//set n and m dimension
	p3p.computePoses(iV, wP, poses);
	Vector3d C1 = poses.col(0);
	Vector3d C2 = poses.col(4);
	Vector3d C3 = poses.col(8);
	Vector3d C4 = poses.col(12);
	Eigen::Matrix3d R1 = poses(1:3, 2 : 4);   //CHECK THE SINTAX
	Eigen::Matrix3d R2 = poses(1:3, 6 : 8);	  //CHECK THE SINTAX
	Eigen::Matrix3d R3 = poses(1:3, 10 : 12); //CHECK THE SINTAX
	Eigen::Matrix3d R4 = poses(1:3, 14 : 16); //CHECK THE SINTAX
	Eigen::Matrix<double, dynamic, dynamic> solution;
	solution << C1,C2,C3,C4,R1,R2,R3,R4;
	return solution;
}


function[C R] = p3p_solver_new(P, f)
/*
% Punti fissi nella base[mm]

P1 = P(:, 1);
P2 = P(:, 2);
P3 = P(:, 3);
P4 = P(:, 4);

% Versori normati nel riferimento dell camera

f1 = f(:, 1);
f2 = f(:, 2);
f3 = f(:, 3);
f4 = f(:, 4);
*/
% Primo calcolo
% Input al codice di Kneip :
wP = [P1 P2 P4]; % worldPoints
iV = [f1 f2 f4]; % imageVectors

% Risoluzione del p3p :
poses = p3p(wP, iV);
C1 = poses(1:3, 1);
R1 = poses(1:3, 2 : 4);
C2 = poses(1:3, 5);
R2 = poses(1:3, 6 : 8);
C3 = poses(1:3, 9);
R3 = poses(1:3, 10 : 12);
C4 = poses(1:3, 13);
R4 = poses(1:3, 14 : 16);
C = [C1 C2 C3 C4];
R = [R1 R2 R3 R4];

% Discriminazione della soluzione esatta
F31 = (R1*(P3 - C1)) / norm(R1*(P3 - C1));
F32 = (R2*(P3 - C2)) / norm(R2*(P3 - C2));
F33 = (R3*(P3 - C3)) / norm(R3*(P3 - C3));
F34 = (R4*(P3 - C4)) / norm(R4*(P3 - C4));

dF = abs([norm(F31 - f3) norm(F32 - f3) norm(F33 - f3) norm(F34 - f3)]);
i = find(dF == min(dF));

C = C(:, i);
R = R(:, (3 * (i - 1) + 1) : (3 * (i - 1) + 3));

