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

/*

function[C R] = p3p_solver_new(P, f)


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

*/