//Copyright (c) 2016 Alberto Cenzato. All rights reserved.

#include <stdlib.h>
#include <chrono>
#include <Eigen/Dense>
#include <unsupported/Eigen/NumericalDiff>
#include <unsupported/Eigen/NonLinearOptimization>
#include <opencv2/opencv.hpp>
#include "P3p.h"
#include "GenPurpFunc.h"

using namespace std;
using namespace Eigen;
using namespace cv;

namespace GenPurpFunc {



	/* Function che risolve il p3p con il metodo di Laurent Kneip
	@P: Matrice 3x4 dei 4 punti nello spazio assoluto. Le colonne sono i punti
	@f: Matrice 3x4 dei 4 versori nel sistema camera che individuano le
		direzioni dei quattro punti fissi.Le colonne sono i versori
	 Output :
	   C - Centro del sistema camera nel sistema di riferimento assoluto
	   R - Matrice di rotazione dal sistema assoluto a quello camera */

	Matrix<double, 3, 4> p3p_solver(Matrix<double, 3, 4> &P, Matrix<double, 3, 4> &f) {

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

		printf("\niV:");
		printMatrixd(iV, 3, 3);
		printf("\nwP;");
		printMatrixd(wP, 3, 3);

		//risoluzione del p3p
		P3p p3p;
		Eigen::Matrix<double, 3, 16> poses = p3p.computePoses(iV, wP);	//set n and m dimension

		printf("poses: \n");
		printMatrixd(poses, 3, 16);

		//Vector3d C1 = poses.col(0);
		//Vector3d C2 = poses.col(4);
		//Vector3d C3 = poses.col(8);
		//Vector3d C4 = poses.col(12);
		Matrix3d R1;
		R1.col(0) = poses.col(1);
		R1.col(1) = poses.col(2);
		R1.col(2) = poses.col(3);
		printf("R1: ");
		printMatrixd(R1, 3, 3);

		Matrix3d R2;
		R2.col(0) = poses.col(5);
		R2.col(1) = poses.col(6);
		R2.col(2) = poses.col(7);
		printf("R2: ");
		printMatrixd(R2, 3, 3);

		Matrix3d R3;
		R3.col(0) = poses.col(9);
		R3.col(1) = poses.col(10);
		R3.col(2) = poses.col(11);
		printf("R3: ");
		printMatrixd(R3, 3, 3);
	
		Matrix3d R4;
		R4.col(0) = poses.col(13);
		R4.col(1) = poses.col(14);
		R4.col(2) = poses.col(15);
		printf("R4: ");
		printMatrixd(R4, 3, 3);
	
		//Eigen::Matrix<double, 3, 4> C;
		//C.col(0) = C1;
		//C.col(1) = C2;
		//C.col(2) = C3;
		//C.col(3) = C4;
		//printf("C: ");
		//printMatrix(C, 3, 4);
	
		Eigen::Matrix<double, 3, 12> R;
		R << R1, R2, R3, R4;
		printf("R: ");
		printMatrixd(R, 3, 12);
	
		//Discriminazione della soluzione esatta
		Vector3d F31 = (R1*(P3 - poses.col(0)));	//Vector3d F31 = (R1*(P3 - C1));
		Vector3d F32 = (R2*(P3 - poses.col(4)));	//Vector3d F32 = (R2*(P3 - C2));
		Vector3d F33 = (R3*(P3 - poses.col(8)));	//Vector3d F33 = (R3*(P3 - C3));
		Vector3d F34 = (R4*(P3 - poses.col(12)));	//Vector3d F34 = (R4*(P3 - C4));
		F31.normalize();
		F32.normalize();
		F33.normalize();
		F34.normalize();
		printf("F31: ");
		printMatrixd(F31, 3, 1);
		printf("F32: ");
		printMatrixd(F32, 3, 1);
		printf("F33: ");
		printMatrixd(F33, 3, 1);
		printf("F34: ");
		printMatrixd(F34, 3, 1);
	
		Eigen::Matrix<double, 4, 1> dF;
		dF << abs((F31 - f3).norm()), abs((F32 - f3).norm()), abs((F33 - f3).norm()), abs((F34 - f3).norm());
		printf("dF: ");
		printMatrixd(dF, 4, 1);
	
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


	
}
