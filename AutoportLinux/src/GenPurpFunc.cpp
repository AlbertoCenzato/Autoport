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
		printMatrix(iV, 3, 3);
		printf("\nwP;");
		printMatrix(wP, 3, 3);

		//risoluzione del p3p
		P3p p3p;
		Eigen::Matrix<double, 3, 16> poses = p3p.computePoses(iV, wP);	//set n and m dimension

		printf("poses: \n");
		printMatrix(poses, 3, 16);

		//Vector3d C1 = poses.col(0);
		//Vector3d C2 = poses.col(4);
		//Vector3d C3 = poses.col(8);
		//Vector3d C4 = poses.col(12);
		Matrix3d R1;
		R1.col(0) = poses.col(1);
		R1.col(1) = poses.col(2);
		R1.col(2) = poses.col(3);
		printf("R1: ");
		printMatrix(R1, 3, 3);

		Matrix3d R2;
		R2.col(0) = poses.col(5);
		R2.col(1) = poses.col(6);
		R2.col(2) = poses.col(7);
		printf("R2: ");
		printMatrix(R2, 3, 3);

		Matrix3d R3;
		R3.col(0) = poses.col(9);
		R3.col(1) = poses.col(10);
		R3.col(2) = poses.col(11);
		printf("R3: ");
		printMatrix(R3, 3, 3);
	
		Matrix3d R4;
		R4.col(0) = poses.col(13);
		R4.col(1) = poses.col(14);
		R4.col(2) = poses.col(15);
		printf("R4: ");
		printMatrix(R4, 3, 3);
	
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
		printMatrix(R, 3, 12);
	
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
		printMatrix(F31, 3, 1);
		printf("F32: ");
		printMatrix(F32, 3, 1);
		printf("F33: ");
		printMatrix(F33, 3, 1);
		printf("F34: ");
		printMatrix(F34, 3, 1);
	
		Eigen::Matrix<double, 4, 1> dF;
		dF << abs((F31 - f3).norm()), abs((F32 - f3).norm()), abs((F33 - f3).norm()), abs((F34 - f3).norm());
		printf("dF: ");
		printMatrix(dF, 4, 1);
	
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
	template<typename _Scalar, int NX = Dynamic, int NY = Dynamic> struct Functor {
		typedef _Scalar Scalar;
		enum {
			InputsAtCompileTime = NX,
			ValuesAtCompileTime = NY
		};
		typedef Matrix<Scalar, InputsAtCompileTime, 1> InputType;
		typedef Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
		typedef Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

		int m_inputs, m_values;

		Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
		Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

		int inputs() const { return m_inputs; }
		int values() const { return m_values; }
	};
	
	struct PinHoleEquations : Functor<double> {

		double *v;
		Point2f *leds;
		Point3f *points;
		double focal;
		double d_pxl;

		PinHoleEquations(double* v, Point2f* leds, Point3f* points, double focal, double d_pxl)
			: Functor(6,8), v(v), leds(leds), points(points), focal(focal), d_pxl(d_pxl) {}

		int operator()(VectorXd &variables, VectorXd &fvec) const {

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

			//x positions of leds in the image
			double x_pxl_0 = leds[0].x;
			double x_pxl_1 = leds[1].x;
			double x_pxl_2 = leds[2].x;
			double x_pxl_3 = leds[3].x;
			double x_pxl_4 = leds[4].x;
			double x_pxl_5 = leds[5].x;
			double x_pxl_6 = leds[6].x;
			double x_pxl_7 = leds[7].x;
			//y positions of leds in the image
			double y_pxl_0 = leds[0].y;
			double y_pxl_1 = leds[1].y;
			double y_pxl_2 = leds[2].y;
			double y_pxl_3 = leds[3].y;
			double y_pxl_4 = leds[4].y;
			double y_pxl_5 = leds[5].y;
			double y_pxl_6 = leds[6].y;
			double y_pxl_7 = leds[7].y;

			//x positions of leds in real world
			double Px_0 = points[0].x;
			double Px_1 = points[1].x;
			double Px_2 = points[2].x;
			double Px_3 = points[3].x;
			double Px_4 = points[4].x;
			double Px_5 = points[5].x;
			double Px_6 = points[6].x;
			double Px_7 = points[7].x;
			//y positions of leds in real world
			double Py_0 = points[0].y;
			double Py_1 = points[1].y;
			double Py_2 = points[2].y;
			double Py_3 = points[3].y;
			double Py_4 = points[4].y;
			double Py_5 = points[5].y;
			double Py_6 = points[6].y;
			double Py_7 = points[7].y;
			//z postions of leds in real world
			double Pz_0 = points[0].z;
			double Pz_1 = points[1].z;
			double Pz_2 = points[2].z;
			double Pz_3 = points[3].z;
			double Pz_4 = points[4].z;
			double Pz_5 = points[5].z;
			double Pz_6 = points[6].z;
			double Pz_7 = points[7].z;

			//double vx = v[0];
			//double vy = v[1];
			//double vz = v[2];

			//precomputation of frequently used expressions
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

			//x equations
			fvec(0)  = x_pxl_0 - (focal*(cosPcosY*(Px_0 + x) - sinPitch*(Pz_0 + Z) + cosPsinY*(Py_0 + Y))) / (d_pxl*((Px_0 + x)*(sinRsinY + cosRcosYsinP) - (Py_0 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_0 + Z)));
			fvec(2)  = x_pxl_1 - (focal*(cosPcosY*(Px_1 + x) - sinPitch*(Pz_1 + Z) + cosPsinY*(Py_1 + Y))) / (d_pxl*((Px_1 + x)*(sinRsinY + cosRcosYsinP) - (Py_1 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_1 + Z)));
			fvec(4)  = x_pxl_2 - (focal*(cosPcosY*(Px_2 + x) - sinPitch*(Pz_2 + Z) + cosPsinY*(Py_2 + Y))) / (d_pxl*((Px_2 + x)*(sinRsinY + cosRcosYsinP) - (Py_2 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_2 + Z)));
			fvec(6)  = x_pxl_3 - (focal*(cosPcosY*(Px_3 + x) - sinPitch*(Pz_3 + Z) + cosPsinY*(Py_3 + Y))) / (d_pxl*((Px_3 + x)*(sinRsinY + cosRcosYsinP) - (Py_3 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_3 + Z)));
			fvec(8)  = x_pxl_4 - (focal*(cosPcosY*(Px_4 + x) - sinPitch*(Pz_4 + Z) + cosPsinY*(Py_4 + Y))) / (d_pxl*((Px_4 + x)*(sinRsinY + cosRcosYsinP) - (Py_4 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_4 + Z)));
			fvec(10) = x_pxl_5 - (focal*(cosPcosY*(Px_5 + x) - sinPitch*(Pz_5 + Z) + cosPsinY*(Py_5 + Y))) / (d_pxl*((Px_5 + x)*(sinRsinY + cosRcosYsinP) - (Py_5 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_5 + Z)));
			fvec(12) = x_pxl_6 - (focal*(cosPcosY*(Px_6 + x) - sinPitch*(Pz_6 + Z) + cosPsinY*(Py_6 + Y))) / (d_pxl*((Px_6 + x)*(sinRsinY + cosRcosYsinP) - (Py_6 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_6 + Z)));
			fvec(14) = x_pxl_7 - (focal*(cosPcosY*(Px_7 + x) - sinPitch*(Pz_7 + Z) + cosPsinY*(Py_7 + Y))) / (d_pxl*((Px_7 + x)*(sinRsinY + cosRcosYsinP) - (Py_7 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_7 + Z)));
			//y equations
			fvec(1)  = y_pxl_0 - (focal*((Py_0 + Y)*(cosRcosY + sinPsinRsinY) - (Px_0 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_0 + Z))) / (d_pxl*((Px_0 + x)*(sinRsinY + cosRcosYsinP) - (Py_0 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_0 + Z)));
			fvec(3)  = y_pxl_1 - (focal*((Py_1 + Y)*(cosRcosY + sinPsinRsinY) - (Px_1 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_1 + Z))) / (d_pxl*((Px_1 + x)*(sinRsinY + cosRcosYsinP) - (Py_1 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_1 + Z)));
			fvec(5)  = y_pxl_2 - (focal*((Py_2 + Y)*(cosRcosY + sinPsinRsinY) - (Px_2 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_2 + Z))) / (d_pxl*((Px_2 + x)*(sinRsinY + cosRcosYsinP) - (Py_2 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_2 + Z)));
			fvec(7)  = y_pxl_3 - (focal*((Py_3 + Y)*(cosRcosY + sinPsinRsinY) - (Px_3 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_3 + Z))) / (d_pxl*((Px_3 + x)*(sinRsinY + cosRcosYsinP) - (Py_3 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_3 + Z)));
			fvec(9)  = y_pxl_4 - (focal*((Py_4 + Y)*(cosRcosY + sinPsinRsinY) - (Px_4 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_4 + Z))) / (d_pxl*((Px_4 + x)*(sinRsinY + cosRcosYsinP) - (Py_4 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_4 + Z)));
			fvec(11) = y_pxl_5 - (focal*((Py_5 + Y)*(cosRcosY + sinPsinRsinY) - (Px_5 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_5 + Z))) / (d_pxl*((Px_5 + x)*(sinRsinY + cosRcosYsinP) - (Py_5 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_5 + Z)));
			fvec(13) = y_pxl_6 - (focal*((Py_6 + Y)*(cosRcosY + sinPsinRsinY) - (Px_6 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_6 + Z))) / (d_pxl*((Px_6 + x)*(sinRsinY + cosRcosYsinP) - (Py_6 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_6 + Z)));
			fvec(15) = y_pxl_7 - (focal*((Py_7 + Y)*(cosRcosY + sinPsinRsinY) - (Px_7 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_7 + Z))) / (d_pxl*((Px_7 + x)*(sinRsinY + cosRcosYsinP) - (Py_7 + Y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_7 + Z)));

			return 1;
		}
		
	};

	long pinHoleFSolve(Matrix<double,6,1> &variables, double* v, Point2f* leds, Point3f* points, double focal, double d_pxl) {
		
		auto begin = chrono::high_resolution_clock::now();

		PinHoleEquations pinHoleFunctor(v, leds, points, focal, d_pxl);
		NumericalDiff<PinHoleEquations> numDiff(pinHoleFunctor);
		LevenbergMarquardt<NumericalDiff<PinHoleEquations>, double> levMarq(numDiff);
		levMarq.parameters.maxfev = 2000;
		levMarq.parameters.xtol = 1.0e-10;
		VectorXd dynVar(6);
		dynVar(0) = variables(0);
		dynVar(1) = variables(1);
		dynVar(2) = variables(2);
		dynVar(3) = variables(3);
		dynVar(4) = variables(4);
		dynVar(5) = variables(5);

		//walking inside the world of black magic...
		levMarq.minimize(dynVar);
		variables = dynVar;

		auto end = chrono::high_resolution_clock::now();
		long total = chrono::duration_cast<chrono::nanoseconds>(end - begin).count();
		//double timeInMillis = total / pow(10, 6);
		//printf("\nFsolve: %f millisecondi", timeInMillis);

		return total;
	}

}
