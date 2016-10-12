/*
 * PositionEstimation.h
 *
 *  Created on: Mar 14, 2016
 *      Author: alberto
 */

#pragma once

#include <list>
#include <chrono>
#include <Eigen/Dense>
#include <unsupported/Eigen/NumericalDiff>
#include <unsupported/Eigen/NonLinearOptimization>
#include "GenPurpFunc.hpp"
#include "Settings.hpp"

using namespace std;
using namespace cv;
using namespace Eigen;

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


class PositionEstimation {

	//specific functor for PinHole
	struct PinHoleEquations : Functor<double> {

		uint numberOfPoints;

		double focalX;
		double focalY;
		double pixelDimension;

		float  *camSysX;
		float  *camSysY;
		double *realWorldX;
		double *realWorldY;
		double *realWorldZ;

		PinHoleEquations(vector<Point3d> &realWorldPoints, double focalX, double focalY, double pixelDimension)
		: Functor(6,2*realWorldPoints.size()), focalX(focalX), focalY(focalY), pixelDimension(pixelDimension), camSysX(NULL), camSysY(NULL) {

			numberOfPoints = realWorldPoints.size();
			realWorldX = new double[numberOfPoints];
			realWorldY = new double[numberOfPoints];
			realWorldZ = new double[numberOfPoints];
			for(uint i = 0; i < numberOfPoints; i++) {
				realWorldX[i] = realWorldPoints.at(i).x;
				realWorldY[i] = realWorldPoints.at(i).y;
				realWorldZ[i] = realWorldPoints.at(i).z;
			}

		}

		~PinHoleEquations() {
			delete [] camSysX;
			delete [] camSysY;
			delete [] realWorldX;
			delete [] realWorldY;
			delete [] realWorldZ;
		}

		bool addCamSysPoints(vector<Point2f> &cameraSystemPoints) {
			if(cameraSystemPoints.size() != numberOfPoints)
				return false;

			delete [] camSysX;
			delete [] camSysY;
			camSysX = new float[numberOfPoints];
			camSysY = new float[numberOfPoints];
			for(uint i = 0; i < numberOfPoints; i++) {
				camSysX[i] = cameraSystemPoints.at(i).x;
				camSysY[i] = cameraSystemPoints.at(i).y;
			}
			return true;
		}


		int operator()(VectorXd &position, VectorXd &fvec) const {

			double x 	 = position(0);
			double y 	 = position(1);
			double z 	 = position(2);
			double yaw 	 = position(3);
			double pitch = position(4);
			double roll  = position(5);

			double cosYaw   = cos(yaw);
			double sinYaw   = sin(yaw);
			double cosPitch = cos(pitch);
			double sinPitch = sin(pitch);
			double cosRoll  = cos(roll);
			double sinRoll  = sin(roll);

			//precomputation of frequently used expressions
			double cosPcosY     = cosPitch * cosYaw;
			double cosPsinY     = cosPitch * sinYaw;
			double sinRsinY     = sinRoll  * sinYaw;
			double cosRcosYsinP = cosRoll  * cosYaw   * sinPitch;
			double cosYsinR	    = cosYaw   * sinRoll;
			double cosRsinPsinY = cosRoll  * sinPitch * sinYaw;
			double cosPcosR     = cosPitch * cosRoll;
			double cosRcosY	    = cosRoll  * cosYaw;
			double cosRsinY	    = cosRoll  * sinYaw;
			double cosYsinPsinR = cosYaw   * sinPitch * sinRoll;
			double cosPsinR	    = cosPitch * sinRoll;

			for(uint i = 0; i < numberOfPoints; i++) {
				double Px_0 = realWorldX[i];
				double Py_0 = realWorldY[i];
				double Pz_0 = realWorldZ[i];
				fvec(i*2)     = camSysX[i] - (focalX*(x - Pz_0*sinPitch + Px_0*cosPcosY + Py_0*cosPsinY))/(pixelDimension*(z + Px_0*(sinRsinY + cosRcosYsinP) - Py_0*(cosYsinR - cosRsinPsinY) + Pz_0*cosPcosR));
				fvec((i*2)+1) = camSysY[i] - (focalY*(y - Px_0*(cosRsinY - cosYsinPsinR) + Py_0*(cosRcosY + sinPitch*sinRsinY) + Pz_0*cosPsinR))/(pixelDimension*(z + Px_0*(sinRsinY + cosRcosYsinP) - Py_0*(cosYsinR - cosRsinPsinY) + Pz_0*cosPcosR));
			}

			return 0;
		}

	};

	PinHoleEquations *pinHoleFunctor;

	vector<Point3d> *realWorldPoints;
	vector<Point2f> *cameraSystemPoints;

	vector<Point3d> *currRealWorldSet;

	list<Position_XYZ_YPR*> lastKnownPositions;
	static const int MAX_LAST_KNOWN_POSITIONS_SIZE = 5;
	uchar pointsToEvaluate;
	uint numberOfUsedPoints;


public:


	PositionEstimation(vector<Point3d> &realWorldPoints) {
		Settings &settings = Settings::getInstance();
		focalX = settings.focalX;
		focalY = settings.focalY;
		pixelDimension = settings.pixelDimension;
		this->realWorldPoints  = &realWorldPoints;
		this->currRealWorldSet = new vector<Point3d>(realWorldPoints);
		lastKnownPositions = list<Position_XYZ_YPR*>();
		lastKnownPositions.push_front(&settings.initialPosition);
		cameraSystemPoints = NULL;
		pointsToEvaluate = 0xFF;	// bit array set to all-ones: 11111111; uses all points
		numberOfUsedPoints = 8;
		pinHoleFunctor = new PinHoleEquations(*currRealWorldSet, focalX, focalY, pixelDimension);
	}

	PositionEstimation() : PositionEstimation(Settings::getInstance().realWorldPoints) {}

	~PositionEstimation() {
		delete cameraSystemPoints;
		delete currRealWorldSet;
		delete pinHoleFunctor;
	}

	void evaluate(vector<Point2f> &, Matrix<double,3,2> &evaluatedPoints);

	PositionEstimation* setPointsToEvaluate(uchar pointsToEvaluate);

private:

	double focalX;
	double focalY;
	double pixelDimension;

	void levenbergMarquardt() {

		VectorXd &dynVar = *(new VectorXd(6));
		Position_XYZ_YPR *lastKnownPos = lastKnownPositions.front();
		dynVar(0) = lastKnownPos->x;
		dynVar(1) = lastKnownPos->y;
		dynVar(2) = lastKnownPos->z;
		dynVar(3) = lastKnownPos->yaw;
		dynVar(4) = lastKnownPos->pitch;
		dynVar(5) = lastKnownPos->roll;

		//char usedLeds = 0xAB; //1010 1011
		//int numberOfUsedLeds = 5;

		if(!pinHoleFunctor->addCamSysPoints(*cameraSystemPoints)) {
			cout << "ERROR!!!!!!" << endl;
			throw Exception();
		}
		NumericalDiff<PinHoleEquations> numDiff(*pinHoleFunctor);
		LevenbergMarquardt<NumericalDiff<PinHoleEquations>, double> levMarq(numDiff);
		levMarq.parameters.maxfev = 1000;
		levMarq.parameters.xtol = 1.0e-6;

		//walking inside the world of black magic...
		levMarq.minimize(dynVar); //levMarq.minimize(dynVar);

		lastKnownPos = new Position_XYZ_YPR;
		lastKnownPos->x 	= dynVar(0);
		lastKnownPos->y 	= dynVar(1);
		lastKnownPos->z 	= dynVar(2);
		lastKnownPos->yaw 	= dynVar(3);
		lastKnownPos->pitch = dynVar(4);
		lastKnownPos->roll  = dynVar(5);
		lastKnownPositions.push_front(lastKnownPos);

		return;
	}

	void lowPassFilter() {
		return;
	}

	void kalmanFilter() {
		return;
	}

};
