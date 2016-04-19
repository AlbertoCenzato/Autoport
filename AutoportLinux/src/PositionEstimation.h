/*
 * PositionEstimation.h
 *
 *  Created on: Mar 14, 2016
 *      Author: alberto
 */

#ifndef POSITIONESTIMATION_H_
#define POSITIONESTIMATION_H_

#include <list>
#include <chrono>
#include <Eigen/Dense>
#include <unsupported/Eigen/NumericalDiff>
#include <unsupported/Eigen/NonLinearOptimization>

using namespace std;
using namespace cv;
using namespace Eigen;

struct Position_XYZ_YPR {
	float x, y, z, yaw, pitch, roll;

};

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

//specific functor for PinHole
struct PinHoleEquations : Functor<double> {

	vector<Point2f> *cameraSystemPoints;
	vector<Point3f> *realWorldPoints;
	float focal;
	float pixelDimension;

	PinHoleEquations(vector<Point2f> *cameraSystemPoints, vector<Point3f> *realWorldPoints, float focal, float pixelDimention)
	: Functor(6,8), cameraSystemPoints(cameraSystemPoints), realWorldPoints(realWorldPoints), focal(focal), pixelDimension(pixelDimention) {}

	int operator()(VectorXd &position, VectorXd &fvec) const {

		float x 	= (float)position(0);
		float y 	= (float)position(1);
		float z 	= (float)position(2);
		float yaw 	= (float)position(3);
		float pitch = (float)position(4);
		float roll 	= (float)position(5);

		float cosYaw = cos(yaw);
		float sinYaw = sin(yaw);
		float cosPitch = cos(pitch);
		float sinPitch = sin(pitch);
		float cosRoll = cos(roll);
		float sinRoll = sin(roll);

		//x positions of LEDs in the image
		float x_pxl_0 = cameraSystemPoints->at(0).x;
		float x_pxl_1 = cameraSystemPoints->at(1).x;
		float x_pxl_2 = cameraSystemPoints->at(2).x;
		float x_pxl_3 = cameraSystemPoints->at(3).x;
		float x_pxl_4 = cameraSystemPoints->at(4).x;
		float x_pxl_5 = cameraSystemPoints->at(5).x;
		float x_pxl_6 = cameraSystemPoints->at(6).x;
		float x_pxl_7 = cameraSystemPoints->at(7).x;
		//y positions of LEDs in the image
		float y_pxl_0 = cameraSystemPoints->at(0).y;
		float y_pxl_1 = cameraSystemPoints->at(1).y;
		float y_pxl_2 = cameraSystemPoints->at(2).y;
		float y_pxl_3 = cameraSystemPoints->at(3).y;
		float y_pxl_4 = cameraSystemPoints->at(4).y;
		float y_pxl_5 = cameraSystemPoints->at(5).y;
		float y_pxl_6 = cameraSystemPoints->at(6).y;
		float y_pxl_7 = cameraSystemPoints->at(7).y;

		//x positions of LEDs in real world
		float Px_0 = realWorldPoints->at(0).x;
		float Px_1 = realWorldPoints->at(1).x;
		float Px_2 = realWorldPoints->at(2).x;
		float Px_3 = realWorldPoints->at(3).x;
		float Px_4 = realWorldPoints->at(4).x;
		float Px_5 = realWorldPoints->at(5).x;
		float Px_6 = realWorldPoints->at(6).x;
		float Px_7 = realWorldPoints->at(7).x;
		//y positions of LEDs in real world
		float Py_0 = realWorldPoints->at(0).y;
		float Py_1 = realWorldPoints->at(1).y;
		float Py_2 = realWorldPoints->at(2).y;
		float Py_3 = realWorldPoints->at(3).y;
		float Py_4 = realWorldPoints->at(4).y;
		float Py_5 = realWorldPoints->at(5).y;
		float Py_6 = realWorldPoints->at(6).y;
		float Py_7 = realWorldPoints->at(7).y;
		//z postions of LEDs in real world
		float Pz_0 = realWorldPoints->at(0).z;
		float Pz_1 = realWorldPoints->at(1).z;
		float Pz_2 = realWorldPoints->at(2).z;
		float Pz_3 = realWorldPoints->at(3).z;
		float Pz_4 = realWorldPoints->at(4).z;
		float Pz_5 = realWorldPoints->at(5).z;
		float Pz_6 = realWorldPoints->at(6).z;
		float Pz_7 = realWorldPoints->at(7).z;

		//precomputation of frequently used expressions
		float cosPcosY     = cosPitch*cosYaw;
		float cosPsinY     = cosPitch*sinYaw;
		float sinRsinY     = sinRoll*sinYaw;
		float cosRcosYsinP = cosRoll*cosYaw*sinPitch;
		float cosYsinR	   = cosYaw*sinRoll;
		float cosRsinPsinY = cosRoll*sinPitch*sinYaw;
		float cosPcosR     = cosPitch*cosRoll;
		float cosRcosY	   = cosRoll*cosYaw;
		float sinPsinRsinY = sinPitch*sinRoll*sinYaw;
		float cosRsinY	   = cosRoll*sinYaw;
		float cosYsinPsinR = cosYaw*sinPitch*sinRoll;
		float cosPsinR	   = cosPitch*sinRoll;

		//x equations
		fvec(0)  = x_pxl_0 - (focal*(cosPcosY*(Px_0 + x) - sinPitch*(Pz_0 + z) + cosPsinY*(Py_0 + y))) / (pixelDimension*((Px_0 + x)*(sinRsinY + cosRcosYsinP) - (Py_0 + y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_0 + z)));
		fvec(2)  = x_pxl_1 - (focal*(cosPcosY*(Px_1 + x) - sinPitch*(Pz_1 + z) + cosPsinY*(Py_1 + y))) / (pixelDimension*((Px_1 + x)*(sinRsinY + cosRcosYsinP) - (Py_1 + y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_1 + z)));
		fvec(4)  = x_pxl_2 - (focal*(cosPcosY*(Px_2 + x) - sinPitch*(Pz_2 + z) + cosPsinY*(Py_2 + y))) / (pixelDimension*((Px_2 + x)*(sinRsinY + cosRcosYsinP) - (Py_2 + y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_2 + z)));
		fvec(6)  = x_pxl_3 - (focal*(cosPcosY*(Px_3 + x) - sinPitch*(Pz_3 + z) + cosPsinY*(Py_3 + y))) / (pixelDimension*((Px_3 + x)*(sinRsinY + cosRcosYsinP) - (Py_3 + y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_3 + z)));
		fvec(8)  = x_pxl_4 - (focal*(cosPcosY*(Px_4 + x) - sinPitch*(Pz_4 + z) + cosPsinY*(Py_4 + y))) / (pixelDimension*((Px_4 + x)*(sinRsinY + cosRcosYsinP) - (Py_4 + y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_4 + z)));
		fvec(10) = x_pxl_5 - (focal*(cosPcosY*(Px_5 + x) - sinPitch*(Pz_5 + z) + cosPsinY*(Py_5 + y))) / (pixelDimension*((Px_5 + x)*(sinRsinY + cosRcosYsinP) - (Py_5 + y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_5 + z)));
		fvec(12) = x_pxl_6 - (focal*(cosPcosY*(Px_6 + x) - sinPitch*(Pz_6 + z) + cosPsinY*(Py_6 + y))) / (pixelDimension*((Px_6 + x)*(sinRsinY + cosRcosYsinP) - (Py_6 + y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_6 + z)));
		fvec(14) = x_pxl_7 - (focal*(cosPcosY*(Px_7 + x) - sinPitch*(Pz_7 + z) + cosPsinY*(Py_7 + y))) / (pixelDimension*((Px_7 + x)*(sinRsinY + cosRcosYsinP) - (Py_7 + y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_7 + z)));
		//y equations
		fvec(1)  = y_pxl_0 - (focal*((Py_0 + y)*(cosRcosY + sinPsinRsinY) - (Px_0 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_0 + z))) / (pixelDimension*((Px_0 + x)*(sinRsinY + cosRcosYsinP) - (Py_0 + y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_0 + z)));
		fvec(3)  = y_pxl_1 - (focal*((Py_1 + y)*(cosRcosY + sinPsinRsinY) - (Px_1 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_1 + z))) / (pixelDimension*((Px_1 + x)*(sinRsinY + cosRcosYsinP) - (Py_1 + y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_1 + z)));
		fvec(5)  = y_pxl_2 - (focal*((Py_2 + y)*(cosRcosY + sinPsinRsinY) - (Px_2 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_2 + z))) / (pixelDimension*((Px_2 + x)*(sinRsinY + cosRcosYsinP) - (Py_2 + y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_2 + z)));
		fvec(7)  = y_pxl_3 - (focal*((Py_3 + y)*(cosRcosY + sinPsinRsinY) - (Px_3 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_3 + z))) / (pixelDimension*((Px_3 + x)*(sinRsinY + cosRcosYsinP) - (Py_3 + y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_3 + z)));
		fvec(9)  = y_pxl_4 - (focal*((Py_4 + y)*(cosRcosY + sinPsinRsinY) - (Px_4 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_4 + z))) / (pixelDimension*((Px_4 + x)*(sinRsinY + cosRcosYsinP) - (Py_4 + y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_4 + z)));
		fvec(11) = y_pxl_5 - (focal*((Py_5 + y)*(cosRcosY + sinPsinRsinY) - (Px_5 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_5 + z))) / (pixelDimension*((Px_5 + x)*(sinRsinY + cosRcosYsinP) - (Py_5 + y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_5 + z)));
		fvec(13) = y_pxl_6 - (focal*((Py_6 + y)*(cosRcosY + sinPsinRsinY) - (Px_6 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_6 + z))) / (pixelDimension*((Px_6 + x)*(sinRsinY + cosRcosYsinP) - (Py_6 + y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_6 + z)));
		fvec(15) = y_pxl_7 - (focal*((Py_7 + y)*(cosRcosY + sinPsinRsinY) - (Px_7 + x)*(cosRsinY - cosYsinPsinR) + cosPsinR*(Pz_7 + z))) / (pixelDimension*((Px_7 + x)*(sinRsinY + cosRcosYsinP) - (Py_7 + y)*(cosYsinR - cosRsinPsinY) + cosPcosR*(Pz_7 + z)));

		return 1;
	}

};



class PositionEstimation {

	vector<Point3f> *realWorldPoints;
	//-50, -50,  30, -30,  //1, 3, 7, 5
	//-30,  20, -20, -10,
	//  0,   0,  20,   0;

	vector<Point2f> *cameraSystemPoints;
	list<Position_XYZ_YPR*> *lastKnownPositions;
	static const int MAX_LAST_KNOWN_POSITIONS_SIZE = 5;


public:

	PositionEstimation(Position_XYZ_YPR *initialPosition, vector<Point3f> *realWorldPoints) {
		this->realWorldPoints = realWorldPoints;
		lastKnownPositions = new list<Position_XYZ_YPR*>();
		lastKnownPositions->push_front(initialPosition);
	}

	Matrix<float,3,2>* evaluate(vector<Point2f> *);

	inline void clearAll() {

		return;
	}

private:

	const float FOCAL = 3.46031f;
	const float PIXEL_DIMENSION = 1.4e-3f;

	void positionEstimation() {

		VectorXd &dynVar = *(new VectorXd(6));
		Position_XYZ_YPR *lastKnownPos = lastKnownPositions->front();
		dynVar(0) = (double)lastKnownPos->x;
		dynVar(1) = (double)lastKnownPos->y;
		dynVar(2) = (double)lastKnownPos->z;
		dynVar(3) = (double)lastKnownPos->yaw;
		dynVar(4) = (double)lastKnownPos->pitch;
		dynVar(5) = (double)lastKnownPos->roll;

		PinHoleEquations pinHoleFunctor(cameraSystemPoints, realWorldPoints, FOCAL, PIXEL_DIMENSION);
		NumericalDiff<PinHoleEquations> numDiff(pinHoleFunctor);
		LevenbergMarquardt<NumericalDiff<PinHoleEquations>, double> levMarq(numDiff);
		levMarq.parameters.maxfev = 2000;
		levMarq.parameters.xtol = 1.0e-10;

		//walking inside the world of black magic...
		levMarq.minimize(dynVar);

		lastKnownPos = new Position_XYZ_YPR;
		lastKnownPos->x 	= (float)dynVar(0);
		lastKnownPos->y 	= (float)dynVar(1);
		lastKnownPos->z 	= (float)dynVar(2);
		lastKnownPos->yaw 	= (float)dynVar(3);
		lastKnownPos->pitch = (float)dynVar(4);
		lastKnownPos->roll  = (float)dynVar(5);
		lastKnownPositions->push_front(lastKnownPos);

		return;
	}

	void lowPassFilter() {
		return;
	}

	void kalmanFilter() {
		return;
	}

};




#endif /* POSITIONESTIMATION_H_ */
