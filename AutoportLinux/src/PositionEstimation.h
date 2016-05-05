/*
 * PositionEstimation.h
 *
 *  Created on: Mar 14, 2016
 *      Author: alberto
 */

#ifndef POSITIONESTIMATION_H_
#define POSITIONESTIMATION_H_

#include <memory>
#include <list>
#include <chrono>
#include <Eigen/Dense>
#include <unsupported/Eigen/NumericalDiff>
#include <unsupported/Eigen/NonLinearOptimization>

using namespace std;
using namespace cv;
using namespace Eigen;

struct Position_XYZ_YPR {
	double x, y, z, yaw, pitch, roll;

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
	vector<Point3d> *realWorldPoints;
	char usedLeds;
	int numberOfUsedLeds;
	double focalX;
	double focalY;
	double pixelDimension;

	float *cameraSysX;
	float *cameraSysY;
	double *realSysX;
	double *realSysY;
	double *realSysZ;

	PinHoleEquations(vector<Point2f> *cameraSystemPoints, vector<Point3d> *realWorldPoints, uchar usedLeds, int numberOfUsedLeds, double focalX, double focalY, double pixelDimension)
	: Functor(6,2*numberOfUsedLeds), cameraSystemPoints(cameraSystemPoints), realWorldPoints(realWorldPoints), usedLeds(usedLeds), numberOfUsedLeds(numberOfUsedLeds), focalX(focalX), focalY(focalY), pixelDimension(pixelDimension) {


		cameraSysX = new float[numberOfUsedLeds];
		cameraSysY = new float[numberOfUsedLeds];
		realSysX = new double[numberOfUsedLeds];
		realSysY = new double[numberOfUsedLeds];
		realSysZ = new double[numberOfUsedLeds];
		uchar tmp = usedLeds;
		for(int i = 0, count = 0; i < 8; i++) {
			if((tmp & 0x80) != 0) {
				cameraSysX[count] = cameraSystemPoints->at(i).x;
				cameraSysY[count] = cameraSystemPoints->at(i).y;
				realSysX[count] = realWorldPoints->at(i).x;
				realSysY[count] = realWorldPoints->at(i).y;
				realSysZ[count] = realWorldPoints->at(i).z;
				cout << i << endl;
				count++;
			}
			tmp = tmp << 1;
		}
	}

	~PinHoleEquations() {
		delete [] cameraSysX;
		delete [] cameraSysY;
		delete [] realSysX;
		delete [] realSysY;
		delete [] realSysZ;
	}

	int operator()(VectorXd &position, VectorXd &fvec) const {

		double x 	 = position(0);
		double y 	 = position(1);
		double z 	 = position(2);
		double yaw 	 = position(3);
		double pitch = position(4);
		double roll  = position(5);

		double cosYaw = cos(yaw);
		double sinYaw = sin(yaw);
		double cosPitch = cos(pitch);
		double sinPitch = sin(pitch);
		double cosRoll = cos(roll);
		double sinRoll = sin(roll);

		//precomputation of frequently used expressions
		double cosPcosY     = cosPitch*cosYaw;
		double cosPsinY     = cosPitch*sinYaw;
		double sinRsinY     = sinRoll*sinYaw;
		double cosRcosYsinP = cosRoll*cosYaw*sinPitch;
		double cosYsinR	    = cosYaw*sinRoll;
		double cosRsinPsinY = cosRoll*sinPitch*sinYaw;
		double cosPcosR     = cosPitch*cosRoll;
		double cosRcosY	    = cosRoll*cosYaw;
		double cosRsinY	    = cosRoll*sinYaw;
		double cosYsinPsinR = cosYaw*sinPitch*sinRoll;
		double cosPsinR	    = cosPitch*sinRoll;

		for(int i = 0; i < numberOfUsedLeds; i++) {
			double Px_0 = realSysX[i];
			double Py_0 = realSysY[i];
			double Pz_0 = realSysZ[i];
			fvec(i*2)     = cameraSysX[i] - (focalX*(x - Pz_0*sinPitch + Px_0*cosPcosY + Py_0*cosPsinY))/(pixelDimension*(z + Px_0*(sinRsinY + cosRcosYsinP) - Py_0*(cosYsinR - cosRsinPsinY) + Pz_0*cosPcosR));
			fvec((i*2)+1) = cameraSysY[i] - (focalY*(y - Px_0*(cosRsinY - cosYsinPsinR) + Py_0*(cosRcosY + sinPitch*sinRsinY) + Pz_0*cosPsinR))/(pixelDimension*(z + Px_0*(sinRsinY + cosRcosYsinP) - Py_0*(cosYsinR - cosRsinPsinY) + Pz_0*cosPcosR));
		}

		/*
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
		double Px_0 = realWorldPoints->at(0).x;
		double Px_1 = realWorldPoints->at(1).x;
		double Px_2 = realWorldPoints->at(2).x;
		double Px_3 = realWorldPoints->at(3).x;
		double Px_4 = realWorldPoints->at(4).x;
		double Px_5 = realWorldPoints->at(5).x;
		double Px_6 = realWorldPoints->at(6).x;
		double Px_7 = realWorldPoints->at(7).x;
		//y positions of LEDs in real world
		double Py_0 = realWorldPoints->at(0).y;
		double Py_1 = realWorldPoints->at(1).y;
		double Py_2 = realWorldPoints->at(2).y;
		double Py_3 = realWorldPoints->at(3).y;
		double Py_4 = realWorldPoints->at(4).y;
		double Py_5 = realWorldPoints->at(5).y;
		double Py_6 = realWorldPoints->at(6).y;
		double Py_7 = realWorldPoints->at(7).y;
		//z postions of LEDs in real world
		double Pz_0 = realWorldPoints->at(0).z;
		double Pz_1 = realWorldPoints->at(1).z;
		double Pz_2 = realWorldPoints->at(2).z;
		double Pz_3 = realWorldPoints->at(3).z;
		double Pz_4 = realWorldPoints->at(4).z;
		double Pz_5 = realWorldPoints->at(5).z;
		double Pz_6 = realWorldPoints->at(6).z;
		double Pz_7 = realWorldPoints->at(7).z;

		//precomputation of frequently used expressions
		double cosPcosY     = cosPitch*cosYaw;
		double cosPsinY     = cosPitch*sinYaw;
		double sinRsinY     = sinRoll*sinYaw;
		double cosRcosYsinP = cosRoll*cosYaw*sinPitch;
		double cosYsinR	   = cosYaw*sinRoll;
		double cosRsinPsinY = cosRoll*sinPitch*sinYaw;
		double cosPcosR     = cosPitch*cosRoll;
		double cosRcosY	   = cosRoll*cosYaw;
		double cosRsinY	   = cosRoll*sinYaw;
		double cosYsinPsinR = cosYaw*sinPitch*sinRoll;
		double cosPsinR	   = cosPitch*sinRoll;

		fvec(0)  = x_pxl_0 - (focalX*(x - Pz_0*sinPitch + Px_0*cosPcosY + Py_0*cosPsinY))/(pixelDimension*(z + Px_0*(sinRsinY + cosRcosYsinP) - Py_0*(cosYsinR - cosRsinPsinY) + Pz_0*cosPcosR));
		fvec(2)  = x_pxl_1 - (focalX*(x - Pz_1*sinPitch + Px_1*cosPcosY + Py_1*cosPsinY))/(pixelDimension*(z + Px_1*(sinRsinY + cosRcosYsinP) - Py_1*(cosYsinR - cosRsinPsinY) + Pz_1*cosPcosR));
		fvec(4)  = x_pxl_2 - (focalX*(x - Pz_2*sinPitch + Px_2*cosPcosY + Py_2*cosPsinY))/(pixelDimension*(z + Px_2*(sinRsinY + cosRcosYsinP) - Py_2*(cosYsinR - cosRsinPsinY) + Pz_2*cosPcosR));
		fvec(6)  = x_pxl_3 - (focalX*(x - Pz_3*sinPitch + Px_3*cosPcosY + Py_3*cosPsinY))/(pixelDimension*(z + Px_3*(sinRsinY + cosRcosYsinP) - Py_3*(cosYsinR - cosRsinPsinY) + Pz_3*cosPcosR));
		fvec(8)  = x_pxl_4 - (focalX*(x - Pz_4*sinPitch + Px_4*cosPcosY + Py_4*cosPsinY))/(pixelDimension*(z + Px_4*(sinRsinY + cosRcosYsinP) - Py_4*(cosYsinR - cosRsinPsinY) + Pz_4*cosPcosR));
		fvec(10) = x_pxl_5 - (focalX*(x - Pz_5*sinPitch + Px_5*cosPcosY + Py_5*cosPsinY))/(pixelDimension*(z + Px_5*(sinRsinY + cosRcosYsinP) - Py_5*(cosYsinR - cosRsinPsinY) + Pz_5*cosPcosR));
		fvec(12) = x_pxl_6 - (focalX*(x - Pz_6*sinPitch + Px_6*cosPcosY + Py_6*cosPsinY))/(pixelDimension*(z + Px_6*(sinRsinY + cosRcosYsinP) - Py_6*(cosYsinR - cosRsinPsinY) + Pz_6*cosPcosR));
		fvec(14) = x_pxl_7 - (focalX*(x - Pz_7*sinPitch + Px_7*cosPcosY + Py_7*cosPsinY))/(pixelDimension*(z + Px_7*(sinRsinY + cosRcosYsinP) - Py_7*(cosYsinR - cosRsinPsinY) + Pz_7*cosPcosR));

		fvec(1)  = y_pxl_0 - (focalY*(y - Px_0*(cosRsinY - cosYsinPsinR) + Py_0*(cosRcosY + sinPitch*sinRsinY) + Pz_0*cosPsinR))/(pixelDimension*(z + Px_0*(sinRsinY + cosRcosYsinP) - Py_0*(cosYsinR - cosRsinPsinY) + Pz_0*cosPcosR));
		fvec(3)  = y_pxl_1 - (focalY*(y - Px_1*(cosRsinY - cosYsinPsinR) + Py_1*(cosRcosY + sinPitch*sinRsinY) + Pz_1*cosPsinR))/(pixelDimension*(z + Px_1*(sinRsinY + cosRcosYsinP) - Py_1*(cosYsinR - cosRsinPsinY) + Pz_1*cosPcosR));
		fvec(5)  = y_pxl_2 - (focalY*(y - Px_2*(cosRsinY - cosYsinPsinR) + Py_2*(cosRcosY + sinPitch*sinRsinY) + Pz_2*cosPsinR))/(pixelDimension*(z + Px_2*(sinRsinY + cosRcosYsinP) - Py_2*(cosYsinR - cosRsinPsinY) + Pz_2*cosPcosR));
		fvec(7)  = y_pxl_3 - (focalY*(y - Px_3*(cosRsinY - cosYsinPsinR) + Py_3*(cosRcosY + sinPitch*sinRsinY) + Pz_3*cosPsinR))/(pixelDimension*(z + Px_3*(sinRsinY + cosRcosYsinP) - Py_3*(cosYsinR - cosRsinPsinY) + Pz_3*cosPcosR));
		fvec(9)  = y_pxl_4 - (focalY*(y - Px_4*(cosRsinY - cosYsinPsinR) + Py_4*(cosRcosY + sinPitch*sinRsinY) + Pz_4*cosPsinR))/(pixelDimension*(z + Px_4*(sinRsinY + cosRcosYsinP) - Py_4*(cosYsinR - cosRsinPsinY) + Pz_4*cosPcosR));
		fvec(11) = y_pxl_5 - (focalY*(y - Px_5*(cosRsinY - cosYsinPsinR) + Py_5*(cosRcosY + sinPitch*sinRsinY) + Pz_5*cosPsinR))/(pixelDimension*(z + Px_5*(sinRsinY + cosRcosYsinP) - Py_5*(cosYsinR - cosRsinPsinY) + Pz_5*cosPcosR));
		fvec(13) = y_pxl_6 - (focalY*(y - Px_6*(cosRsinY - cosYsinPsinR) + Py_6*(cosRcosY + sinPitch*sinRsinY) + Pz_6*cosPsinR))/(pixelDimension*(z + Px_6*(sinRsinY + cosRcosYsinP) - Py_6*(cosYsinR - cosRsinPsinY) + Pz_6*cosPcosR));
		fvec(15) = y_pxl_7 - (focalY*(y - Px_7*(cosRsinY - cosYsinPsinR) + Py_7*(cosRcosY + sinPitch*sinRsinY) + Pz_7*cosPsinR))/(pixelDimension*(z + Px_7*(sinRsinY + cosRcosYsinP) - Py_7*(cosYsinR - cosRsinPsinY) + Pz_7*cosPcosR));
		*/


		/*
		//x equations
		fvec(0)   = x_pxl_0 - (focal*(x - Pz_0*sinPitch + Px_0*cosPcosY + Py_0*cosPsinY))/(pixelDimension*(z + Px_0*(sinRsinY + cosRcosYsinP) - Py_0*(cosYsinR - cosRsinPsinY) + Pz_0*cosPcosR));
		fvec(2)   = x_pxl_1 - (focal*(x - Pz_1*sinPitch + Px_1*cosPcosY + Py_1*cosPsinY))/(pixelDimension*(z + Px_1*(sinRsinY + cosRcosYsinP) - Py_1*(cosYsinR - cosRsinPsinY) + Pz_1*cosPcosR));
		fvec(4)   = x_pxl_2 - (focal*(x - Pz_2*sinPitch + Px_2*cosPcosY + Py_2*cosPsinY))/(pixelDimension*(z + Px_2*(sinRsinY + cosRcosYsinP) - Py_2*(cosYsinR - cosRsinPsinY) + Pz_2*cosPcosR));
		fvec(6)   = x_pxl_3 - (focal*(x - Pz_3*sinPitch + Px_3*cosPcosY + Py_3*cosPsinY))/(pixelDimension*(z + Px_3*(sinRsinY + cosRcosYsinP) - Py_3*(cosYsinR - cosRsinPsinY) + Pz_3*cosPcosR));
		fvec(8)   = x_pxl_4 - (focal*(x - Pz_4*sinPitch + Px_4*cosPcosY + Py_4*cosPsinY))/(pixelDimension*(z + Px_4*(sinRsinY + cosRcosYsinP) - Py_4*(cosYsinR - cosRsinPsinY) + Pz_4*cosPcosR));
		fvec(10)  = x_pxl_5 - (focal*(x - Pz_5*sinPitch + Px_5*cosPcosY + Py_5*cosPsinY))/(pixelDimension*(z + Px_5*(sinRsinY + cosRcosYsinP) - Py_5*(cosYsinR - cosRsinPsinY) + Pz_5*cosPcosR));
		fvec(12)  = x_pxl_6 - (focal*(x - Pz_6*sinPitch + Px_6*cosPcosY + Py_6*cosPsinY))/(pixelDimension*(z + Px_6*(sinRsinY + cosRcosYsinP) - Py_6*(cosYsinR - cosRsinPsinY) + Pz_6*cosPcosR));
		fvec(14)  = x_pxl_7 - (focal*(x - Pz_7*sinPitch + Px_7*cosPcosY + Py_7*cosPsinY))/(pixelDimension*(z + Px_7*(sinRsinY + cosRcosYsinP) - Py_7*(cosYsinR - cosRsinPsinY) + Pz_7*cosPcosR));

		//y equations
		fvec(1)   = y_pxl_0 - (focal*(y - Px_0*(cosRsinY - cosYsinPsinR) + Py_0*(cosRcosY + sinPitch*sinRsinY) + Pz_0*cosPsinR))/(pixelDimension*(z + Px_0*(sinRsinY + cosRcosYsinP) - Py_0*(cosYsinR - cosRsinPsinY) + Pz_0*cosPcosR));
		fvec(3)   = y_pxl_1 - (focal*(y - Px_1*(cosRsinY - cosYsinPsinR) + Py_1*(cosRcosY + sinPitch*sinRsinY) + Pz_1*cosPsinR))/(pixelDimension*(z + Px_1*(sinRsinY + cosRcosYsinP) - Py_1*(cosYsinR - cosRsinPsinY) + Pz_1*cosPcosR));
		fvec(5)   = y_pxl_2 - (focal*(y - Px_2*(cosRsinY - cosYsinPsinR) + Py_2*(cosRcosY + sinPitch*sinRsinY) + Pz_2*cosPsinR))/(pixelDimension*(z + Px_2*(sinRsinY + cosRcosYsinP) - Py_2*(cosYsinR - cosRsinPsinY) + Pz_2*cosPcosR));
		fvec(7)   = y_pxl_3 - (focal*(y - Px_3*(cosRsinY - cosYsinPsinR) + Py_3*(cosRcosY + sinPitch*sinRsinY) + Pz_3*cosPsinR))/(pixelDimension*(z + Px_3*(sinRsinY + cosRcosYsinP) - Py_3*(cosYsinR - cosRsinPsinY) + Pz_3*cosPcosR));
		fvec(9)   = y_pxl_4 - (focal*(y - Px_4*(cosRsinY - cosYsinPsinR) + Py_4*(cosRcosY + sinPitch*sinRsinY) + Pz_4*cosPsinR))/(pixelDimension*(z + Px_4*(sinRsinY + cosRcosYsinP) - Py_4*(cosYsinR - cosRsinPsinY) + Pz_4*cosPcosR));
		fvec(11)  = y_pxl_5 - (focal*(y - Px_5*(cosRsinY - cosYsinPsinR) + Py_5*(cosRcosY + sinPitch*sinRsinY) + Pz_5*cosPsinR))/(pixelDimension*(z + Px_5*(sinRsinY + cosRcosYsinP) - Py_5*(cosYsinR - cosRsinPsinY) + Pz_5*cosPcosR));
		fvec(13)  = y_pxl_6 - (focal*(y - Px_6*(cosRsinY - cosYsinPsinR) + Py_6*(cosRcosY + sinPitch*sinRsinY) + Pz_6*cosPsinR))/(pixelDimension*(z + Px_6*(sinRsinY + cosRcosYsinP) - Py_6*(cosYsinR - cosRsinPsinY) + Pz_6*cosPcosR));
		fvec(15)  = y_pxl_7 - (focal*(y - Px_7*(cosRsinY - cosYsinPsinR) + Py_7*(cosRcosY + sinPitch*sinRsinY) + Pz_7*cosPsinR))/(pixelDimension*(z + Px_7*(sinRsinY + cosRcosYsinP) - Py_7*(cosYsinR - cosRsinPsinY) + Pz_7*cosPcosR));
		*/

		return 1;
	}

};



class PositionEstimation {

	vector<Point3d> *realWorldPoints;
	vector<Point2f> *cameraSystemPoints;
	list<Position_XYZ_YPR*> *lastKnownPositions;
	static const int MAX_LAST_KNOWN_POSITIONS_SIZE = 5;


public:

	PositionEstimation(Position_XYZ_YPR *initialPosition, vector<Point3d> *realWorldPoints) {
		this->realWorldPoints = realWorldPoints;
		lastKnownPositions = new list<Position_XYZ_YPR*>();
		lastKnownPositions->push_front(initialPosition);
		cameraSystemPoints = NULL;
	}

	Matrix<double,3,2>* evaluate(vector<Point2f> *);

	inline void clearAll() {

		return;
	}

private:

	const double FOCAL_X = 3.46031d;
	const double FOCAL_Y = 3.46031d;
	const double PIXEL_DIMENSION = 1.4e-3d;

	void positionEstimation() {

		VectorXd &dynVar = *(new VectorXd(6));
		Position_XYZ_YPR *lastKnownPos = lastKnownPositions->front();
		dynVar(0) = lastKnownPos->x;
		dynVar(1) = lastKnownPos->y;
		dynVar(2) = lastKnownPos->z;
		dynVar(3) = lastKnownPos->yaw;
		dynVar(4) = lastKnownPos->pitch;
		dynVar(5) = lastKnownPos->roll;

		char usedLeds = 0xAB; //1010 1011
		int numberOfUsedLeds = 5;

		PinHoleEquations *pinHoleFunctor = new PinHoleEquations(cameraSystemPoints, realWorldPoints, usedLeds, numberOfUsedLeds, FOCAL_X, FOCAL_Y, PIXEL_DIMENSION);
		NumericalDiff<PinHoleEquations> numDiff(*pinHoleFunctor);
		LevenbergMarquardt<NumericalDiff<PinHoleEquations>, double> levMarq(numDiff);
		levMarq.parameters.maxfev = 1000;
		levMarq.parameters.xtol = 1.0e-6;

		//walking inside the world of black magic...
		levMarq.minimize(dynVar);

		lastKnownPos = new Position_XYZ_YPR;
		lastKnownPos->x 	= dynVar(0);
		lastKnownPos->y 	= dynVar(1);
		lastKnownPos->z 	= dynVar(2);
		lastKnownPos->yaw 	= dynVar(3);
		lastKnownPos->pitch = dynVar(4);
		lastKnownPos->roll  = dynVar(5);
		lastKnownPositions->push_front(lastKnownPos);

		delete pinHoleFunctor;

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
