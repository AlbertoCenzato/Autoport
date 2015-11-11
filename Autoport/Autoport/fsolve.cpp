struct PinHoleEquations {
	PinHoleEquations(int x) : x(x) {}
	Eigen::Matrix<double,11,1>* operator()(Eigen::Matrix<double, dynamic, 1> &X) const {
		
		double X = X(1);
		double Y = X(2);
		double Z = X(3);

		double Yaw = X(4);
		double Pitch = X(5);
		double Roll = X(6);
		
		Eigen::Matrix<double, 11, 1> Y;
		Y(0)  = x_pxl_1 - (focal*(cos(Pitch)*cos(Yaw)*(Px_1 - X) - sin(Pitch)*(Pz_1 - Z) + cos(Pitch)*sin(Yaw)*(Py_1 - Y))) / (d_pxl*((Px_1 - X)*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - (Py_1 - Y)*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + cos(Pitch)*cos(Roll)*(Pz_1 - Z)));
		Y(1) = y_pxl_1 - (focal*((Py_1 - Y)*(cos(Roll)*cos(Yaw) + sin(Pitch)*sin(Roll)*sin(Yaw)) - (Px_1 - X)*(cos(Roll)*sin(Yaw) - cos(Yaw)*sin(Pitch)*sin(Roll)) + cos(Pitch)*sin(Roll)*(Pz_1 - Z))) / (d_pxl*((Px_1 - X)*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - (Py_1 - Y)*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + cos(Pitch)*cos(Roll)*(Pz_1 - Z)));
		Y(2) = x_pxl_2 - (focal*(cos(Pitch)*cos(Yaw)*(Px_2 - X) - sin(Pitch)*(Pz_2 - Z) + cos(Pitch)*sin(Yaw)*(Py_2 - Y))) / (d_pxl*((Px_2 - X)*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - (Py_2 - Y)*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + cos(Pitch)*cos(Roll)*(Pz_2 - Z)));

		Y(3) = y_pxl_2 - (focal*((Py_2 - Y)*(cos(Roll)*cos(Yaw) + sin(Pitch)*sin(Roll)*sin(Yaw)) - (Px_2 - X)*(cos(Roll)*sin(Yaw) - cos(Yaw)*sin(Pitch)*sin(Roll)) + cos(Pitch)*sin(Roll)*(Pz_2 - Z))) / (d_pxl*((Px_2 - X)*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - (Py_2 - Y)*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + cos(Pitch)*cos(Roll)*(Pz_2 - Z)));
		Y(4) = x_pxl_3 - (focal*(cos(Pitch)*cos(Yaw)*(Px_3 - X) - sin(Pitch)*(Pz_3 - Z) + cos(Pitch)*sin(Yaw)*(Py_3 - Y))) / (d_pxl*((Px_3 - X)*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - (Py_3 - Y)*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + cos(Pitch)*cos(Roll)*(Pz_3 - Z)));
		Y(5) = y_pxl_3 - (focal*((Py_3 - Y)*(cos(Roll)*cos(Yaw) + sin(Pitch)*sin(Roll)*sin(Yaw)) - (Px_3 - X)*(cos(Roll)*sin(Yaw) - cos(Yaw)*sin(Pitch)*sin(Roll)) + cos(Pitch)*sin(Roll)*(Pz_3 - Z))) / (d_pxl*((Px_3 - X)*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - (Py_3 - Y)*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + cos(Pitch)*cos(Roll)*(Pz_3 - Z)));
		Y(6) = x_pxl_4 - (focal*(cos(Pitch)*cos(Yaw)*(Px_4 - X) - sin(Pitch)*(Pz_4 - Z) + cos(Pitch)*sin(Yaw)*(Py_4 - Y))) / (d_pxl*((Px_4 - X)*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - (Py_4 - Y)*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + cos(Pitch)*cos(Roll)*(Pz_4 - Z)));
		Y(7) = y_pxl_4 - (focal*((Py_4 - Y)*(cos(Roll)*cos(Yaw) + sin(Pitch)*sin(Roll)*sin(Yaw)) - (Px_4 - X)*(cos(Roll)*sin(Yaw) - cos(Yaw)*sin(Pitch)*sin(Roll)) + cos(Pitch)*sin(Roll)*(Pz_4 - Z))) / (d_pxl*((Px_4 - X)*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - (Py_4 - Y)*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + cos(Pitch)*cos(Roll)*(Pz_4 - Z)));
		Y(8) = vx + (qx - Z*sin(Pitch) + X*cos(Pitch)*cos(Yaw) + Y*cos(Pitch)*sin(Yaw)) / (abs(qx - Z*sin(Pitch) + X*cos(Pitch)*cos(Yaw) + Y*cos(Pitch)*sin(Yaw)) ^ 2 + abs(qz + X*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - Y*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + Z*cos(Pitch)*cos(Roll)) ^ 2 + abs(qy - X*(cos(Roll)*sin(Yaw) - cos(Yaw)*sin(Pitch)*sin(Roll)) + Y*(cos(Roll)*cos(Yaw) + sin(Pitch)*sin(Roll)*sin(Yaw)) + Z*cos(Pitch)*sin(Roll)) ^ 2) ^ (1 / 2);
		Y(9) = vy + (qy - X*(cos(Roll)*sin(Yaw) - cos(Yaw)*sin(Pitch)*sin(Roll)) + Y*(cos(Roll)*cos(Yaw) + sin(Pitch)*sin(Roll)*sin(Yaw)) + Z*cos(Pitch)*sin(Roll)) / (abs(qx - Z*sin(Pitch) + X*cos(Pitch)*cos(Yaw) + Y*cos(Pitch)*sin(Yaw)) ^ 2 + abs(qz + X*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - Y*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + Z*cos(Pitch)*cos(Roll)) ^ 2 + abs(qy - X*(cos(Roll)*sin(Yaw) - cos(Yaw)*sin(Pitch)*sin(Roll)) + Y*(cos(Roll)*cos(Yaw) + sin(Pitch)*sin(Roll)*sin(Yaw)) + Z*cos(Pitch)*sin(Roll)) ^ 2) ^ (1 / 2);
		Y(10) = vz + (qz + X*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - Y*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + Z*cos(Pitch)*cos(Roll)) / (abs(qx - Z*sin(Pitch) + X*cos(Pitch)*cos(Yaw) + Y*cos(Pitch)*sin(Yaw)) ^ 2 + abs(qz + X*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - Y*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + Z*cos(Pitch)*cos(Roll)) ^ 2 + abs(qy - X*(cos(Roll)*sin(Yaw) - cos(Yaw)*sin(Pitch)*sin(Roll)) + Y*(cos(Roll)*cos(Yaw) + sin(Pitch)*sin(Roll)*sin(Yaw)) + Z*cos(Pitch)*sin(Roll)) ^ 2) ^ (1 / 2) ];

		return &Y; 
	}
	/*
private:
	int x;
	*/
};


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
X = X0(1);
Y = X0(2);
Z = X0(3);

Yaw = X0(4);
Pitch = X0(5);
Roll = X0(6);

//TR = [x_pxl_1 - (focal*(cos(Pitch)*cos(Yaw)*(Px_1 - X) - sin(Pitch)*(Pz_1 - Z) + cos(Pitch)*sin(Yaw)*(Py_1 - Y))) / (d_pxl*((Px_1 - X)*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - (Py_1 - Y)*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + cos(Pitch)*cos(Roll)*(Pz_1 - Z)));
//y_pxl_1 - (focal*((Py_1 - Y)*(cos(Roll)*cos(Yaw) + sin(Pitch)*sin(Roll)*sin(Yaw)) - (Px_1 - X)*(cos(Roll)*sin(Yaw) - cos(Yaw)*sin(Pitch)*sin(Roll)) + cos(Pitch)*sin(Roll)*(Pz_1 - Z))) / (d_pxl*((Px_1 - X)*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - (Py_1 - Y)*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + cos(Pitch)*cos(Roll)*(Pz_1 - Z)));
//x_pxl_2 - (focal*(cos(Pitch)*cos(Yaw)*(Px_2 - X) - sin(Pitch)*(Pz_2 - Z) + cos(Pitch)*sin(Yaw)*(Py_2 - Y))) / (d_pxl*((Px_2 - X)*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - (Py_2 - Y)*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + cos(Pitch)*cos(Roll)*(Pz_2 - Z)));
//y_pxl_2 - (focal*((Py_2 - Y)*(cos(Roll)*cos(Yaw) + sin(Pitch)*sin(Roll)*sin(Yaw)) - (Px_2 - X)*(cos(Roll)*sin(Yaw) - cos(Yaw)*sin(Pitch)*sin(Roll)) + cos(Pitch)*sin(Roll)*(Pz_2 - Z))) / (d_pxl*((Px_2 - X)*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - (Py_2 - Y)*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + cos(Pitch)*cos(Roll)*(Pz_2 - Z)));
//x_pxl_3 - (focal*(cos(Pitch)*cos(Yaw)*(Px_3 - X) - sin(Pitch)*(Pz_3 - Z) + cos(Pitch)*sin(Yaw)*(Py_3 - Y))) / (d_pxl*((Px_3 - X)*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - (Py_3 - Y)*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + cos(Pitch)*cos(Roll)*(Pz_3 - Z)));
//y_pxl_3 - (focal*((Py_3 - Y)*(cos(Roll)*cos(Yaw) + sin(Pitch)*sin(Roll)*sin(Yaw)) - (Px_3 - X)*(cos(Roll)*sin(Yaw) - cos(Yaw)*sin(Pitch)*sin(Roll)) + cos(Pitch)*sin(Roll)*(Pz_3 - Z))) / (d_pxl*((Px_3 - X)*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - (Py_3 - Y)*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + cos(Pitch)*cos(Roll)*(Pz_3 - Z)));
//x_pxl_4 - (focal*(cos(Pitch)*cos(Yaw)*(Px_4 - X) - sin(Pitch)*(Pz_4 - Z) + cos(Pitch)*sin(Yaw)*(Py_4 - Y))) / (d_pxl*((Px_4 - X)*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - (Py_4 - Y)*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + cos(Pitch)*cos(Roll)*(Pz_4 - Z)));
//y_pxl_4 - (focal*((Py_4 - Y)*(cos(Roll)*cos(Yaw) + sin(Pitch)*sin(Roll)*sin(Yaw)) - (Px_4 - X)*(cos(Roll)*sin(Yaw) - cos(Yaw)*sin(Pitch)*sin(Roll)) + cos(Pitch)*sin(Roll)*(Pz_4 - Z))) / (d_pxl*((Px_4 - X)*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - (Py_4 - Y)*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + cos(Pitch)*cos(Roll)*(Pz_4 - Z)));
//vx + (qx - Z*sin(Pitch) + X*cos(Pitch)*cos(Yaw) + Y*cos(Pitch)*sin(Yaw)) / (abs(qx - Z*sin(Pitch) + X*cos(Pitch)*cos(Yaw) + Y*cos(Pitch)*sin(Yaw)) ^ 2 + abs(qz + X*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - Y*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + Z*cos(Pitch)*cos(Roll)) ^ 2 + abs(qy - X*(cos(Roll)*sin(Yaw) - cos(Yaw)*sin(Pitch)*sin(Roll)) + Y*(cos(Roll)*cos(Yaw) + sin(Pitch)*sin(Roll)*sin(Yaw)) + Z*cos(Pitch)*sin(Roll)) ^ 2) ^ (1 / 2);
//vy + (qy - X*(cos(Roll)*sin(Yaw) - cos(Yaw)*sin(Pitch)*sin(Roll)) + Y*(cos(Roll)*cos(Yaw) + sin(Pitch)*sin(Roll)*sin(Yaw)) + Z*cos(Pitch)*sin(Roll)) / (abs(qx - Z*sin(Pitch) + X*cos(Pitch)*cos(Yaw) + Y*cos(Pitch)*sin(Yaw)) ^ 2 + abs(qz + X*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - Y*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + Z*cos(Pitch)*cos(Roll)) ^ 2 + abs(qy - X*(cos(Roll)*sin(Yaw) - cos(Yaw)*sin(Pitch)*sin(Roll)) + Y*(cos(Roll)*cos(Yaw) + sin(Pitch)*sin(Roll)*sin(Yaw)) + Z*cos(Pitch)*sin(Roll)) ^ 2) ^ (1 / 2);
vz + (qz + X*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - Y*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + Z*cos(Pitch)*cos(Roll)) / (abs(qx - Z*sin(Pitch) + X*cos(Pitch)*cos(Yaw) + Y*cos(Pitch)*sin(Yaw)) ^ 2 + abs(qz + X*(sin(Roll)*sin(Yaw) + cos(Roll)*cos(Yaw)*sin(Pitch)) - Y*(cos(Yaw)*sin(Roll) - cos(Roll)*sin(Pitch)*sin(Yaw)) + Z*cos(Pitch)*cos(Roll)) ^ 2 + abs(qy - X*(cos(Roll)*sin(Yaw) - cos(Yaw)*sin(Pitch)*sin(Roll)) + Y*(cos(Roll)*cos(Yaw) + sin(Pitch)*sin(Roll)*sin(Yaw)) + Z*cos(Pitch)*sin(Roll)) ^ 2) ^ (1 / 2) ];
