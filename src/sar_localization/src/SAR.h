#ifndef SAR_H
#define SAR_H

#include "ros/ros.h"
#include "/opt/eigen/Eigen/Dense"
#include <vector>
#include <complex>
#include <math.h>

#define PI 	3.1415926
#define R	0.08

using namespace std;
using namespace Eigen;

typedef vector<pair<complex<double>, complex<double> > > PairCSIVector;

class CSI
{
public:
	PairCSIVector pairVector;
	double t_stamp;
};

class MOTOR
{
public:
	MOTOR() {}
	bool nearStartPoint();
	double t_stamp;
	double stdYaw;
};

class AP
{
public:
	AP();
	void mysystem(const char *cmdstr);
private:
	bool autoSwitch;
	int apID;
	int apNum;
};

class SAR
{
public:
	SAR();
	void processIMU(double t, Vector3d angular_velocity);
	double radianToDegree(double radian);
	double degreeToRadian(double degree);
	double powerCalculation(Vector3d alpha);
	void inputData();
	bool checkData();
	int SAR_Profile_2D();
	vector<int> SAR_Profile_3D();
private:
	static const int WINDOW_SIZE	= 500;
	static const int INTERVAL		= 13;
	static const int CIRCLE			= 173;
	double Landa;
	int frame_count;
	double current_time;
	bool dataReady;
	CSI csi;
	Vector3d baseDirection;
	MOTOR motor;
	Matrix3d imuAngular[WINDOW_SIZE];
	vector<pair<Vector3d, CSI> > input;	//direct vector, CSI vectors
};

#endif
