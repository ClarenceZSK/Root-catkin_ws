#ifndef SAR_H
#define SAR_H

#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include "/opt/eigen/Eigen/Dense"
#include <vector>
#include <complex>
#include <math.h>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <map>
#include <set>
#include <queue>
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>

#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#include <glib.h>

#define DATA_SIZE 	1000
#define PI 			3.1415926
#define R			0.24
#define STEP_SIZE	1
#define RSL			1
#define AP_NUM		1

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
	MOTOR() {stdYaw = -1;}
	bool nearStartPoint();
	double t_stamp;
	double stdYaw;
};

class AP
{
public:
	AP();
	void mysystem(const char *cmdstr);
	void init();

	bool autoSwitch;
	int apID;
	int apNum;
};

typedef vector<pair<Matrix3d, CSI> > SharedVector;
typedef pair<Vector3d, CSI> InputPair;
typedef Matrix<std::complex<double>, 360, DATA_SIZE> Matrix360CD;
typedef Matrix<std::complex<double>, 360, 1> Vector360CD;

class SAR
{
public:
	SAR();
	void init();

	void processIMU(double t, Vector3d angular_velocity);
	double radianToDegree(double radian);
	double degreeToRadian(double degree);
	double powerCalculation(Vector3d alpha);
	void inputData(SharedVector* shared_ptr);
	bool checkData();
	bool selectData();
	double SAR_Profile_2D();
	double finerResolution(int c_yaw);
	vector<int> SAR_Profile_3D();		//search alpha and beta
	vector<int> SAR_Profile_3D_fast();	//search alpha with the 2D solution first, then fix alpha and search beta
	void switchAP();

	static const int WINDOW_SIZE	= 1000;
	static const int INTERVAL		= 13;
	static const int ARC			= 173;
	double Landa;
	int frame_count;
	int round_count;
	int input_count[AP_NUM];
	int newestIdx;
	double current_time;
	//debug
	double preAngle;
	double maxPow;
	//bool initStart;
	ofstream myfile;
	CSI csi;
	//MOTOR motor;
	AP ap;
	Vector3d baseDirection;
	Matrix3d imuAngular[WINDOW_SIZE];
	InputPair input[AP_NUM][DATA_SIZE];	//direct vector, CSI vectors
	vector<InputPair> selectedInput;	//selected a half-circle data from input to calculate

	//optimize my code
	//Matrix360CD tempCal;
	//Vector360CD sumV;
	////////////////////////////////////////
	////////////////////////////////////////
	sensor_msgs::ChannelFloat32 channel_msg;
	sensor_msgs::PointCloud wifi_msg;
	geometry_msgs::Point32 point_msg;
	///////////////////////////////////////
	GMutex mutex;
	SharedVector inputQueue;
};

#endif
