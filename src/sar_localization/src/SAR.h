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

#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#include <glib.h>

#define DATA_SIZE 	90
#define PI 			3.1415926
#define R			0.08
#define STEP_SIZE	1
#define AP_NUM		3

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
	int SAR_Profile_2D();
	vector<int> SAR_Profile_3D();		//search alpha and beta
	vector<int> SAR_Profile_3D_fast();	//search alpha with the 2D solution first, then fix alpha and search beta
	void switchAP();

	static const int WINDOW_SIZE	= 500;
	static const int INTERVAL		= 13;
	static const int ARC			= 173;
	double Landa;
	int frame_count;
	int round_count;
	int input_count[AP_NUM];
	double current_time;
	bool initStart;
	ofstream myfile;
	CSI csi;
	MOTOR motor;
	AP ap;
	Vector3d baseDirection;
	Matrix3d imuAngular[WINDOW_SIZE];
	InputPair input[AP_NUM][DATA_SIZE];	//direct vector, CSI vectors
	sensor_msgs::ChannelFloat32 channel_msg;
	sensor_msgs::PointCloud wifi_msg;
	geometry_msgs::Point32 point_msg;

	GMutex mutex;
	SharedVector inputQueue;
};

#endif
