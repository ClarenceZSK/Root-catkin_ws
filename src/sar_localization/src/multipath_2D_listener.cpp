#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"
#include "sar_localization/Imu.h"
#include "sar_localization/Csi.h"

#include "/opt/eigen/Eigen/Dense"
#include <vector>
#include <complex>
#include <math.h>
#include <assert.h>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>
#include <glib.h>

//for multiple processes processing
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
//#include <tuple>
//Global variables for data processing
#define PI 3.1415926
#define sizeLimit 200
#define profileLimit 20
//for debug
#define stepSize 1

using namespace std;
using namespace Eigen;

//vector<complex<double> > CSI1;		//CSI from antenna 1
//vector<complex<double> > CSI2;		//CSI from antenna 2
complex<double>  CSI1[sizeLimit];
complex<double>  CSI2[sizeLimit];
double orientation[sizeLimit];
double t_stamp_csi;			//time stamp of csi
double t_stamp_imu;			//time stamp of imu
bool csi_ready = false;
bool imu_ready = false;
//double pitch;
double yaw;
complex<double> csi1;
complex<double> csi2;
//multipath effect processing
Eigen::VectorXi peak_mat(360);
int vib_threshold = 0;			//The peak vibration allowance, 0 means the persistent peak must be the degree exatly the same as before
int comp_time = 2;			//The times of comparison of multiple power profiles for peak elimination, it should be greater than 1

double landa = 0.06;			//The aperture size is 6cm
double r = 0.116;			//The radius (antenna interval)

int dataIndex = 0;
int count_d = 0;
bool start = false;

int AP_ID = 0;		//The associated AP ID
int AP_NUM = 2;		//The number of available APs
pid_t childPID = -2;

//for debug
int preIdx = 0;
double maxT_D = 0;
ofstream myfile;

double PowerCalculation(double alpha)
{
	complex<double> avgCsiHat (0, 0);

	for(int i = 0; i < sizeLimit; ++i)
	{
		double theta = 2*PI/landa*r*cos(alpha-orientation[i]);
		double real_tmp = cos(theta);
		double image_tmp = sin(theta);
		complex<double> tmp (real_tmp, image_tmp);
		//cout << "Relative csi:" << CSI1[i]*conj(CSI2[i]) << endl;
		avgCsiHat += CSI1[i]*conj(CSI2[i])*tmp;
	}
	avgCsiHat /= sizeLimit;
	double ret = avgCsiHat.real()*avgCsiHat.real() + avgCsiHat.imag()*avgCsiHat.imag();
	//printf("Power calculation: %lf\n", ret);
	return ret;
}


Eigen::VectorXi peakElimination(Eigen::VectorXi cur_peak_mat)
{
	Eigen::VectorXi ret;
	for(int i  = 0; i < cur_peak_mat.size(); ++i)
	{
		if(cur_peak_mat(i) == true)
		{
			//start match the previous peak matrix in a group way
			bool peakOr = false;
			for(int j = i-vib_threshold; j <= i+vib_threshold; ++j)
			{
				int dx = (j+360)%360;
	            peakOr = (peakOr || peak_mat(dx) );
			}
			ret(i) = peakOr;
		}
		else
		{
			ret(i) = 0;
		}
	}

	return ret;
}

int mysystem(const char *cmdstr)
{
	if(cmdstr == NULL)
	{
		cerr << "NULL command is not allowed" << endl;
		exit(1);
	}
	if( (childPID = fork()) < 0)
	{
		cerr << "Fail to fork" << endl;
	}
	else if(childPID == 0)
	{
		execl("/bin/sh", "sh", "-c", cmdstr, (char *) 0);
		_exit(127);
	}	
	else	//parent process
	{
		return childPID;
	}
	return childPID;
}

vector<int> countPeak()
{
	vector<int> ret;
	ret.clear();
	for (int i = 0; i < peak_mat.size(); ++i)
	{
		if (peak_mat(i) == 1)
		{
			ret.push_back(i);	
		}
	}
	return ret;
}


vector<int> SAR_Profile_2D()
{
	vector<int> ret;
	ret.clear();
	double timeDifference = fabs(t_stamp_csi-t_stamp_imu);
	if(maxT_D < timeDifference)
	{
		maxT_D = timeDifference;
	}
	orientation[dataIndex % sizeLimit] = yaw;
	CSI1[dataIndex % sizeLimit] = csi1;
	CSI2[dataIndex % sizeLimit] = csi2;
	if(dataIndex > 0 && dataIndex % sizeLimit == 0 && !start)
	{
		printf("Start!\n");
		start = true;
	}
	if(start)
	{
		Eigen::VectorXi cur_peak_mat;
		cur_peak_mat.setZero();
		printf("max T_D:%lf, ", maxT_D);
		start = false;
		maxT_D = 0;
		++count_d;
		
		//When csi and imu data vectors reach size limit, start angle generation
		int resolution = 1;      //search resolution

 		bool up = false;
		bool initial_down = false;
		double prePow = PowerCalculation(0);

		myfile << "#" << count_d << endl;
		for(int alpha = 1; alpha < 360; alpha += resolution)
		{
			double alpha_r = alpha*PI/180.0;
			double powtmp = PowerCalculation(alpha_r);
			if(powtmp > prePow)
			{
				up = true;
			}
			else if(alpha == 1)
			{
				initial_down = true;
			}

			if(up)		//only if up, we wait down for peak detection
			{
				if(powtmp < prePow)		//down
				{
					cur_peak_mat(alpha) = 1;
					up = false;			//once peak detected, we wait for up signal again
				}
			}
			myfile << cur_peak_mat(alpha) << endl;
			prePow = powtmp;
		}
		//after the loop, we has to take care the initial point that it may be also a peak
		double powtmp = PowerCalculation(0);
		if(powtmp > prePow)
		{
			up = true;
		}
		if(up && initial_down)
		{
			cur_peak_mat(0) = 1;
		}
		myfile << cur_peak_mat(0) << endl;

		++dataIndex;
		
		if(count_d == 1)
		{
			peak_mat = cur_peak_mat;
			ret = countPeak();
			printf("Count:%d,peak_num: %d\n",count_d, (int) ret.size());
		}
		else
		{
			peak_mat = peakElimination(cur_peak_mat);
			ret = countPeak();
			printf("Count:%d,peak_num: %d\n",count_d, (int) ret.size());
			return ret;
		}
	}
	++dataIndex;
	return ret;
}

// %Tag(CALLBACK)%
void imuCallback(const sar_localization::Imu::ConstPtr& msg)
{ 
  	t_stamp_imu = msg->header.stamp.toNSec()*1e-6;
  	yaw = msg->yaw;
	yaw = yaw*PI/180.0;
  	imu_ready = true;
}

void csiCallback(const sar_localization::Csi::ConstPtr& msg)
{
  	t_stamp_csi = msg->header.stamp.toNSec()*1e-6;
  	complex<double> csi1tmp (msg->csi1_real, msg->csi1_image);
  	csi1 = csi1tmp; 
  	complex<double> csi2tmp (msg->csi2_real, msg->csi2_image);
  	csi2 = csi2tmp; 
  	csi_ready = true;
}   
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "listener");
	peak_mat.setZero();
  	ros::NodeHandle n;

  	ros::Subscriber sub1 = n.subscribe("imu", 1, imuCallback);
	ros::Subscriber sub2 = n.subscribe("csi", 1, csiCallback);

	pid_t cpid;
	system("iwconfig wlan0 essid TP5G1");
	printf("iwconfig to TP5G1\n");

	system("dhclient wlan0");
	printf("dhclient from TP5G1 completed\n");

	system("iwconfig wlan0 essid TP5G2");
	printf("iwconfig to TP5G2\n");

	system("dhclient wlan0");
	printf("dhclient from TP5G2 completed\n");
	
	system("iwconfig wlan0 essid TP5G1");
	printf("Switch to TP5G1 and start ping\n");
	cpid = mysystem("ping -q -n -i 0.05 192.168.0.2");
		
	myfile.open("power.txt");

	while(ros::ok() )
	{
		//spinner.spinOnce();
		ros::spinOnce();
		if(csi_ready && imu_ready)
		{
			csi_ready = false;
			imu_ready = false;
			vector<int> peakAngles = SAR_Profile_2D();	
			if(!peakAngles.empty() )
			{
				//print peaks after the elimination

				for(int i = 0; i < peakAngles.size(); ++i)
				{
				}
				printf("Alpha:%d\n", angle);
				//Switch to another AP
				switch(AP_ID)
				{
				case 0:
					//Switch to from AP1 to AP2
					AP_ID = (AP_ID+1)%AP_NUM;
					system("pkill -INT -n ping");   //kill the child process first
					system("iwconfig wlan0 essid TP5G2");
					printf("Switch to TP5G2 and start ping\n");
					cpid = mysystem("ping -q -n -i 0.05 192.168.0.3");
					break;
				case 1:
					AP_ID = (AP_ID+1)%AP_NUM;
					system("pkill -INT -n ping");      //kill the child process first
					system("iwconfig wlan0 essid TP5G1");
					printf("Switch to TP5G1 and start ping\n");
					cpid = mysystem("ping -q -n -i 0.05 192.168.0.2");
					break;
				}
			}
		}
	}

	myfile.close();
	system("pkill -INT -n ping");
	return 0;
}
