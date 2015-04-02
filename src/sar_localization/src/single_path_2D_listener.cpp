#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "sar_localization/Imu.h"
#include "sar_localization/Csi.h"
#include "sar_localization/Motor.h"

#include "/opt/eigen/Eigen/Dense"
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <complex>
#include <math.h>
#include <assert.h>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
//#include <glib.h>

//for multiple processes processing
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
//#include <tuple>
//Constant definition for data processing
#define PI 				3.1415926
#define SIZE_LIMIT 		300
#define PROFILE_LIMIT 	20
#define INTERVAL_THRE 	13		//maximun interval must greater than X degree
#define CIRCLE_THRE 	173		//maxAngle-minAngle > 353 degree
#define LANDA 			0.06	//The aperture size is 6cm
#define R 				0.08	//The antenna interval
//for debug
#define STEP_SIZE 1

using namespace std;
using namespace Eigen;

typedef vector<pair<complex<double>, complex<double> > > PairCSIVector;
typedef Eigen::Matrix<double, 360, 1> Vector360d;

struct IMU
{
	double yaw;
	//double pitch;
	//double roll;
	double t_stamp_imu;
} Imu;

struct CSI
{
	PairCSIVector csi;		//CSI of antenna 1  and antenna 2 of receiver for all subcarriers
	double t_stamp_csi;
} Csi;

struct MOTOR
{
	double offset_yaw;
	double t_stamp_motor;
} Motor;

struct APS
{
	int apNum;
	int apID;
} Ap;

struct SAR
{
	int processingSize;
	bool csiReady;
	bool imuReady;
	int countD;
	double maxTimeDiff;
	double stdYaw;
	bool start;
	bool globalStart;	//This is a tag that start to record data as long as the normalization yaw has come
	bool stdFlag;
	bool autoSwitch;
	Vector360d multipathProfile;
	map<double, PairCSIVector> input;   //map<imu, csi1, csi2>
	map<double, CSI> CSIQueue;			//for data synchronization
	map<double, IMU> IMUQueue;
};

//Define some global variables
struct SAR Sar = {
	60,			//processing size
	false,		//csiReady
	false,		//imuReady
	0,			//countD
	0,			//maxTimeDiff
	0,			//stdYaw
	false,		//start
	false,		//globalStart
	false,		//stdFlag
	0,			//autoSwitch
};

ofstream myfile;

double RadianToDegree(double radian)
{
    return radian/PI*180;
}
double DegreeToRadian(double degree)
{
    return degree/180.0*PI;
}


double PowerCalculation(double alpha)
{
	double ret = 0;
	map<double, PairCSIVector>::iterator input_iter = Sar.input.begin();
	//int div = (int) input_iter->second.size();
	int div = Sar.processingSize;
	for(int i = 0; i < div; ++i)	//compute power for each subcarrier, each transmitter
	{
		input_iter = Sar.input.begin();
		complex<double> avgCsiHat(0, 0);
		while(input_iter != Sar.input.end() )
		{
			double input_yaw = DegreeToRadian(input_iter->first);
			complex<double> input_csi1 = input_iter->second[i].first;
			complex<double> input_csi2 = input_iter->second[i].second;
			double theta = 2*PI/LANDA*R*cos(alpha-input_yaw);
			double real_tmp = cos(theta);
			double image_tmp = sin(theta);
			complex<double> tmp (real_tmp, image_tmp);
			//cout << "Relative csi:" << CSI1[i]*conj(CSI2[i]) << endl;
			avgCsiHat += input_csi1*conj(input_csi2)*tmp;
			++input_iter;
		}
		avgCsiHat /= Sar.input.size();
		ret += avgCsiHat.real()*avgCsiHat.real() + avgCsiHat.imag()*avgCsiHat.imag();
	}
	ret /= div;
	//printf("Power calculation: %lf\n", ret);
	return ret;
}

void mysystem(const char *cmdstr)
{
	pid_t childPID;
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
	//parent process
	//...
}

bool DataCheck()
{
	bool ret = false;
	map<double, PairCSIVector>::iterator input_iter;
    input_iter = Sar.input.begin();
    double max_interval = 0;
    double min_interval = 0xffff;
    double maxAngle = 0;
    double minAngle = 0xffff;
    double prey = input_iter->first;
    double cury = 0xffff;
    ++input_iter;
    while(input_iter != Sar.input.end() )
    {
    	cury = input_iter->first;
        double interval = cury - prey;
        if(min_interval > interval) //find min interval
		{
        	min_interval = interval;
        }
        if(max_interval < interval)     //find max interval
        {
            max_interval = interval;
        }
        ++input_iter;
        prey = cury;
    }
    maxAngle = Sar.input.rbegin()->first;
    minAngle = Sar.input.begin()->first;
    double circle_distance = maxAngle - minAngle;
	if(max_interval < INTERVAL_THRE && circle_distance >= CIRCLE_THRE)
    {
    	printf("Max interval:%.2f, min interval:%lf, max angle:%.2f, min angle:%.2f, circle distance:%.2f\n", max_interval, min_interval, maxAngle,   minAngle, circle_distance);
    	ret = true;
        //check data consistancy
        map<double, PairCSIVector>::iterator input_iter = Sar.input.begin();
        int check_size = (int) input_iter->second.size();
        Sar.processingSize = check_size;
        do
        {
        	++input_iter;
            int s2 = (int) input_iter->second.size();
            if(check_size != s2)
            {
            	printf("\n!!!Inconsistent data! Resampling!%d-%d\n\n!"  , check_size, s2);
                Sar.input.clear();
                ret = false;
				//processingSize = 30;
                break;
            }
        }
        while(input_iter != --Sar.input.end() );
	}
    else if(Sar.input.size() > SIZE_LIMIT)   //it indicates some unpredictable situations causing very large input map
    {
    	printf("Too many samples! Clear and Resampling!\n");
        printf("Reason: \n");
        if(max_interval >= INTERVAL_THRE)
        {
        	printf("Too large interval! max_interval/threshold:%.2f/%d\n", max_interval, INTERVAL_THRE);
        }
        if(circle_distance < CIRCLE_THRE)
        {
        	printf("Not a circle! circle_distance/threshold:%.2f/%d\n"  , circle_distance, CIRCLE_THRE);
		}
        Sar.input.clear();
    }
	return ret;
}

int SAR_Profile_2D()
{
	if(Sar.csiReady && Sar.imuReady && Sar.globalStart)
	{
		Sar.csiReady = false;
		Sar.imuReady = false;
		double timeDifference = fabs(Csi.t_stamp_csi - Imu.t_stamp_imu);
		if(Sar.maxTimeDiff < timeDifference)
		{
			Sar.maxTimeDiff = timeDifference;
		}

		double std_input_yaw = RadianToDegree(Imu.yaw - Sar.stdYaw);
		if (std_input_yaw < 0)
		{
			std_input_yaw += 360;
		}
		else if(std_input_yaw >= 360)
		{
			std_input_yaw -= 360;
		}
		//for semi-circulate arc of rotation
		if(std_input_yaw >= 180)
		{
			std_input_yaw -= 180;
		}
		///////////////////////////////////
		//keep 0.1 value
		int tp = std_input_yaw*10;
		std_input_yaw = tp/10.0;
		//printf("STD_INPUT_YAW:%.2f\n",std_input_yaw );
		Sar.input[std_input_yaw] = Csi.csi;
		//input[RadianToDegree(yaw)] =  make_pair(csi1, csi2);
		//printf("std input %.2f\n", std_input_yaw);
		//if(dataIndex > 0 && dataIndex % sizeLimit == 0 && !start)
		//if(input.size() == sizeLimit || 1)
		if(Sar.input.size() > 1 )
		{
			//start data preprocessing
			Sar.start = DataCheck();
		}
		if(Sar.start)
		{
			printf("max T_D:%lf, ", Sar.maxTimeDiff);
			Sar.start = false;
			Sar.maxTimeDiff = 0;
			++Sar.countD;
			int ret_yaw;
			int resolution = STEP_SIZE;      //search resolution
			double maxPower = 0;
			myfile << "#" << Sar.countD << endl;
			for(int alpha = 0; alpha < 360; alpha += resolution)
			{
				double sumpow = 0;
       			for(int step = 0; step < STEP_SIZE; ++step)
      			{
					double alpha_r = (alpha+step)*PI/180.0;
       				double powtmp = PowerCalculation(alpha_r);
					sumpow += powtmp;
					//single path
					if(maxPower < powtmp)
					{
						maxPower= powtmp;
						ret_yaw = alpha;
					}
      			}
				myfile << sumpow << endl;
			}
			//int directPath = findDirectPath();
			//++dataIndex;
      		//return directPath;
			printf("Count:%d,maxPow: %0.3f,sample size:%d, ", Sar.countD, maxPower, (int) Sar.input.size() );
			Sar.input.clear();
			return ret_yaw;
		}
	}
	return -1;
}

// %Tag(CALLBACK)%

void motorCallback(const sar_localization::Motor::ConstPtr& msg)
{
	Motor.t_stamp_motor = msg->header.stamp.toNSec()*1e-6;
	Motor.offset_yaw = msg->offset_yaw;
	if(Motor.offset_yaw < 0.1 || fabs(Motor.offset_yaw-180) <= 4 || (360-Motor.offset_yaw) < 0.2 )
	{
		Sar.globalStart = true;
		Sar.stdFlag = false;
	}
}

void imuCallback(const sar_localization::Imu::ConstPtr& msg)
{
  	Imu.t_stamp_imu = msg->header.stamp.toNSec()*1e-6;
  	Imu.yaw = msg->yaw;
	Imu.yaw = Imu.yaw*PI/180.0;
	//if(!std_flag && input.size() > 2)
	if(!Sar.stdFlag)
	{
		Sar.stdYaw = Imu.yaw + DegreeToRadian(Motor.offset_yaw);
		if(Sar.stdYaw >= 2*PI)
		{
			Sar.stdYaw -= 2*PI;
		}
		Sar.stdFlag = true;
		//printf("T_D:%.2f, std_yaw:%.2f, yaw:%.2f, offset yaw:%.2f\n", fabs(t_stamp_motor-t_stamp_imu), RadianToDegree(std_yaw), RadianToDegree(yaw), offset_yaw);
	}
	Sar.IMUQueue.insert(make_pair(Imu.t_stamp_imu, Imu) );
  	//Sar.imuReady = true;
}

void csiCallback(const sar_localization::Csi::ConstPtr& msg)
{
	Csi.csi.clear();
  	Csi.t_stamp_csi = msg->header.stamp.toNSec()*1e-6;
	vector<double> real1;
	vector<double> image1;
	vector<double> real2;
	vector<double> image2;
	//Extract csi
	for(std::vector<double>::const_iterator pos = msg->csi1_real.data.begin(); pos != msg->csi1_real.data.end(); ++pos)
	{
		real1.push_back(*pos);
	}
	for(std::vector<double>::const_iterator pos = msg->csi1_image.data.begin(); pos != msg->csi1_image.data.end(); ++pos)
	{
		image1.push_back(*pos);
	}
	for(std::vector<double>::const_iterator pos = msg->csi2_real.data.begin(); pos != msg->csi2_real.data.end(); ++pos)
	{
		real2.push_back(*pos);
	}
	for(std::vector<double>::const_iterator pos = msg->csi2_image.data.begin(); pos != msg->csi2_image.data.end(); ++pos)
	{
		image2.push_back(*pos);
	}
	assert(real1.size() == image1.size() && real2.size() == image2.size() && real1.size() == image2.size() );
	for(int i = 0; i < (int) real1.size(); ++i)
	{
		complex<double> csi1tmp(real1[i], image1[i]);
		complex<double> csi2tmp(real2[i], image2[i]);
		Csi.csi.push_back(make_pair(csi1tmp, csi2tmp) );
	}
  	//Sar.csiReady = !msg->check_csi;
	Sar.CSIQueue.insert(make_pair(Csi.t_stamp_csi, Csi) );
	//csiReady = true;
}
// %EndTag(CALLBACK)%

void DataSynchronize()
{
	if(!Sar.IMUQueue.empty() && !Sar.CSIQueue.empty() )
	{
		map<double, IMU>::iterator IMUiter;
		map<double, CSI>::iterator CSIiter;
		double minTD = 0xffff;
		for(CSIiter = Sar.CSIQueue.begin(); CSIiter != Sar.CSIQueue.end(); ++CSIiter)
		{
			double csiT = CSIiter->first;
			for(IMUiter = Sar.IMUQueue.begin(); IMUiter != Sar.IMUQueue.end(); ++IMUiter)
			{
				double imuT = IMUiter->first;
				double TD = fabs(imuT - csiT);
				if(minTD > TD)
				{
					minTD = TD;
					Imu = IMUiter->second;
					Csi = CSIiter->second;
				}
			}
		}
		Sar.imuReady = true;
		Sar.csiReady = true;
		Sar.IMUQueue.clear();
		Sar.CSIQueue.clear();
	}
}

int main(int argc, char **argv)
{
	Sar.multipathProfile.setZero();
	Ap.apNum = 2;
	Ap.apID = 0;
	ros::init(argc, argv, "listener");
  	ros::NodeHandle n;
  	ros::Subscriber sub1 = n.subscribe("imu", 10000, imuCallback);
	ros::Subscriber sub2 = n.subscribe("csi", 10000, csiCallback);
	ros::Subscriber sub3 = n.subscribe("motor", 10000, motorCallback);
	if(Sar.autoSwitch)
	{
		system("iwconfig wlan0 essid TP5G1");
		printf("iwconfig to TP5G1\n");

		system("dhclient wlan0");
		printf("dhclient from TP5G1 completed\n");

		system("iwconfig wlan0 essid TP5G2");
		printf("iwconfig to TP5G2\n");

		system("dhclient wlan0");
    	printf("dhclient from TP5G2 completed\n");

		mysystem("ping -q -n -i 0.02 192.168.0.3");
	}
	myfile.open("power.txt");
	ros::spinOnce();		//empty the queue
	Sar.csiReady = false;
	Sar.imuReady = false;
	//input.clear();
	//std_flag = false;
	while(n.ok() )
	{
		//spinner.spinOnce();
		ros::spinOnce();
		DataSynchronize();
		int angle = SAR_Profile_2D();
		if(angle >= 0)
		{
			printf("Alpha:%d\n", angle);
            //Switch to another AP
			if(Sar.autoSwitch)
			{
       	    	switch(Ap.apID)
            	{
            	case 0:
					//Switch to from AP1 to AP2
              		Ap.apID = (Ap.apID+1)%Ap.apNum;
                	system("pkill -n ping");   //kill the child process first
               		system("iwconfig wlan0 essid TP5G1");
               		printf("Switch to TP5G1 and start ping\n");
              		mysystem("ping -q -n -i 0.02 192.168.0.2");
					ros::spinOnce();
					Sar.csiReady = false;
					Sar.imuReady = false;
					//input.clear();
               		break;
            	case 1:
              		Ap.apID = (Ap.apID+1)%Ap.apNum;
              		system("pkill -n ping");      //kill the child process first
              		system("iwconfig wlan0 essid TP5G2");
               		printf("Switch to TP5G2 and start ping\n");
        	   		mysystem("ping -q -n -i 0.02 192.168.0.3");
					ros::spinOnce();
					Sar.csiReady = false;
					Sar.imuReady = false;
					//input.clear();
           			break;
           		}
			}
		}
	}

	myfile.close();
	if(Sar.autoSwitch)
	{
		system("pkill -n ping");
	}
	return 0;
}
