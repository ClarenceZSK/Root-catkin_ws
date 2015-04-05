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
	double yaw;				//Radian
	double pitch;			//Radian
	//double roll;
	double t_stamp_imu;
} Imu;

struct CSI
{
	PairCSIVector csi;		//CSI of antenna 1 and antenna 2 of receiver for all subcarriers
	double t_stamp_csi;
} Csi;

struct COM					//Combine CSI and IMU
{
	IMU Comimu;
	CSI Comcsi;
} Com;

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
	bool dataControl;
	int outputFrequency;
	int dataSizeControl;
	Vector360d multipathProfile;
	map<double, COM> input;   //map<imu, csi1, csi2>
	map<double, CSI> CSIQueue;			//for data synchronization
	map<double, IMU> IMUQueue;
	vector<pair<IMU, CSI> > SyncInput;
	vector<double> inputTimeSequence;
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
	0,			//dataControl
	0,			//outputFrequency
	60,			//dataSizeControl
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

double PowerCalculation(double alpha, double beta)
{
	double ret = 0;
	map<double, COM>::iterator input_iter = Sar.input.begin();
	//int div = (int) input_iter->second.size();
	int div = Sar.processingSize;
	for(int i = 0; i < div; ++i)	//compute power for each subcarrier, each transmitter
	{
		input_iter = Sar.input.begin();
		complex<double> avgCsiHat(0, 0);
		while(input_iter != Sar.input.end() )
		{
			double input_yaw = DegreeToRadian(input_iter->first);
			double input_pitch = input_iter->second.Comimu.pitch;	//Radian
			complex<double> input_csi1 = input_iter->second.Comcsi.csi[i].first;
			complex<double> input_csi2 = input_iter->second.Comcsi.csi[i].second;
			double theta = 2*PI/LANDA*R*cos(alpha-input_yaw)*sin(beta-input_pitch);
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
	map<double, COM>::iterator input_iter;
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
        map<double, COM>::iterator input_iter = Sar.input.begin();
        int check_size = (int) input_iter->second.Comcsi.csi.size();
        Sar.processingSize = check_size;
        do
        {
        	++input_iter;
            int s2 = (int) input_iter->second.Comcsi.csi.size();
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

void InputData()
{
	//cout << "Input " << Sar.SyncInput.size() << "synced data." << endl;
	for (int i = 0; i < (int) Sar.SyncInput.size(); ++i)
	{
		Imu = Sar.SyncInput[i].first;
		Csi = Sar.SyncInput[i].second;
		Com.Comimu = Imu;
		Com.Comcsi = Csi;
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
    	//0.1 precision
    	int tp = std_input_yaw*10;
    	std_input_yaw = tp/10.0;
    	//printf("STD_INPUT_YAW:%.2f\n",std_input_yaw );
    	Sar.input[std_input_yaw] = Com;
    	Sar.inputTimeSequence.push_back(std_input_yaw);
	}
	Sar.SyncInput.clear();
}

vector<int> SAR_Profile_3D()
{
	vector<int> ret;
	if(Sar.csiReady && Sar.imuReady && Sar.globalStart)
	{
		Sar.csiReady = false;
		Sar.imuReady = false;
		InputData();
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
			int ret_pitch;
			double maxPower = 0;
			myfile << "#" << Sar.countD << endl;
			for(int alpha = 0; alpha < 360; ++alpha)
			{
				for(int beta = 0; beta < 180; ++beta)
				{
       				double powtmp = PowerCalculation(DegreeToRadian(alpha), DegreeToRadian(beta) );
					if(maxPower < powtmp)
					{
						maxPower= powtmp;
						ret_yaw = alpha;
						ret_pitch = beta;
					}
					myfile << powtmp << endl;
				}
			}
			printf("Count:%d,maxPow: %0.3f,sample size:%d, ", Sar.countD, maxPower, (int) Sar.input.size() );
			ret.push_back(ret_yaw);
			ret.push_back(ret_pitch);
			return ret;
		}
	}
	return ret;
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
	Imu.yaw = DegreeToRadian(Imu.yaw);
	Imu.pitch = msg->pitch;
	Imu.pitch = DegreeToRadian(Imu.pitch);
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
		if(Sar.CSIQueue.size() <= Sar.IMUQueue.size() )
		{
			for(CSIiter = Sar.CSIQueue.begin(); CSIiter != Sar.CSIQueue.end(); ++CSIiter)
			{
				//double minTD = 0xffff;
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
				//Sar.SyncInput.push_back(make_pair(Imu, Csi) );
			}
			Sar.SyncInput.push_back(make_pair(Imu, Csi) );
		}
		else
		{
			for(IMUiter = Sar.IMUQueue.begin(); IMUiter != Sar.IMUQueue.end(); ++IMUiter)
			{
				//double minTD = 0xffff;
                double imuT = IMUiter->first;
				for(CSIiter = Sar.CSIQueue.begin(); CSIiter != Sar.CSIQueue.end(); ++CSIiter)
				{
					double csiT = IMUiter->first;
                    double TD = fabs(imuT - csiT);
                    if(minTD > TD)
                    {
                        minTD = TD;
                        Imu = IMUiter->second;
                        Csi = CSIiter->second;
                    }
				}
				//Sar.SyncInput.push_back(make_pair(Imu, Csi) );
			}
			Sar.SyncInput.push_back(make_pair(Imu, Csi) );
		}
		Sar.imuReady = true;
		Sar.csiReady = true;
		//cout << "IMUQueue size:" << Sar.IMUQueue.size() << endl;
		//cout << "CSIQueue size:" << Sar.CSIQueue.size() << endl;
		Sar.IMUQueue.clear();
		Sar.CSIQueue.clear();
	}
}

void DataControl()
{
	vector<double>::iterator inputSeqIter;
	while((int) Sar.input.size() > Sar.dataSizeControl)
	{
		//Get rid of older data from input
		inputSeqIter = Sar.inputTimeSequence.begin();
		double ty = *inputSeqIter;
		map<double, COM>::iterator input_iter;
		input_iter = Sar.input.find(ty);
		if(input_iter != Sar.input.end() )
		{
			Sar.input.erase(input_iter);
		}
		Sar.inputTimeSequence.erase(inputSeqIter);
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
	//std_flag = false;
	while(n.ok() )
	{
		//spinner.spinOnce();
		ros::spinOnce();
		DataSynchronize();
		if(Sar.dataControl)
		{
			DataControl();		//Limit a upper bound of the size of input
		}
		vector<int> angles;
		angles = SAR_Profile_3D();
		if(!angles.empty() )
		{
			assert(angles.size() == 2);
			if(!Sar.dataControl)
			{
				Sar.input.clear();
			}
			printf("Alpha & beta:%d & %d\n", angles[0], angles[1]);
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
