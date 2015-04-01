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
#include <complex>
#include <math.h>
#include <assert.h>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>
//#include <glib.h>
//for multiple processes processing
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
//#include <tuple>
//Constant definition for data processing
#define PI              3.1415926
#define SIZE_LIMIT      300
#define PROFILE_LIMIT   20
#define INTERVAL_THRE   13      //maximun interval must greater than X degree
#define CIRCLE_THRE     173     //maxAngle-minAngle > 353 degree
#define LANDA           0.06    //The aperture size is 6cm
#define R               0.08    //The antenna interval
#define COMP_THRE 		3
#define GRANULARITY 	360
//for debug
#define STEP_SIZE 		1

using namespace std;
using namespace Eigen;

typedef vector<pair<complex<double>, complex<double> > > PairCSIVector;
typedef Eigen::Matrix<double, 360, 1> Vector360d;
typedef Eigen::Matrix<int, GRANULARITY, 1> VectorGi;

struct IMU
{
    double yaw;
    //double pitch;
    //double roll;
    double t_stamp_imu;
} Imu;

struct CSI
{
    PairCSIVector csi;      //CSI of antenna 1  and antenna 2 of receiver for   all subcarriers
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
	int vib;
	int compTime;
    double maxTimeDiff;
    double stdYaw;
    bool start;
    bool globalStart;   //This is a tag that start to record data as long as the normalization yaw has come
    bool stdFlag;
	bool peakVanished;
	bool reset;
    bool autoSwitch;
    //Vector360d multipathProfile;
	VectorGi peakMat;
    map<double, PairCSIVector> input;   //map<imu, csi1, csi2>
	map<int, int> angleSet;
};

//Define some global variables
struct SAR Sar = {
    60,         //processing size
    false,      //csiReady
    false,      //imuReady
    0,          //countD
	7*GRANULARITY/360,	//vib
	COMP_THRE,	//compTime;
    0,          //maxTimeDiff
    0,          //stdYaw
    false,      //start
    false,      //globalStart
    false,      //stdFlag
	false,		//peakVanish
	false,		//reset
    0,          //autoSwitch
};



//multipath effect processing
//Eigen::VectorXi peak_mat(granularity);
//int vib_threshold = 7*granularity/360;			//The peak vibration allowance, 0 means the persistent peak must be the degree exatly the same as before
//int comp_time = comp_threshold;			//The times of comparison of multiple power profiles for peak elimination, it should be greater than 1
//int processingSize = 60;

//double landa = 0.06;			//The aperture size is 6cm
//double r = 0.10;			//The radius (antenna interval)
//Eigen::VectorXi std_profile(360);	//Store a standard multipath profile to recover from 0 result
//bool peakVanished = false;	//It indicates that after angle elimination, there is no angle left. In this case, we need to recover for continuing experiment
//bool reset = 0;

//int count_d = 0;
//bool start = false;
//bool globalStart = false;

//yaw normalize
//bool std_flag = true;
//double std_yaw = -1;
//double offset_yaw = 0;

//auto switch
//int AP_ID = 0;		//The associated AP ID
//int AP_NUM = 2;		//The number of available APs
//pid_t childPID = -2;
//bool autoSwitch = 0;

//AoA localization
multimap<int, int> angleSet;	//The set of angles for localization: <AP_ID, angles>. Probably, more than 1 angles will be selected


//for debug
//int preIdx = 0;
//double maxT_D = 0;
ofstream myfile1;		//power
ofstream myfile2;		//peaks
//ofstream myfile3;		//statistics
int test_target = 220*GRANULARITY/360;	//test peak near XX degree
vector<int> targetDistance;
int detect_range1 = Sar.vib;
int detect_range2 = detect_range1+1;
int detect_range3 = detect_range2+1;
int detect_range4 = detect_range3+1;
int count1 = 0;
int count2 = 0;
int count3 = 0;
int count4 = 0;
int count_detected = 0;
int count_finalOutput = 0;

double RadianToDegree(double radian)
{
	return radian/PI*180;
}
double DegreeToRadian(double degree)
{
	return degree/180.0*PI;
}

void testNearTarget(int alpha)
{
	if(alpha >= test_target-detect_range1 && alpha <= test_target+detect_range1)
	{
		++count1;
	}
	if(alpha >= test_target-detect_range2 && alpha <= test_target+detect_range2)
	{
		++count2;
	}
	if(alpha >= test_target-detect_range3 && alpha <= test_target+detect_range3)
	{
		++count3;
	}
	if(alpha >= test_target-detect_range4 && alpha <= test_target+detect_range4)
	{
		++count4;
	}

}

double PowerCalculation(double alpha)
{
	double ret = 0;
	map<double, PairCSIVector>::iterator input_iter = Sar.input.begin();
	//int div = (int) input_iter->second.size();
	int div = Sar.processingSize;
	for(int i  = 0; i < div; ++i)
	{
		complex<double> avgCsiHat (0, 0);
		input_iter = Sar.input.begin();
		while(input_iter != Sar.input.end() )
		{
			double input_yaw = DegreeToRadian(input_iter->first);
			complex<double> input_csi1 = input_iter->second[i].first;
			complex<double> input_csi2 = input_iter->second[i].second;
			//cout << "input csi1:"<< input_csi1 <<" csi2:" << input_csi2 << endl;
			double theta = 2*PI/LANDA*R*cos(alpha-input_yaw);
			double real_tmp = cos(theta);
			double image_tmp = sin(theta);
			complex<double> tmp (real_tmp, image_tmp);

			avgCsiHat += input_csi1*conj(input_csi2)*tmp;
			++input_iter;
		}
		avgCsiHat /= Sar.input.size();
		ret += avgCsiHat.real()*avgCsiHat.real() + avgCsiHat.imag()*avgCsiHat.imag();
	}
	ret /= div;
	//printf("Pwr: %lf, csi size: %d\n", ret, div);
	return ret;
}


Eigen::VectorXi peakElimination(Eigen::VectorXi cur_peak_mat)
{
	//Eigen::VectorXi ret(granularity);
	VectorGi ret;
	for(int i  = 0; i < cur_peak_mat.size(); ++i)
	{
		if(cur_peak_mat(i) == true)
		{
			//start match the previous peak matrix in a group way
			bool peakOr = false;
			for(int j = i-Sar.vib; j <= i+Sar.vib; ++j)
			{
				int dx = (j+GRANULARITY)%GRANULARITY;
				peakOr = (peakOr || Sar.peakMat(dx) );
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

vector<int> countPeak()
{
	vector<int> ret;
	for (int i = 0; i < Sar.peakMat.size(); ++i)
	{
		myfile2 << Sar.peakMat(i) << endl;
		if (Sar.peakMat(i) == 1)
		{
			ret.push_back(i);
		}
	}
	return ret;
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
        if(max_interval < interval) //find max interval
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
        printf("Max interval:%.2f, min interval:%lf, max angle:%.2f, min angle:%.2f, circle distance:%.2f, sample size:%d\n", max_interval, min_interval, maxAngle, minAngle, circle_distance, (int) Sar.input.size() );
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
    else if(Sar.input.size() > SIZE_LIMIT)   //it indicates some   unpredictable situations causing very large input map
    {
        printf("Too many samples! Clear and Resampling!\n");
        printf("Reason: \n");
        if(max_interval >= INTERVAL_THRE)
        {
            printf("Too large interval! max_interval/threshold:%.2f/%d\n", max_interval, INTERVAL_THRE);
			//for debug
			/*
			map<double, PairCSIVector>::iterator input_iter;
			for(input_iter = Sar.input.begin(); input_iter != Sar.input.end(); ++input_iter)
			{
				cout << input_iter->first << ", ";
			}
			cout << endl;
			*/
			////////////////////////////
        }
        if(circle_distance < CIRCLE_THRE)
        {
            printf("Not a circle! circle_distance/threshold:%.2f/%d\n"  , circle_distance, CIRCLE_THRE);
        }
		Sar.reset = 1;
		Sar.compTime = COMP_THRE;
        Sar.input.clear();
    }
	//limit compare times
    if(Sar.compTime == 0)
    {
    	Sar.compTime = COMP_THRE;
        printf("\nComp %d times, clear and restart!\n\n", COMP_THRE);
        ret = false;
        Sar.input.clear();
        Sar.reset = 1;
    }
    return ret;
}


vector<int> SAR_Profile_2D()
{
	vector<int> ret;
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
		if (std_input_yaw >= 180)
		{
			std_input_yaw -= 180;
		}
		///////////////////////////////////
		int tp = std_input_yaw*10;
		std_input_yaw = tp/10.0;
		//printf("csi size:%d\n",(int) csi.size() );
		Sar.input[std_input_yaw] = Csi.csi;
		if(Sar.input.size() > 1 )
		{
			//start data preprocessing
			Sar.start = DataCheck();
		}
		if(Sar.start)
		{
			//for debug, print input angles
			bool test = 0;
			if(test)
			{
				map<double, PairCSIVector>::iterator input_iter = Sar.input.begin();
				cout << "input angles: ";
				while(input_iter != Sar.input.end() )
				{
					cout << input_iter->first << ", ";
					++input_iter;
				}
				cout << endl;
			}
			/////////////////////////////////
			Sar.compTime--;
			VectorGi cur_peak_mat;
			cur_peak_mat.setZero();
			printf("max T_D:%lf, ", Sar.maxTimeDiff);
			Sar.start = false;
			Sar.maxTimeDiff = 0;
			++Sar.countD;
			int resolution = 1;      //search resolution
			bool up = false;
			bool initial_down = false;
			double prePow = PowerCalculation(0);
			myfile1 << "#" << Sar.countD << endl;	//for power
			myfile1 << prePow << endl;
			myfile2 << "#" << Sar.countD << endl;	//for peaks
			for(int alpha = 1; alpha < GRANULARITY; alpha += resolution)
			{
				double alpha_r = alpha/(GRANULARITY/360.0)*PI/180.0;
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
						//for debug
						testNearTarget(alpha);
						///////////
						up = false;			//once peak detected, we wait for up signal again
					}
				}
				myfile1 << powtmp << endl;
				//myfile2 << cur_peak_mat(alpha) << endl;
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
			myfile2 << cur_peak_mat(0) << endl;

			//for debug, add statistical information
			//getStatistics(cur_peak_mat);
			///////////////////////////////////////

			if(Sar.countD == 1 || Sar.reset)
			{
				Sar.reset = 0;
				Sar.peakMat = cur_peak_mat;
				ret = countPeak();
				printf("Vib:%d,Count:%d,peak_num: %d\n", Sar.vib, Sar.countD, (int) ret.size());
				Sar.compTime = COMP_THRE;
				Sar.input.clear();
			}
			else	//start peak elimination
			{
				Sar.peakMat = peakElimination(cur_peak_mat);
				ret = countPeak();
				printf("Vib:%d,Count:%d,peak_num: %d\n", Sar.vib, Sar.countD, (int) ret.size());
				Sar.input.clear();
				//some additional processing includes:
				//1. peak re-elimination from the last moment multipath profile
				//2. extract the potential correct result if the ret size is 1
				if(ret.empty() )
				{
					Sar.peakVanished = true;
					//peak_mat.setZero();	//store the last moment peak situation
					Sar.reset = 1;
					Sar.compTime = COMP_THRE;
				}
				else if(ret.size() == 1)		//It indicates that we have obtained the final converged result. At this time, we restart the process based on the current peak situation
				{
					Sar.reset = 1;
					Sar.compTime = COMP_THRE;
					//peak_mat.setZero();
				}
				return ret;
			}
		}
	}

	return ret;
}


// %Tag(CALLBACK)%
void motorCallback(const sar_localization::Motor::ConstPtr& msg)
{
	Motor.t_stamp_motor = msg->header.stamp.toNSec()*1e-6;
	Motor.offset_yaw = msg->offset_yaw;
	if( (Motor.offset_yaw > 0.1 && Motor.offset_yaw < 0.5) || fabs(Motor.offset_yaw-180) <= 4 || (360-Motor.offset_yaw) < 1   )
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

	Sar.imuReady = true;
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
		//printf("raw csi1:%lf %lfi, csi2:%lf %lfi\n", csi1tmp.real(), csi1tmp.imag(), csi2tmp.real(), csi2tmp.imag() );
		Csi.csi.push_back(make_pair(csi1tmp, csi2tmp) );
	}
	Sar.csiReady = !msg->check_csi;
	//csi_ready = true;
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
	Sar.peakMat.setZero();
	Ap.apNum = 2;
	Ap.apID = 0;
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub1 = n.subscribe("imu", 1000, imuCallback);
	ros::Subscriber sub2 = n.subscribe("csi", 1000, csiCallback);
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

		mysystem("ping -q -n -i 0.05 192.168.0.3");
	}
	myfile1.open("power.txt");
	myfile2.open("peaks.txt");
	//myfile3.open("statistics.txt");
	ros::spinOnce();		//empty the queue
	Sar.csiReady = false;
	Sar.imuReady = false;
	vector<int> peakAngles;
	vector<int> prePeakAngles;
	while(n.ok() )
	{
		//spinner.spinOnce();
		ros::spinOnce();
		peakAngles.clear();
		peakAngles = SAR_Profile_2D();
		if(!peakAngles.empty() )
		{
			//print peaks after the elimination
			prePeakAngles = peakAngles;
			printf("%d peaks: ", (int) peakAngles.size() );
			for(int i = 0; i < (int) peakAngles.size(); ++i)
			{
				printf("%.1f ", peakAngles[i]/(GRANULARITY/360.0));
				if(peakAngles[i] >= test_target-detect_range4 && peakAngles[i] <= test_target+detect_range4)
				{
					++count_detected;
				}
			}
			printf("\n");
			if(peakAngles.size() == 1)
			{
				printf("\n");
				printf("!Get the converged result! Restart!\n");
				printf("\n");
			}
			if(peakAngles.size() == 1 || Sar.compTime == 0)
			{
				//Store this potential result
				angleSet.insert(make_pair(Ap.apID, peakAngles[0]) );
			}
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
						mysystem("ping -q -n -i 0.05 192.168.0.2");
						break;
					case 1:
						Ap.apID = (Ap.apID+1)%Ap.apNum;
						system("pkill -n ping");      //kill the child process first
						system("iwconfig wlan0 essid TP5G2");
						printf("Switch to TP5G2 and start ping\n");
						mysystem("ping -q -n -i 0.05 192.168.0.3");
						break;
				}
			}

		}
		else if(Sar.peakVanished)
		{
			Sar.peakVanished = false;
			printf("Peak vanished, restart peak elimination based on the last moment multipath profile.\n");
			//Put the last moment peaks to angleSet for localization
			for(unsigned int i = 0; i < prePeakAngles.size(); ++i)
			{
				Sar.angleSet.insert(make_pair(Ap.apID, prePeakAngles[i]) );
			}
			Sar.compTime = COMP_THRE;
			prePeakAngles.clear();
		}


	}

	myfile1.close();
	myfile2.close();

	if(Sar.autoSwitch)
	{
		system("pkill -n ping");
	}
	//print out the final anglular result
	if(!Sar.angleSet.empty() )
	{
		printf("AP_ID -> Angles: ");
		multimap<int, int>::iterator finalRetPos = Sar.angleSet.begin();
		while(finalRetPos != Sar.angleSet.end() )
		{
			printf("%d->%d; ", finalRetPos->first, finalRetPos->second);

			if(finalRetPos->second >= test_target-detect_range4 && finalRetPos->second <= test_target+detect_range4)
			{
				++count_finalOutput;
			}
			++finalRetPos;
		}
		printf("\n");
	}
	printf("%.2f correct peaks exist in the output\n", (float) count_finalOutput/(float)angleSet.size() );
	printf("%d(%d, %d, %d) target peaks %.1f exist in %d detections -> %.2f(%.2f, %.2f, %.2f)\n", count1, count2, count3, count4, test_target/(GRANULARITY/360.0), Sar.countD, (float)count1/(float)Sar.countD, (float)count2/(float)Sar.countD, (float)count3/(float)Sar.countD, (float)count4/(float)Sar.countD);

	//write statistics and print out
	/*
	map<int, int>::iterator stat_pos = statistics.begin();
	while(stat_pos != statistics.end() )
	{
		//myfile3 << stat_pos->first << " " << (float)stat_pos->second/(float)count_d << endl;
		//print out
		printf("Alpha:%d->%d/%d=%.2f\n", stat_pos->first, stat_pos->second, count_d, (float)stat_pos->second/(float)count_d );
		++stat_pos;
	}
	*/
	//myfile3.close();

	return 0;
}
