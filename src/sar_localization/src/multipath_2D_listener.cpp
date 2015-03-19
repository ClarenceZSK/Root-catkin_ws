#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"
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
//Global variables for data processing
#define PI 3.1415926
#define sizeLimit 500
#define profileLimit 20

#define interval_threshold 6
#define circle_threshold 353
//for debug
#define stepSize 1

using namespace std;
using namespace Eigen;

//vector<complex<double> > CSI1;		//CSI from antenna 1
//vector<complex<double> > CSI2;		//CSI from antenna 2
//complex<double>  CSI1[sizeLimit];
//complex<double>  CSI2[sizeLimit];
//double orientation[sizeLimit];

map<double, pair<complex<double>, complex<double> > > input;

double t_stamp_csi;			//time stamp of csi
double t_stamp_imu;			//time stamp of imu
double t_stamp_motor;       //time stamp of motor
bool csi_ready = false;
bool imu_ready = false;
//double pitch;
double yaw;
complex<double> csi1;
complex<double> csi2;
//multipath effect processing
Eigen::VectorXi peak_mat(360);
int vib_threshold = 8;			//The peak vibration allowance, 0 means the persistent peak must be the degree exatly the same as before
int comp_time = 2;			//The times of comparison of multiple power profiles for peak elimination, it should be greater than 1

double landa = 0.06;			//The aperture size is 6cm
double r = 0.06;			//The radius (antenna interval)

int dataIndex = 0;
int count_d = 0;
bool start = false;
bool globalStart = false;

//yaw normalize
bool std_flag = true;
double std_yaw = -1;
double offset_yaw = 0;

int AP_ID = 0;		//The associated AP ID
int AP_NUM = 2;		//The number of available APs
pid_t childPID = -2;

//auto switch
bool autoSwitch = 0;

//for debug
int preIdx = 0;
double maxT_D = 0;
ofstream myfile1;		//power
ofstream myfile2;		//peaks

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
	map<double, pair<complex<double>, complex<double> > >::iterator input_iter;
    complex<double> avgCsiHat (0, 0);
    input_iter = input.begin();
    //for(int i = 0; i < sizeLimit; ++i)
    while(input_iter != input.end() )
    {
        double input_yaw = DegreeToRadian(input_iter->first);
        complex<double> input_csi1 = input_iter->second.first;
        complex<double> input_csi2 = input_iter->second.second;
        double theta = 2*PI/landa*r*cos(alpha-input_yaw);
        double real_tmp = cos(theta);
        double image_tmp = sin(theta);
        complex<double> tmp (real_tmp, image_tmp);
        avgCsiHat += input_csi1*conj(input_csi2)*tmp;
        ++input_iter;
    }
    avgCsiHat /= input.size();

	double ret = avgCsiHat.real()*avgCsiHat.real() + avgCsiHat.imag()*avgCsiHat.imag();
	//printf("Power calculation: %lf\n", ret);
	return ret;
}


Eigen::VectorXi peakElimination(Eigen::VectorXi cur_peak_mat)
{
	Eigen::VectorXi ret(360);
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

void mysystem(const char *cmdstr)
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
	//parent process
	//...
}

vector<int> countPeak()
{
	vector<int> ret;
	ret.clear();
	for (int i = 0; i < peak_mat.size(); ++i)
	{
		myfile2 << peak_mat(i) << endl;
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
	if(csi_ready && imu_ready && globalStart)
	{
		csi_ready = false;
		imu_ready = false;
		double timeDifference = fabs(t_stamp_csi-t_stamp_imu);
		if(maxT_D < timeDifference)
		{
			maxT_D = timeDifference;
		}
		double std_input_yaw = RadianToDegree(yaw - std_yaw);
        if (std_input_yaw < 0)
        {
            std_input_yaw += 360;
        }
		else if(std_input_yaw >= 360)
        {
            std_input_yaw -= 360;
        }
		int tp = std_input_yaw*10;
		std_input_yaw = tp/10.0;
        //printf("STD_INPUT_YAW:%.2f\n",std_input_yaw );
        input[std_input_yaw] =  make_pair(csi1, csi2);

		if(input.size() > 1 )
        {
            //start data preprocessing
            //

            map<double, pair<complex<double>, complex<double> > >::iterator input_iter;
            input_iter = input.begin();
            double max_interval = 0;
            double min_interval = 0xffff;
            double maxAngle = 0;
            double minAngle = 0xffff;

            double prey = input_iter->first;
            double cury = 0xffff;
            ++input_iter;
			while(input_iter != input.end() )
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
			maxAngle = input.rbegin()->first;
            minAngle = input.begin()->first;
            double circle_distance = maxAngle - minAngle;

            if(max_interval < interval_threshold && circle_distance >= circle_threshold)
            {
                printf("Max interval:%.2f, min interval:%.2f, max angle:%.2f, min angle:%.2f, circle distance:%.2f\n", max_interval, min_interval, maxAngle, minAngle, circle_distance);
                start = true;
            }
            else if(input.size() > sizeLimit)   //it indicates some unpredictable situations causing very large input map
            {
                printf("Too many samples! Clear and Resampling!\n");
				printf("Reason: \n");
                if(max_interval >= interval_threshold)
                {
                    printf("Too large interval! max_interval/threshold:%.2f/%d  \n", max_interval, interval_threshold);
                }
                if(circle_distance < circle_threshold)
                {
                    printf("Not a circle! circle_distance/threshold:%.2f/%d\n"  , circle_distance, circle_threshold);
                }

                input.clear();
            }

		}

		if(start)
		{
			Eigen::VectorXi cur_peak_mat(360);
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

			myfile1 << "#" << count_d << endl;	//for power
			myfile1 << prePow << endl;

			myfile2 << "#" << count_d << endl;	//for peaks

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

			++dataIndex;

			if(count_d == 1)
			{
				peak_mat = cur_peak_mat;
				ret = countPeak();
				printf("Vib:%d,Count:%d,peak_num: %d\n", vib_threshold, count_d, (int) ret.size());
				input.clear();
			}
			else	//start peak elimination
			{
				peak_mat = peakElimination(cur_peak_mat);
				ret = countPeak();
				printf("Vib:%d,Count:%d,peak_num: %d\n", vib_threshold, count_d, (int) ret.size());
				input.clear();
				return ret;
			}
		}
	}

	return ret;
}

// %Tag(CALLBACK)%
void motorCallback(const sar_localization::Motor::ConstPtr& msg)
{
    t_stamp_motor = msg->header.stamp.toNSec()*1e-6;
    offset_yaw = msg->offset_yaw;
    if(offset_yaw < 0.1 || fabs(offset_yaw-180) <= 4 || (360-offset_yaw) < 0.2   )
    {
		globalStart = true;
        std_flag = false;
    }
}


void imuCallback(const sar_localization::Imu::ConstPtr& msg)
{
  	t_stamp_imu = msg->header.stamp.toNSec()*1e-6;
  	yaw = msg->yaw;
	yaw = yaw*PI/180.0;
	//if(!std_flag && input.size() > 2)
    if(!std_flag)
    {
		std_yaw = yaw + DegreeToRadian(offset_yaw);
        if(std_yaw >= 2*PI)
        {
            std_yaw -= 2*PI;
        }
        std_flag = true;
        //printf("T_D:%.2f, std_yaw:%.2f, yaw:%.2f, offset yaw:%.2f\n", fabs(t_stamp_motor-t_stamp_imu), RadianToDegree(std_yaw), RadianToDegree(yaw), offset_yaw);
    }

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

  	ros::Subscriber sub1 = n.subscribe("imu", 1000, imuCallback);
	ros::Subscriber sub2 = n.subscribe("csi", 1000, csiCallback);
	ros::Subscriber sub3 = n.subscribe("motor", 10000, motorCallback);

	if(autoSwitch)
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

	ros::spinOnce();		//empty the queue
	csi_ready = false;
	imu_ready = false;
	while(n.ok() )
	{
		//spinner.spinOnce();
		ros::spinOnce();

		vector<int> peakAngles = SAR_Profile_2D();
		if(!peakAngles.empty() )
		{
			//print peaks after the elimination
			printf("%d peaks: ", (int) peakAngles.size() );
			for(int i = 0; i < (int) peakAngles.size(); ++i)
			{
				printf("%d ", peakAngles[i]);
			}
			printf("\n");
			//Switch to another AP
			if(autoSwitch)
			{
				switch(AP_ID)
				{
				case 0:
					//Switch to from AP1 to AP2
					AP_ID = (AP_ID+1)%AP_NUM;
					system("pkill -n ping");   //kill the child process first
					system("iwconfig wlan0 essid TP5G1");
					printf("Switch to TP5G1 and start ping\n");
					mysystem("ping -q -n -i 0.05 192.168.0.2");
					break;
				case 1:
					AP_ID = (AP_ID+1)%AP_NUM;
					system("pkill -n ping");      //kill the child process first
					system("iwconfig wlan0 essid TP5G2");
					printf("Switch to TP5G2 and start ping\n");
					mysystem("ping -q -n -i 0.05 192.168.0.3");
					break;
				}
			}
		}
	}

	myfile1.close();
	myfile2.close();
	if(autoSwitch)
	{
		system("pkill -n ping");
	}
	return 0;
}
