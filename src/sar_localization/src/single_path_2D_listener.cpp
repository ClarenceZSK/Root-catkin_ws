#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"
#include "sar_localization/Imu.h"
#include "sar_localization/Csi.h"

#include "/opt/eigen/Eigen/Dense"
#include <vector>
#include <map>
#include <set>
#include <complex>
#include <math.h>
#include <assert.h>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <glib.h>

//for multiple processes processing
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
//#include <tuple>
//Global variables for data processing
#define PI 3.1415926
#define sizeLimit 500
#define profileLimit 20

#define interval_threshold 20	//maximun interval must greater than 25 degree
#define circle_threshold 353	//maxAngle-minAngle > 353 degree
//for debug
#define stepSize 1

using namespace std;
using namespace Eigen;

//vector<complex<double> > CSI1;		//CSI from antenna 1
//vector<complex<double> > CSI2;		//CSI from antenna 2
complex<double>  CSI1[sizeLimit];
complex<double>  CSI2[sizeLimit];
double orientation[sizeLimit];

map<double, pair<complex<double>, complex<double> > > input;		

double t_stamp_csi;			//time stamp of csi
double t_stamp_imu;			//time stamp of imu
bool csi_ready = false;
bool imu_ready = false;
//double pitch;
double yaw;
complex<double> csi1;
complex<double> csi2;

Eigen::VectorXd multipathProfile(360);

double landa = 0.06;			//The aperture size is 6cm
double r = 0.06;			//The radius (antenna interval)

int dataIndex = 0;
int count_d = 0;
bool start = false;

int AP_ID = 0;		//The associated AP ID
int AP_NUM = 2;		//The number of available APs
pid_t childPID = -2;

//auto switch
bool autoSwith = 0;

//for data preprocessing to make sure the collected data really formed a circle
//double min_interval = 360;	//the minimum distance of two adjacent imu data
//double max_interval = 0;	//the maximun distance between two adjacent imu data
//double circle_distance = 0;


//for debug
int preIdx = 0;
double maxT_D = 0;
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
		//cout << "Relative csi:" << CSI1[i]*conj(CSI2[i]) << endl;
		avgCsiHat += input_csi1*conj(input_csi2)*tmp;
		++input_iter;
	}
	avgCsiHat /= input.size();
	double ret = avgCsiHat.real()*avgCsiHat.real() + avgCsiHat.imag()*avgCsiHat.imag();
	//printf("Power calculation: %lf\n", ret);
	return ret;
}
/*
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
*/

int findDirectPath()
{
	Eigen::VectorXd avgProfile = multipathProfile/(double)count_d;
	int idx;
	printf("Count:%d,preMax:%lf,",count_d,avgProfile(preIdx) );
	double maxV = avgProfile.maxCoeff(&idx);
	//printf("Mul-pro max:%lf, avg-pro max:%lf\n",multipathProfile(row,col), avgProfile(row,col) );
	preIdx = idx;
	printf("max pow:%lf\n", maxV);

	return idx;
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


int SAR_Profile_2D()
{
	if(csi_ready && imu_ready)
	{
		csi_ready = false;
		imu_ready = false;
		double timeDifference = fabs(t_stamp_csi-t_stamp_imu);
		if(maxT_D < timeDifference)
		{
			maxT_D = timeDifference;
		}
		
		//orientation[dataIndex % sizeLimit] = yaw;
		//CSI1[dataIndex % sizeLimit] = csi1;
		//CSI2[dataIndex % sizeLimit] = csi2;
		//++dataIndex;

		input[RadianToDegree(yaw)] =  make_pair(csi1, csi2);
		//if(dataIndex > 0 && dataIndex % sizeLimit == 0 && !start)
		if(input.size() == sizeLimit && 1)
		{
			//start data preprocessing
			//
			
			map<double, pair<complex<double>, complex<double> > >::iterator input_iter;
			input_iter = input.begin();
			double max_interval = 0;
			double min_interval = 0xffff;
			double maxAngle = 0;
			double minAngle = 0xffff;
			
			
			while(input_iter != input.end() )
			{
				double y1 = input_iter->first;
				++input_iter;
				if(input_iter == input.end() )
				{
					break;
				}
				double y2 = input_iter->first;
				++input_iter;
				double interval = y2 - y1;

				if(min_interval > interval)	//find min interval
				{
					min_interval = interval;	
				}

				if(max_interval < interval)		//find max interval
				{
					max_interval = interval;
				}
			}
			maxAngle = input.rbegin()->first;
			minAngle = input.begin()->first;
			
			double circle_distance = maxAngle - minAngle;
			
			if(max_interval < interval_threshold && circle_distance >= circle_threshold)
			{
				printf("Max interval:%.2f, min interval:%.2f, max angle:%.2f, min angle:%.2f, circle distance:%.2f\n", max_interval, min_interval, maxAngle, minAngle, circle_distance);
				start = true;
			}
			else if(input.size() > sizeLimit)	//it indicates some unpredictable situations causing very large input map
			{
				printf("Too many samples! Clear and Resampling!\n");
				input.clear();
			}
			
		
		}
		if(start)
		{
			printf("max T_D:%lf, ", maxT_D);
			start = false;
			maxT_D = 0;
			++count_d;
			int r_yaw;
			//When csi and imu data vectors reach size limit, start angle generation
			int resolution = stepSize;      //search resolution
			double power = 0;
	  
			myfile << "#" << count_d << endl;
			
			for(int alpha = 0; alpha < 360; alpha += resolution)
			{
				double sumpow = 0;
       			for (int step = 0; step < stepSize; ++step)
      			{
					double alpha_r = (alpha+step)*PI/180.0;
       				double powtmp = PowerCalculation(alpha_r);
					sumpow += powtmp;
					////myfile << powtmp << endl;
					//multipathProfile(alpha) += powtmp;
					//single path
					if(power < powtmp)
					{
						power= powtmp;
						r_yaw = alpha;
					}
				
      			}
				myfile << sumpow << endl;
			}
			//int directPath = findDirectPath();
			//++dataIndex;
      		//return directPath;
			printf("Count:%d,maxPow: %0.3f,sample size:%d, ",count_d, power,(int) input.size() );
			input.clear();
			return r_yaw;
		}	
		
	}

	return -1;
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
/*
void processing()
{
	pid_t cpid;	
	while(ros::ok())
	{
	int angle = SAR_Profile_2D();
	if(angle > 0)
        {
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
                         cpid = mysystem("ping -q -n -i 0.01 192.168.0.3");
                         break;
                 case 1:
                         AP_ID = (AP_ID+1)%AP_NUM;
                         system("pkill -INT -n ping");      //kill the child process first
                         system("iwconfig wlan0 essid TP5G1");
                         printf("Switch to TP5G1 and start ping\n");
                         cpid = mysystem("ping -q -n -i 0.01 192.168.0.2");
                         break;
                 }
        }

	}

}
*/

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "listener");
	multipathProfile.setZero();
  	ros::NodeHandle n;

  	ros::Subscriber sub1 = n.subscribe("imu", 10000, imuCallback);
	ros::Subscriber sub2 = n.subscribe("csi", 10000, csiCallback);

	//ros::MultiThreadedSpinner spinner(2);

	//system configuration
	//system("service network-manager stop");
	//printf("Network-manager stop\n");

	//system("modprobe -r iwlwifi mac80211");
	//printf("Remove wifi module completed\n");

	//system("modprobe iwlwifi connector_log=0x1");
	//printf("Load connector log module\n");
	
	//system("iwlist wlan0 scan");
	//printf("Scan completed\n");

	if(autoSwith)
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
	myfile.open("power.txt");
	ros::spinOnce();		//empty the queue
	csi_ready = false;
	imu_ready = false;
	while(n.ok() )
	{
		//spinner.spinOnce();
		ros::spinOnce();
		int angle = SAR_Profile_2D();	
		if(angle >= 0)
		{
			printf("Alpha:%d\n", angle);
            //Switch to another AP
			if(autoSwith)
			{
       	    	switch(AP_ID)
            	{
            	case 0:
					//Switch to from AP1 to AP2
              		AP_ID = (AP_ID+1)%AP_NUM;
                	system("pkill -INT -n ping");   //kill the child process first
               		system("iwconfig wlan0 essid TP5G2");
               		printf("Switch to TP5G2 and start ping\n");
              		mysystem("ping -q -n -i 0.05 192.168.0.3");
               		break;
            	case 1:
              		AP_ID = (AP_ID+1)%AP_NUM;
              		system("pkill -INT -n ping");      //kill the child process first
              		system("iwconfig wlan0 essid TP5G1");
               		printf("Switch to TP5G1 and start ping\n");
        	   		mysystem("ping -q -n -i 0.05 192.168.0.2");
           		break;
           		}
			}
		}
	}

	/*
	GThread* process_thread;
  	GError* err=NULL;

  	if ((process_thread = g_thread_new( "data_processing", (GThreadFunc)processing, NULL)) == NULL)
  	{
   		printf("Failed to create serial handling thread: %s!!\n", err->message);
    		g_error_free(err);
  	}
	// %Tag(SPIN)%
        ros::spin();
	// %EndTag(SPIN)%
	*/
	myfile.close();
	if(autoSwith)
	{
		system("pkill -INT -n ping");
	}
	return 0;
}	
