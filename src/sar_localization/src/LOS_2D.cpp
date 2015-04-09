#include "SAR.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
//#include "sar_localization/Imu.h"
#include <sar_localization/Csi.h>
#include <sar_localization/Motor.h>
#include <sensor_msgs/Imu.h>

#include <map>
#include <set>
#include <queue>
#include <assert.h>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
//#include <glib.h>

//for multiple processes processing
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>

using namespace std;

SAR sar;
AP ap;
MOTOR motor;
queue<sensor_msgs::Imu> imu_buf;
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
			printf("Count:%d,maxPow: %0.3f,sample size:%d, ", Sar.countD, maxPower, (int) Sar.input.size() );
			return ret_yaw;
		}
	}
	return -1;
}

void send_imu(const sensor_msgs::Imu &imu_msg)
{
	double t = imu_msg.header.stamp.toSec();
	//double dx = imu_msg.linear_acceleration.x;
	//double dy = imu_msg.linear_acceleration.y;
	//double dz = imu_msg.linear_acceleration.z;
	double rx = imu_msg.angular_velocity.x;
	double ry = imu_msg.angular_velocity.y;
	double rz = imu_msg.angular_velocity.z;

	sar.processIMU(t, Vector3d(rx, ry, rz) );
}

// %Tag(CALLBACK)%

void motorCallback(const sar_localization::Motor::ConstPtr& msg)
{
	motor.t_stamp = msg->header.stamp.toSec();
	motor.stdYaw = msg->std_yaw;
}

void imuCallback(const sensor_msgs::ImuConstPtr &imu_msg)
{
	Sar.imu_buf.push(*imu_msg);
}

void csiCallback(const sar_localization::Csi::ConstPtr& msg)
{
	sar.csi.pairVector.clear();
  	sar.csi.t_stamp = msg->header.stamp.toSec();
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
		sar.csi.pairVector.push_back(make_pair(csi1tmp, csi2tmp) );
	}
  	//Sar.csiReady = !msg->check_csi;
	while(!imu_buf.empty() && sar.csi.t_stamp >= imu_buf.front().header.stamp.toSec() )
	{
		sendIMU(imu_buf.front());
		imu_buf.pop();
		sar.dataReady = true;
	}
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
  	ros::NodeHandle n;
  	ros::Subscriber sub1 = n.subscribe("/imu_3dm_gx4/imu", 10000, imuCallback);
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
	//std_flag = false;
	while(n.ok() )
	{
		//spinner.spinOnce();
		ros::spinOnce();
		sar.inputData();
		sar.checkData();
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
