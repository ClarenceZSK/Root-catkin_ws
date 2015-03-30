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
//Global variables for data processing
#define PI 3.1415926
#define sizeLimit 500
#define profileLimit 20

#define interval_threshold 13
#define circle_threshold 353
#define comp_threshold 3
#define granularity 360
//for debug
#define stepSize 1

using namespace std;
using namespace Eigen;

//vector<complex<double> > CSI1;		//CSI from antenna 1
//vector<complex<double> > CSI2;		//CSI from antenna 2
//complex<double>  CSI1[sizeLimit];
//complex<double>  CSI2[sizeLimit];
//double orientation[sizeLimit];

map<double, vector<pair<complex<double>, complex<double> > > > input; //map<imu, csi1, csi2>

double t_stamp_csi;			//time stamp of csi
double t_stamp_imu;			//time stamp of imu
double t_stamp_motor;       //time stamp of motor
bool csi_ready = false;
bool imu_ready = false;
//double pitch;
double yaw;
vector<pair<complex<double>, complex<double> > > csi;
//multipath effect processing
Eigen::VectorXi peak_mat(granularity);
int vib_threshold = 7*granularity/360;			//The peak vibration allowance, 0 means the persistent peak must be the degree exatly the same as before
int comp_time = comp_threshold;			//The times of comparison of multiple power profiles for peak elimination, it should be greater than 1
int processingSize = 60;

double landa = 0.06;			//The aperture size is 6cm
double r = 0.10;			//The radius (antenna interval)
//Eigen::VectorXi std_profile(360);	//Store a standard multipath profile to recover from 0 result
bool peakVanished = false;	//It indicates that after angle elimination, there is no angle left. In this case, we need to recover for continuing experiment
bool reset = 0;

int count_d = 0;
bool start = false;
bool globalStart = false;

//yaw normalize
bool std_flag = true;
double std_yaw = -1;
double offset_yaw = 0;

//auto switch
int AP_ID = 0;		//The associated AP ID
int AP_NUM = 2;		//The number of available APs
pid_t childPID = -2;
bool autoSwitch = 0;

//AoA localization
multimap<int, int> angleSet;	//The set of angles for localization: <AP_ID, angles>. Probably, more than 1 angles will be selected


//for debug
int preIdx = 0;
double maxT_D = 0;
ofstream myfile1;		//power
ofstream myfile2;		//peaks
//ofstream myfile3;		//statistics
int test_target = 220*granularity/360;	//test peak near XX degree
vector<int> targetDistance;
int detect_range1 = vib_threshold;
int detect_range2 = detect_range1+1;
int detect_range3 = detect_range2+1;
int detect_range4 = detect_range3+1;
int count1 = 0;
int count2 = 0;
int count3 = 0;
int count4 = 0;
int count_detected = 0;
int count_finalOutput = 0;
map<int, int> statistics;	//first: alpha, second: count

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
/*
void getStatistics(Eigen::VectorXi cur_peaks)
{
	int range = vib_threshold;
	for(int i = 0; i < cur_peaks.size(); ++i)
	{
		if(cur_peaks(i) == 1)
		{
			int idx1 = i-range;
			int idx2 = i+range;
			for(int j = idx1; j < idx2; ++j)
			{
				if(j < 0)
				{
					statistics[j+360]++;
				}
				else if(j >= 360)
				{
					statistics[j-360]++;
				}
				else
				{
					statistics[j]++;
				}
			}
		}
	}
}
*/

double PowerCalculation(double alpha)
{
	double ret = 0;
	map<double, vector<pair<complex<double>, complex<double> > > >::iterator input_iter = input.begin();
	//printf("Input size:%d\n", input.size() );
	//int div = (int) input_iter->second.size();
	int div = processingSize;
	for(int i  = 0; i < div; ++i)
	{
		complex<double> avgCsiHat (0, 0);
		input_iter = input.begin();
		while(input_iter != input.end() )
		{
			double input_yaw = DegreeToRadian(input_iter->first);
			double realtmp1 = input_iter->second[i].first.real();
			double imagtmp1 = input_iter->second[i].first.imag();
			double realtmp2 = input_iter->second[i].second.real();
			double imagtmp2 = input_iter->second[i].second.imag();
			complex<double> input_csi1(realtmp1, imagtmp1);
			complex<double> input_csi2(realtmp2, imagtmp2);
			//cout << "input csi1:"<< input_csi1 <<" csi2:" << input_csi2 << endl;
			double theta = 2*PI/landa*r*cos(alpha-input_yaw);
			double real_tmp = cos(theta);
			double image_tmp = sin(theta);
			complex<double> tmp (real_tmp, image_tmp);

			avgCsiHat += input_csi1*conj(input_csi2)*tmp;
			++input_iter;
		}
		avgCsiHat /= input.size();
		//printf("real: %lf, imag: %lf, ret: %lf\n", avgCsiHat.real(),avgCsiHat.imag(),ret);
		ret += avgCsiHat.real()*avgCsiHat.real() + avgCsiHat.imag()*avgCsiHat.imag();
		//printf("real: %lf, imag: %lf, ret: %lf\n", avgCsiHat.real(),avgCsiHat.imag(),ret);
	}
	//printf("Pwr: %lf, csi size: %d\n", ret, csi.size() );
	ret /= div;
	//printf("Pwr: %lf, csi size: %d\n", ret, div);
	return ret;
}


Eigen::VectorXi peakElimination(Eigen::VectorXi cur_peak_mat)
{
	Eigen::VectorXi ret(granularity);
	for(int i  = 0; i < cur_peak_mat.size(); ++i)
	{
		if(cur_peak_mat(i) == true)
		{
			//start match the previous peak matrix in a group way
			bool peakOr = false;
			for(int j = i-vib_threshold; j <= i+vib_threshold; ++j)
			{
				int dx = (j+granularity)%granularity;
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
		//printf("csi size:%d\n",(int) csi.size() );
		input[std_input_yaw] = csi;

		if(input.size() > 1 )
		{
			//start data preprocessing
			//

			map<double, vector<pair<complex<double>, complex<double> > > >::iterator input_iter;
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
				printf("Max interval:%.2f, min interval:%.2f, max angle:%.2f, min angle:%.2f, sample size:%d, circle distance:%.2f\n", max_interval, min_interval, maxAngle, minAngle, (int)input.size(), circle_distance);
				start = true;
				//check data consistancy
				map<double, vector<pair<complex<double>, complex<double> > > >::iterator input_iter = input.begin();
				int check_size = (int) input_iter->second.size();
				processingSize = check_size;
				do
				{
					++input_iter;
					int s2 = (int) input_iter->second.size();
					if(check_size != s2)
					{
						//printf("Inconsistent data!%d-%d! Process only 30 groups data.\n", check_size, s2);
						printf("\n!!!!Inconsistent data!%d-%d! Resampling!\n", check_size, s2);
						//processingSize = 30;
						input.clear();
						start = false;
						break;
					}
				}
				while(input_iter != --input.end() );
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
				reset = 1;
				comp_time = comp_threshold;
				input.clear();
			}

			//limit compare times
			if(comp_time == 0)
			{
				comp_time = comp_threshold;
				printf("\nComp %d times, clear and restart!\n\n", comp_threshold);
				start = false;
				input.clear();
				reset = 1;
			}
		}

		if(start)
		{
			//for debug, print input angles
			bool test = 0;
			if(test)
			{
				map<double, vector<pair<complex<double>, complex<double> > > >::iterator input_iter = input.begin();
				cout << "input angles: ";
				while(input_iter != input.end() )
				{
					cout << input_iter->first << ", ";
					++input_iter;
				}
				cout << endl;
			}
			comp_time--;
			Eigen::VectorXi cur_peak_mat(granularity);
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

			for(int alpha = 1; alpha < granularity; alpha += resolution)
			{
				double alpha_r = alpha/(granularity/360.0)*PI/180.0;
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

			if(count_d == 1 || reset)
			{
				reset = 0;
				peak_mat = cur_peak_mat;
				ret = countPeak();
				printf("Vib:%d,Count:%d,peak_num: %d\n", vib_threshold, count_d, (int) ret.size());
				comp_time = comp_threshold;
				input.clear();
			}
			else	//start peak elimination
			{
				peak_mat = peakElimination(cur_peak_mat);
				ret = countPeak();
				printf("Vib:%d,Count:%d,peak_num: %d\n", vib_threshold, count_d, (int) ret.size());
				input.clear();
				//some additional processing includes:
				//1. peak re-elimination from the last moment multipath profile
				//2. extract the potential correct result if the ret size is 1
				if(ret.empty() )
				{
					peakVanished = true;
					//peak_mat.setZero();	//store the last moment peak situation
					reset = 1;
					comp_time = comp_threshold;
				}
				else if(ret.size() == 1)		//It indicates that we have obtained the final converged result. At this time, we restart the process based on the current peak situation
				{
					reset = 1;
					comp_time = comp_threshold;
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
	t_stamp_motor = msg->header.stamp.toNSec()*1e-6;
	offset_yaw = msg->offset_yaw;
	if( (offset_yaw > 0.1 && offset_yaw < 0.5) || fabs(offset_yaw-180) <= 4 || (360-offset_yaw) < 1   )
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
	csi.clear();
	t_stamp_csi = msg->header.stamp.toNSec()*1e-6;
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
		csi.push_back(make_pair(csi1tmp, csi2tmp) );
	}
	csi_ready = !msg->check_csi;
	//csi_ready = true;
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
	//myfile3.open("statistics.txt");

	ros::spinOnce();		//empty the queue
	csi_ready = false;
	imu_ready = false;
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
				printf("%.1f ", peakAngles[i]/(granularity/360.0));
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

			if(peakAngles.size() == 1 || comp_time == 0)
			{
				//Store this potential result
				angleSet.insert(make_pair(AP_ID, peakAngles[0]) );
			}

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
		else if(peakVanished)
		{
			peakVanished = false;
			printf("Peak vanished, restart peak elimination based on the last moment multipath profile.\n");
			//Put the last moment peaks to angleSet for localization
			for(unsigned int i = 0; i < prePeakAngles.size(); ++i)
			{
				angleSet.insert(make_pair(AP_ID, prePeakAngles[i]) );
			}
			comp_time = comp_threshold;
			prePeakAngles.clear();
		}


	}

	myfile1.close();
	myfile2.close();

	if(autoSwitch)
	{
		system("pkill -n ping");
	}
	//print out the final anglular result
	if(!angleSet.empty() )
	{
		printf("AP_ID -> Angles: ");
		multimap<int, int>::iterator finalRetPos = angleSet.begin();
		while(finalRetPos != angleSet.end() )
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
	printf("%d(%d, %d, %d) target peaks %.1f exist in %d detections -> %.2f(%.2f, %.2f, %.2f)\n", count1, count2, count3, count4, test_target/(granularity/360.0), count_d, (float)count1/(float)count_d, (float)count2/(float)count_d, (float)count3/(float)count_d, (float)count4/(float)count_d);

	//write statistics and print out
	map<int, int>::iterator stat_pos = statistics.begin();
	while(stat_pos != statistics.end() )
	{
		//myfile3 << stat_pos->first << " " << (float)stat_pos->second/(float)count_d << endl;
		//print out
		printf("Alpha:%d->%d/%d=%.2f\n", stat_pos->first, stat_pos->second, count_d, (float)stat_pos->second/(float)count_d );
		++stat_pos;
	}
	//myfile3.close();

	return 0;
}
