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

Eigen::VectorXd multipathProfile(360);

double landa = 0.06;					//The aperture size is 6cm
double r = 0.116;							//The radius (antenna interval)

int dataIndex = 0;
int count_d = 0;
bool start = false;

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

int SAR_Profile_2D()
{
	//Align csi and imu data
	if(csi_ready && imu_ready)
	{
		csi_ready = false;
		imu_ready = false;
		double timeDifference = fabs(t_stamp_csi-t_stamp_imu);
		if(maxT_D < timeDifference)
		{
			maxT_D = timeDifference;
		}
		//printf("T_D:%lf, ", timeDifference);
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
			printf("max T_D:%lf, ", maxT_D);
			start = false;
			maxT_D = 0;
			++count_d;
			int r_yaw;
			//When csi and imu data vectors reach size limit, start angle generati    on
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
			++dataIndex;
      //return directPath;
			printf("Count:%d,maxPow: %0.3f, ",count_d, power);
			return r_yaw;
		}
		
		++dataIndex;
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
	multipathProfile.setZero();
  ros::NodeHandle n;

  ros::Subscriber sub1 = n.subscribe("imu", 1000, imuCallback);
	ros::Subscriber sub2 = n.subscribe("csi", 1000, csiCallback);

	
	myfile.open("power.txt");
	
	// %Tag(SPIN)%
	while (n.ok())
  {
	  ros::spinOnce();
		//do something
		int angle = SAR_Profile_2D();
		if(angle > 0)
		{
			printf("Alpha:%d\n", angle);
		}

	}
	// %EndTag(SPIN)%
	myfile.close();
  return 0;
}
