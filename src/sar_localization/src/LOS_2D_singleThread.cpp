#include "SAR_S.h"
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
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>

//for multiple processes processing
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>

using namespace std;

SAR sar;
queue<sensor_msgs::Imu> imu_buf;
visualization_msgs::Marker marker;
ros::Publisher marker_pub;

double RadianToDegree(double radian)
{
	return radian/PI*180;
}
double DegreeToRadian(double degree)
{
	return degree/180.0*PI;
}


void sendIMU(const sensor_msgs::Imu &imu_msg)
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
	sar.motor.t_stamp = msg->header.stamp.toSec();
	sar.motor.stdYaw = msg->std_yaw;
}

void imuCallback(const sensor_msgs::ImuConstPtr &imu_msg)
{
	imu_buf.push(*imu_msg);
}

void csiCallback(const sar_localization::Csi::ConstPtr& msg)
{
	int nt = msg->Ntx;
	if(nt == 2)
		return;
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

	double imu_t = 0;
	while(!imu_buf.empty() && sar.csi.t_stamp >= imu_buf.front().header.stamp.toSec() )
	{
		sendIMU(imu_buf.front());
		imu_t = imu_buf.front().header.stamp.toSec();
		imu_buf.pop();
		sar.dataReady = true;
	}
	if(imu_t != 0)
	{
		double tD = sar.csi.t_stamp - imu_t;
		if(sar.maxTimeDiff < tD)
		{
			sar.maxTimeDiff = tD;
		}
	}
}
// %EndTag(CALLBACK)%

//init marker
void initMarker()
{
	marker.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time::now();
	marker.ns = "listener";
	marker.id = 0;
	uint32_t shape = visualization_msgs::Marker::ARROW;
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;

	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;
	marker.lifetime = ros::Duration();
}

void setMarkerOrientation(int alpha, int beta)
{
	//while (marker_pub.getNumSubscribers() < 1)
	//{
	//	if (!ros::ok() )
	//	{
	//		return;
	//	}
	//	ROS_WARN_ONCE("Please create a Subscriber to the marker");
	//}
	//specify orientation
	//marker.pose.position.x = cos(DegreeToRadian(alpha));
	//marker.pose.position.y = sin(DegreeToRadian(alpha));
	//printf("%lf %lf\n", marker.pose.position.x, marker.pose.position.y);
	marker.pose.orientation.x = cos(DegreeToRadian(alpha))*sin(DegreeToRadian(beta));
	marker.pose.orientation.y = sin(DegreeToRadian(alpha))*sin(DegreeToRadian(beta));
	marker.pose.orientation.z = cos(DegreeToRadian(beta));
	cout << "Mark orientation:" << marker.pose.orientation.x << ", " << marker.pose.orientation.y << ", " << marker.pose.orientation.z << endl;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub1 = n.subscribe("/imu_3dm_gx4/imu", 10000, imuCallback);
	ros::Subscriber sub2 = n.subscribe("csi", 10000, csiCallback);
	ros::Subscriber sub3 = n.subscribe("motor", 10000, motorCallback);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	ros::Publisher wifi_pub = n.advertise<sensor_msgs::PointCloud>("wifi_estimator", 1);
	if(sar.ap.autoSwitch)
	{
		sar.ap.init();
	}
	sar.myfile.open("power.txt");
	sar.init();
	ros::spinOnce();		//empty the queue
	//std_flag = false;
	while(n.ok() )
	{
		//spinner.spinOnce();
		ros::spinOnce();
		sar.inputData();
		bool start = false;
		if (sar.motor.nearStartPoint() )
		{
			start = sar.checkData();
		}
		if (start)
		{
			int angle = sar.SAR_Profile_2D();
			printf("Alpha:%d\n", angle);
			sar.initInput = true;
			//init marker
			initMarker();
			setMarkerOrientation(angle, 90);
			marker_pub.publish(marker);
			//Switch to another AP
			if(sar.ap.autoSwitch)
			{
				sar.switchAP();
			}
		}
	}

	sar.myfile.close();
	if(sar.ap.autoSwitch)
	{
		system("pkill -n ping");
	}
	return 0;
}
