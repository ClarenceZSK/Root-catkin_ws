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
#include <visualization_msgs/Marker.h>

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
	//cout << "imu push" << endl;
	imu_buf.push(*imu_msg);
}

void csiCallback(const sar_localization::Csi::ConstPtr& msg)
{
	int nt = msg->Ntx;
	//cout << "nt=" << nt << endl;
	if(nt > 1)
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
		//cout << "data ready" << endl;
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

void WiFiMsgPublish(void* data_ptr)
{
	cout << "Enter wifi pub thread." << endl;
	sensor_msgs::PointCloud *shared_ptr = (sensor_msgs::PointCloud*)data_ptr;
	ros::NodeHandle n2;
	ros::Publisher wifi_pub = n2.advertise<sensor_msgs::PointCloud>("wifi_estimator/wifi", 1);
	ros::Rate r(10);
	sensor_msgs::PointCloud wifiMsg;
	int alpha_p[sar.ap.apNum];
	bool start = false;
	while(n2.ok())
	{
		///////////////////////////
		g_mutex_lock(&sar.mutex);
		//if((int)shared_ptr->points.size() == sar.ap.apNum)
		wifiMsg = *shared_ptr;
		start = true;
		shared_ptr->channels.clear();
		shared_ptr->points.clear();
		for(int i  = 0; i < sar.ap.apNum; ++i)
		{
			alpha_p[i] = sar.alpha[i];
		}
		g_mutex_unlock(&sar.mutex);
		///////////////////////////
		if(start)
		{
			wifiMsg.header.stamp = ros::Time::now();
			wifi_pub.publish(wifiMsg);
			///////////////////////////
			g_mutex_lock(&sar.mutex);
			cout << "(";
			for(int i = 0; i < sar.ap.apNum-1; ++i)
			{
				//cout << "(" << wifiMsg.points[i].x << ", " << wifiMsg.points[i].y << ", " << wifiMsg.points[i].z << "), ";
				cout << alpha_p[i] << ", ";
			}
			cout << alpha_p[sar.ap.apNum-1] << ")" << endl;
			g_mutex_unlock(&sar.mutex);
			////////////////////////////
		}
		r.sleep();
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener_singleThread");
	ros::NodeHandle n;
	ros::Subscriber sub1 = n.subscribe("/imu_3dm_gx4/imu", 10000, imuCallback);
	ros::Subscriber sub2 = n.subscribe("csi", 10000, csiCallback);
	ros::Subscriber sub3 = n.subscribe("motor", 10000, motorCallback);
	if(sar.ap.autoSwitch)
	{
		sar.ap.init();
	}
	/////////////////////////////////////
	//Create a thread to publish WiFi msgs
	g_mutex_init(&sar.mutex);
	GThread* wifiPub_thread;
	GError* err = NULL;
	sensor_msgs::PointCloud *data_ptr = &sar.wifi_msg;
	if ((wifiPub_thread = g_thread_new( "sar", (GThreadFunc) WiFiMsgPublish, (void *)data_ptr)) == NULL)
	{
		printf("Failed to create serial handling thread: %s!!\n", err->message);
		g_error_free(err);
	}
	//////////////////////////////////////
	sar.myfile.open("power.txt");
	sar.init();
	ros::spinOnce();		//empty the queue
	ofstream retFile;
	retFile.open("results.txt");
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
			if(sar.preAlpha[sar.ap.apID] >= 0)
			{
				int angleDif = abs(angle - sar.preAlpha[sar.ap.apID]);
				if(abs(angleDif - 180) < 10)
				{
					angle = sar.mirror(angle);
					sar.preAlpha[sar.ap.apID] = angle;
				}
				else
				{
					sar.preAlpha[sar.ap.apID] = angle;
				}
			}
			else
			{
				sar.preAlpha[sar.ap.apID] = angle;
			}
			retFile << "Round:" << sar.round_count << "; max power:" << sar.maxPow << "; sample size:" << sar.input.size() << "; AP->Alpha:" << sar.ap.apID << "->" << angle << endl;
			sar.alpha[sar.ap.apID] = angle;
			sar.point_msg.x = cos(DegreeToRadian(angle) );
			sar.point_msg.y = sin(DegreeToRadian(angle) );
			sar.point_msg.z = 0;
			sar.channel_msg.values.push_back(sar.ap.apID);
			sar.wifi_msg.header.stamp = ros::Time::now();
			sar.wifi_msg.points.push_back(sar.point_msg);
			sar.wifi_msg.channels.push_back(sar.channel_msg);
			sar.channel_msg.values.clear();
			printf("AP: %d, Alpha:%d\n", sar.ap.apID, angle);
			sar.initInput = true;
			//init marker
			//initMarker();
			//setMarkerOrientation(angle, 90);
			//marker_pub.publish(marker);
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
