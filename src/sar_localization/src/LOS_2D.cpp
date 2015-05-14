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
//#include <sar_localization/Motor.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>

//for multithread processing
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#include <glib.h>

using namespace std;

bool globalStart = false;
SAR sar;
queue<sensor_msgs::Imu> imu_buf;
ros::Publisher imu_pub;

double RadianToDegree(double radian)
{
	return radian/PI*180;
}

double DegreeToRadian(double degree)
{
	return degree/180.0*PI;
}

double sendIMU(const sensor_msgs::Imu &imu_msg)
{
	double t = imu_msg.header.stamp.toSec();
	//double dx = imu_msg.linear_acceleration.x;
	//double dy = imu_msg.linear_acceleration.y;
	//double dz = imu_msg.linear_acceleration.z;
	double rx = imu_msg.angular_velocity.x;
	double ry = imu_msg.angular_velocity.y;
	double rz = imu_msg.angular_velocity.z;
	sar.processIMU(t, Vector3d(rx, ry, rz) );
	return t;
}

// %Tag(CALLBACK)%
/*
void motorCallback(const sar_localization::Motor::ConstPtr& msg)
{
	sar.motor.t_stamp = msg->header.stamp.toSec();
	sar.motor.stdYaw = msg->std_yaw;
	if(sar.motor.nearStartPoint() )
	{
		g_mutex_lock(&sar.mutex);
		sar.init();
		g_mutex_unlock(&sar.mutex);
	}
}
*/

void imuCallback(const sensor_msgs::ImuConstPtr &imu_msg)
{
	imu_buf.push(*imu_msg);
	imu_pub.publish(*imu_msg);
}

void csiCallback(const sar_localization::Csi::ConstPtr& msg)
{
	if(msg->Ntx > 1)
	{
		return;
	}
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
	double t = 0;
	while(!imu_buf.empty() && sar.csi.t_stamp >= imu_buf.front().header.stamp.toSec() )
	{
		t = sendIMU(imu_buf.front());
		imu_buf.pop();
	}
	if(!globalStart)
	{
		globalStart = true;
		sar.input[sar.ap.apID][sar.input_count[sar.ap.apID]%DATA_SIZE] = make_pair(sar.baseDirection, sar.csi);
		++sar.input_count[sar.ap.apID];
		sar.init();
	}
	else
	{
		//maintain a queue
		//Gmutex
		g_mutex_lock(&sar.mutex);
		sar.inputQueue.push_back(make_pair(sar.imuAngular[sar.frame_count], sar.csi) );
		++sar.frame_count;	//for each csi, we set a frame
		g_mutex_unlock(&sar.mutex);
	}
}
// %EndTag(CALLBACK)%

void SAR_processing(void* data_ptr)
{
	SharedVector *shared_ptr = (SharedVector*)data_ptr;
	ros::NodeHandle n2;
	ros::Publisher wifi_pub = n2.advertise<sensor_msgs::PointCloud>("wifi_estimator/wifi", 1);
	queue<sensor_msgs::Imu> imu_buf;
	if(sar.ap.autoSwitch)
    {
        sar.ap.init();
    }
	sar.myfile.open("power.txt");
	int countWiFimsg = 0;
	sar.init();
	ofstream retFile;
	retFile.open("results.txt");
	//int countRep = 0;
	while(n2.ok())
	{
		/*
		if(sar.motor.nearStartPoint() && !sar.initStart)
      	{
          	cout << "\nStart!" << sar.motor.stdYaw << endl;
          	sar.init();
          	sar.initStart = true;
      	}
		*/
		if(!globalStart)
		{
			g_mutex_lock(&sar.mutex);
			shared_ptr->clear();
			g_mutex_unlock(&sar.mutex);
			continue;
		}
		sar.inputData(shared_ptr);
		bool start = false;
		start = sar.checkData();
		if (start)
		{
			bool goodData = sar.selectData();
			if(!goodData)
				continue;
			double angle = sar.SAR_Profile_2D();
			//if(sar.preAngle < 0)
			//{
			//	sar.preAngle = angle;
			//}
			//else if(abs(angle-sar.preAngle) > 20)
			//{
			//	sar.preAngle = angle;
			//}
			retFile << "Round:" << sar.round_count << "; max power:" << sar.maxPow << "; sample size:" << sar.selectedInput.size() << "; Alpha:" << angle << endl;
			printf("Newest IDX: %d, Sample size: %d, Alpha:--------%.1f\n", sar.newestIdx, (int) sar.selectedInput.size(), angle);
			//publish wifi msgs
			if(fabs(sar.preAngle - angle) > 0 || 1)
			{
				printf("Newest IDX: %d, Sample size: %d, Alpha:--------%.1f\n", sar.newestIdx, (int) sar.selectedInput.size(), angle);
				sar.point_msg.x = cos(DegreeToRadian(angle) );
				sar.point_msg.y = sin(DegreeToRadian(angle) );
				sar.point_msg.z = 1;
				sar.channel_msg.values.push_back(sar.ap.apID);
				sar.wifi_msg.header.stamp = ros::Time::now();
				sar.wifi_msg.channels.push_back(sar.channel_msg);
				sar.wifi_msg.points.push_back(sar.point_msg);
				countWiFimsg++;
				wifi_pub.publish(sar.wifi_msg);
				cout << "Publish " << countWiFimsg << " WiFi msg!" << endl;
				sar.channel_msg.values.clear();
				sar.wifi_msg.channels.clear();
				sar.wifi_msg.points.clear();
			}
			/////////////////////////////////////
			sar.preAngle = angle;
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
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	//forward IMU
	imu_pub = n.advertise<sensor_msgs::Imu>("wifi_estimator/wifi_imu", 1000);
	if(!sar.ap.autoSwitch)
		sleep(15);
	ros::Subscriber sub1 = n.subscribe("/imu_3dm_gx4/imu", 10000, imuCallback);
	ros::Subscriber sub2 = n.subscribe("csi", 10000, csiCallback);
	//ros::Subscriber sub3 = n.subscribe("motor", 10000, motorCallback);
	int countWiFimsg = 0;
	//multithread
	g_mutex_init(&sar.mutex);
	GThread* data_thread;
	GError* err=NULL;
	SharedVector *data_ptr = &sar.inputQueue;
	if ((data_thread = g_thread_new( "sar", (GThreadFunc) SAR_processing, (void *)data_ptr)) == NULL)
	{
		printf("Failed to create serial handling thread: %s!!\n", err->message);
		g_error_free(err);
	}
	cout << "start spin" <<endl;
	ros::spin();
	return 0;
}
