#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>


#include "wifi_ekf.h"

#define G_CNT 0

#include <queue>
using namespace std;
queue<sensor_msgs::Imu> imu_buf;
queue<sensor_msgs::PointCloud> wifi_buf;

ros::Publisher pub_path;
ros::Publisher pub_odometry;
ros::Publisher pub_pose;
nav_msgs::Path path;


int gravity_cnt = 0;
Eigen::Vector3d sum_g(0.0, 0.0, 0.0);
bool filter_start = false;

WifiEkf wifi_ekf;

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    Eigen::Vector3d acc(imu_msg->linear_acceleration.x,
                        imu_msg->linear_acceleration.y,
                        imu_msg->linear_acceleration.z);
    if (gravity_cnt < G_CNT)
    {
        sum_g += acc;
        gravity_cnt++;
        return;
    }
    else if (gravity_cnt == G_CNT)
    {
        sum_g += acc;
        gravity_cnt++;
		//for simulation
        wifi_ekf.init(imu_msg->header.stamp.toSec(), Eigen::Vector3d(0.0, 0.0, -9.8));
		//for experiments
        //wifi_ekf.init(imu_msg->header.stamp.toSec(), sum_g/gravity_cnt);
        filter_start = true;
        return;
    }
    imu_buf.push(*imu_msg);
}

void wifi_callback(const sensor_msgs::PointCloudConstPtr &wifi_msg)
{
    wifi_buf.push(*wifi_msg);
}

void process()
{
    if (!filter_start || imu_buf.empty() || wifi_buf.empty())
        return;
    if (wifi_buf.front().header.stamp.toSec() < imu_buf.front().header.stamp.toSec())
    {
        wifi_buf.pop();
        return;
    }
    if (wifi_buf.front().header.stamp.toSec() >= imu_buf.back().header.stamp.toSec())
    {
        return;
    }

    sensor_msgs::PointCloud wifi_msg = wifi_buf.front();
    wifi_buf.pop();

    ROS_DEBUG("IMU integration");
    double check_t = 0.0;
    while (imu_buf.front().header.stamp.toSec() <= wifi_msg.header.stamp.toSec())
    {
        check_t = imu_buf.front().header.stamp.toSec();
        wifi_ekf.predict(check_t,
                         Eigen::Vector3d(imu_buf.front().linear_acceleration.x,
                                         imu_buf.front().linear_acceleration.y,
                                         imu_buf.front().linear_acceleration.z),
                         Eigen::Vector3d(imu_buf.front().angular_velocity.x,
                                         imu_buf.front().angular_velocity.y,
                                         imu_buf.front().angular_velocity.z)
                        );
        imu_buf.pop();
    }

    ROS_DEBUG("Time shift: %f", std::fabs(check_t - wifi_msg.header.stamp.toSec()));
    wifi_ekf.update(wifi_msg.points[0].x);
    ROS_INFO_STREAM("wifi position: " << wifi_ekf.p.transpose());
    ROS_INFO_STREAM("wifi velocity: " << wifi_ekf.v.transpose());
    ROS_INFO_STREAM("ap: " << wifi_ekf.ap.transpose());
    ROS_INFO_STREAM("gravity: " << wifi_ekf.g.transpose() << " norm: " << wifi_ekf.g.norm());
    ROS_INFO_STREAM("cov: " << wifi_ekf.P.diagonal().transpose());

    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time::now();
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = wifi_ekf.p(0);
    odometry.pose.pose.position.y = wifi_ekf.p(1);
    odometry.pose.pose.position.z = wifi_ekf.p(2);
    odometry.pose.pose.orientation.x = Eigen::Quaterniond(wifi_ekf.q).x();
    odometry.pose.pose.orientation.y = Eigen::Quaterniond(wifi_ekf.q).y();
    odometry.pose.pose.orientation.z = Eigen::Quaterniond(wifi_ekf.q).z();
    odometry.pose.pose.orientation.w = Eigen::Quaterniond(wifi_ekf.q).w();
    odometry.twist.twist.linear.x = wifi_ekf.v(0);
    odometry.twist.twist.linear.y = wifi_ekf.v(1);
    odometry.twist.twist.linear.z = wifi_ekf.v(2);
    pub_odometry.publish(odometry);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose = odometry.pose.pose;
    path.poses.push_back(pose_stamped);
    pub_path.publish(path);
    pub_pose.publish(pose_stamped);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "wifi_ekf");
    ros::NodeHandle n("~");
    //ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    pub_path     = n.advertise<nav_msgs::Path>("path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_pose     = n.advertise<geometry_msgs::PoseStamped>("pose", 1000);

    path.header.frame_id = "world";

    ros::Subscriber sub_imu  = n.subscribe("/data_generator/imu", 1000, imu_callback);
    ros::Subscriber sub_wifi = n.subscribe("/data_generator/wifi", 1000, wifi_callback);

    ros::Rate r(1000);
    while (ros::ok())
    {
        process();
        ros::spinOnce();
        r.sleep();
    }
    //ros::spin();
}