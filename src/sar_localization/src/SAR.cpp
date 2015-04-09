#include "SAR.h"

using namespace Eigen;
//SAR
SAR::SAR():Landa(0.05222), frame_count(0), current_time(-1)
{
	dataReady = false;
	ROS_INFO("SAR init finished");
}

void SAR::processIMU(double t, Vector3d angular_velocity)
{
	if (current_time < 0)
        current_time = t;
    double dt = t - current_time;
    current_time = t;
    if (frame_count != WINDOW_SIZE)
    {
        Quaterniond q(imuAngular[frame_count]);
        Quaterniond dq(1,
                       angular_velocity(0) * dt / 2,
                       angular_velocity(1) * dt / 2,
                       angular_velocity(2) * dt / 2);
        dq.w() = 1 - dq.vec().transpose() * dq.vec();
        imuAngular[frame_count] = (q * dq).normalized();
	}
}

double SAR::degreeToRadian(double degree)
{
	return degree/180.0*PI;
}

double SAR::radianToDegree(double radian)
{
	return radian/PI*180;
}

double SAR::powerCalculation(Vector3d alpha)
{

}

void SAR::inputData()
{
	if(!dataReady)
	{
		return;
	}
	dataReady = false;
	if(motor.nearStartPoint() )
	{
		frame_count = 0;
		if(motor.stdYaw-0 < 1e-5)
		{
			baseDirection << 0, 1, 0;
		}
		else
		{
			baseDirection << 0, -1, 0;
		}
		input.clear();
		input.push_back(make_pair(baseDirection, csi) );
	}
	else
	{
		Matrix3d rotation = Matrix3d::Identity();
		for(int i = frame_count; i >= 0; --i)
		{
			rotation *= imuAngular[i];
		}
		Vector3d direction = rotation * baseDirection;
		input.push_back(make_pair(direction, csi) );
		++frame_count;
	}
}

bool SAR::checkData()
{
	
}

//AP
AP::AP():autoSwitch(0), apID(0), apNum(2)
{
	ROS_INFO("AP init finished!");
}

void AP::mysystem(const char *cmdstr)
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
}

//MOTOR
bool MOTOR::nearStartPoint()
{
	if(stdYaw - 0 <= 1e-5 || 180 - stdYaw <= 0.1)
	{
		return true;
	}
	else
	{
		return false;
	}
}
