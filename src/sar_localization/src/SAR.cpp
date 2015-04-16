#include "SAR.h"

using namespace std;
using namespace Eigen;
//SAR
SAR::SAR():Landa(0.05222), frame_count(0), round_count(0), input_count(0), current_time(-1), input(DATA_SIZE)
{
	baseDirection << 0, 1, 0;
	firstNearStart = true;
	initInput = false;
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
		//cout << "Frame:" << frame_count << " IMU:\n" << imuAngular[frame_count] << endl;
	}
}

void SAR::init()
{
	frame_count = 0;
    if(motor.stdYaw < 1e-5)
    {
        baseDirection << 0, 1, 0;
    }
    else
    {
        baseDirection << 0, -1, 0;
    }
	for(int i = 0; i < WINDOW_SIZE; ++i)
	{
		imuAngular[i].setIdentity();
	}
	inputQueue.clear();
}

double SAR::degreeToRadian(double degree)
{
	return degree/180.0*PI;
}

double SAR::radianToDegree(double radian)
{
	return radian/PI*180;
}

void SAR::inputData(SharedVector* shared_ptr)	//input accumulated data
{
	if(shared_ptr->empty() )
	{
		return;
	}
	if(initInput)
	{
		if(firstNearStart)
		{
			cout << "Reach start point, init input!" << endl;
			firstNearStart = false;
		}
		input[input_count%DATA_SIZE] = make_pair(baseDirection, csi);
		++input_count;
		initInput = false;
	}
	else
	{
		firstNearStart = true;
		/////////////////////
		int idx = 0;
		Vector3d base = baseDirection;
		g_mutex_lock(&mutex);
		assert(frame_count == shared_ptr->size() );
		//cout << "Shared data size:" << shared_ptr->size() << endl;
		Vector3d direction (0, 0, 0);
		//cout << "shared_ptr size:" << shared_ptr->size() << "-" << frame_count << endl;
		//cout << "start base direction:\n" << baseDirection << endl;
		while(idx != frame_count)
		{
			Matrix3d rotation = Matrix3d::Identity();
			for(int i = idx; i >= 0; --i)
			{
				rotation *= shared_ptr->at(idx).first;
			}
			direction = rotation * base;
			input[input_count%DATA_SIZE] = make_pair(direction, shared_ptr->at(idx).second);
			++input_count;
			++idx;
		}
		baseDirection = direction;
		//cout << "end base direction:\n" << baseDirection << endl;
		shared_ptr->clear();
		frame_count = 0;
		g_mutex_unlock(&mutex);
		//////////////////////
	}
}

bool SAR::checkData()
{
	if(input_count < DATA_SIZE)
	{
		return false;
	}
	//check if semi-circulate arc
	int size = (int) input.size();
	//double cosArc = input[--size].first.dot(baseDirection);
	//assert(cosArc <= 1 && cosArc >= -1);
	//if(cosArc > cos(degreeToRadian(ARC) ) )
	//{
	//	return false;
	//}

	//check max interval
	double maxInterval = 0;
	Vector3d pre = input[0].first;
	Vector3d intervalPre;
	Vector3d intervalNext;
	int nextid = 0;
	for(int i = 1; i < size; ++i)
	{
		Vector3d next = input[i].first;
		double cosTmp = pre.dot(next);
		if(maxInterval < cosTmp)
		{
			maxInterval = cosTmp;
			intervalPre = pre;
			intervalNext = next;
			nextid = i;
		}
		pre = next;
	}
	if(maxInterval < cos(degreeToRadian(INTERVAL) ) )
	{
		cout << "Too large interval! Pre:\n" << intervalPre << "preid:" << nextid-1 << " Next:\n" << intervalNext << "nextid:" << nextid << endl;
		//init();
		return false;
	}
	else
	{
		//check data consistancy
		int checkSubcarrierNum = (int) input[0].second.pairVector.size();
		for (int i = 1; i < size; ++i)
		{
			int subcarrierNum = (int) input[i].second.pairVector.size();
			if(subcarrierNum != checkSubcarrierNum)
			{
				cout << "!Data inconsistant!" << checkSubcarrierNum << "--" << subcarrierNum << endl;
				//init();
				return false;
			}
		}
		return true;
	}
}

double SAR::powerCalculation(Vector3d dr_std)
{
	double ret = 0;
	int div = (int) input[0].second.pairVector.size();
	//int div = 30;
	for(int i = 0; i < div; ++i)
	{
		complex<double> avgCsiHat(0, 0);
		for(int j = 0; j < (int) input.size(); ++j)
		{
			Vector3d directionV = input[j].first;
			complex<double> input_csi1 = input[j].second.pairVector[i].first;
			complex<double> input_csi2 = input[j].second.pairVector[i].second;
			double theta = (2*PI/Landa) * R * directionV.dot(dr_std);
			assert(directionV.norm() == 1);
			double real_tmp = cos(theta);
			double image_tmp = sin(theta);
			complex<double> tmp (real_tmp, image_tmp);
			avgCsiHat += input_csi1 * conj(input_csi2) * tmp;
		}
		avgCsiHat /= input.size();
		ret += avgCsiHat.real() * avgCsiHat.real() + avgCsiHat.imag() * avgCsiHat.imag();
	}
	ret /= div;
	return ret;
}

int SAR::SAR_Profile_2D()
{
	int ret_yaw = 0;
	int resolution = STEP_SIZE;      //search resolution
	double maxPower = 0;
	++round_count;
	myfile << "#" << round_count << endl;
	for(int alpha = 0; alpha < 360; alpha += resolution)
	{
		Vector3d dr (cos(degreeToRadian(alpha) ), sin(degreeToRadian(alpha) ), 0);
		double powtmp = powerCalculation(dr);
		if(maxPower < powtmp)
		{
			maxPower= powtmp;
			ret_yaw = alpha;
		}
		myfile << powtmp << endl;
	}
	printf("round:%d,maxPow:%0.3f,sample size:%d, ", round_count, maxPower, (int) input.size() );
	return ret_yaw;
}

vector<int> SAR::SAR_Profile_3D()
{
	vector<int> ret;
	int resolution = STEP_SIZE;      //search resolution
	double maxPower = 0;
	int ret_yaw, ret_pitch;
    ++round_count;
	myfile << "#" << round_count << endl;
	for(int alpha = 0; alpha < 360; alpha += resolution)
	{
		for(int beta = 0; beta <= 180; beta += resolution)
		{
			Vector3d dr (sin(degreeToRadian(beta) ) * cos(degreeToRadian(alpha) ), sin(degreeToRadian(beta) ) * sin(degreeToRadian(alpha) ), cos(degreeToRadian(beta) ) );
			double powtmp = powerCalculation(dr);
          	if(maxPower < powtmp)
          	{
              	maxPower= powtmp;
              	ret_yaw = alpha;
				ret_pitch = beta;
          	}
          	myfile << powtmp << endl;
		}
	}
	printf("round:%d,maxPow:%0.3f,sample size:%d, ", round_count, maxPower, (int) input.size() );
	ret.push_back(ret_yaw);
    ret.push_back(ret_pitch);
	return ret;
}

vector<int> SAR::SAR_Profile_3D_fast()
{
	vector<int> ret;
    int resolution = STEP_SIZE;      //search resolution
    double maxPower = 0;
    int ret_yaw, ret_pitch;
    ++round_count;
	myfile << "#" << round_count << endl;
    for(int alpha = 0; alpha < 360; alpha += resolution)
    {
		Vector3d dr (cos(degreeToRadian(alpha) ), sin(degreeToRadian(alpha) ), 1);
		double powtmp = powerCalculation(dr);
		if(maxPower < powtmp)
		{
			maxPower = powtmp;
			ret_yaw = alpha;
		}
		myfile << powtmp << endl;
	}
	maxPower = 0;
	for(int beta = 0; beta <= 180; beta += resolution)
	{
		Vector3d dr (sin(degreeToRadian(beta) ) * cos(degreeToRadian(ret_yaw) ), sin(degreeToRadian(beta) ) * sin(degreeToRadian(ret_yaw) ), cos(degreeToRadian(beta) ) );
		double powtmp = powerCalculation(dr);
		if(maxPower < powtmp)
		{
			maxPower = powtmp;
			ret_pitch = beta;
		}
		myfile << powtmp << endl;
	}
	printf("round:%d,maxPow:%0.3f,sample size:%d, ", round_count, maxPower, (int) input.size() );
	ret.push_back(ret_yaw);
    ret.push_back(ret_pitch);
    return ret;
}

void SAR::switchAP()
{
    switch(ap.apID)
    {
    case 0:
    //Switch to from AP1 to AP2
        ap.apID = (ap.apID+1)%ap.apNum;
        system("pkill -n ping");   //kill the child process first
        system("iwconfig wlan0 essid TP5G1");
        printf("Switch to TP5G1 and start ping\n");
        ap.mysystem("ping -q -n -i 0.02 192.168.0.2");
        Landa = 0.05222;		//channel 149, frequency 5745MHz
        break;
	case 1:
	    ap.apID = (ap.apID+1)%ap.apNum;
        system("pkill -n ping");      //kill the child process fir  st
        system("iwconfig wlan0 essid TP5G2");
        printf("Switch to TP5G2 and start ping\n");
        ap.mysystem("ping -q -n -i 0.02 192.168.0.3");
		Landa = 0.05186;		//channel 157, frequency 5785MHz
        break;
    }
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

void AP::init()
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
