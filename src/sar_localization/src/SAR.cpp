#include "SAR.h"

using namespace std;
using namespace Eigen;
//SAR
SAR::SAR():Landa(0.05222), frame_count(0), round_count(0), current_time(-1)
{
	//initStart = false;
	baseDirection << 0, 1, 0;
	newestIdx = -1;
	maxPow = 0;
	for(int i = 0; i < AP_NUM; ++i)
	{
		input_count[i] = 0;
	}
	preAngle = -1;
	ROS_INFO("SAR init finished");
}

void SAR::processIMU(double t, Vector3d angular_velocity)
{
	if (current_time < 0)
        current_time = t;
    double dt = t - current_time;
    current_time = t;
	g_mutex_lock(&mutex);
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
	g_mutex_unlock(&mutex);
}

void SAR::init()
{
	g_mutex_lock(&mutex);
	for(int i = 0; i < WINDOW_SIZE; ++i)
	{
		imuAngular[i].setIdentity();
	}
	frame_count = 0;
	inputQueue.clear();
	g_mutex_unlock(&mutex);
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
	SharedVector Swap;
	int currentFrameCount = 0;
	///////////////////////////
	g_mutex_lock(&mutex);
	Swap.swap(*shared_ptr);
	currentFrameCount = frame_count;
	g_mutex_unlock(&mutex);
	init();
	///////////////////////////
	int idx = 0;
	Vector3d base = baseDirection;
	assert(currentFrameCount == Swap.size() );
	//cout << "Add " << currentFrameCount << " Data. Start index: " << input_count[ap.apID]%DATA_SIZE << endl;
	Vector3d direction (0, 0, 0);
	//cout << "start base direction:\n" << baseDirection << endl;
	while(idx != currentFrameCount)
	{
		Matrix3d rotation = Matrix3d::Identity();
		for(int i = 0; i <= idx; ++i)
		{
			rotation *= Swap[i].first;
			//cout << "Swap[" << idx << "].first:\n" << Swap[idx].first << endl;
		}
		//cout << "Rotation:\n" << rotation << endl;
		direction = rotation * base;
		//cout << "Direction:\n" << direction << endl;
		input[ap.apID][input_count[ap.apID]%DATA_SIZE] = make_pair(direction, Swap[idx].second);
		//cout << "Direction:\n" << direction << endl;
		++input_count[ap.apID];
		++idx;
	}
	newestIdx = (input_count[ap.apID] - 1)%DATA_SIZE;
	baseDirection = direction;
	//cout << "end base direction:\n" << baseDirection << endl;
}

bool SAR::selectData()			//select a circular data to calculate
{
	//cout << "Select data" << endl;
	if(!selectedInput.empty() )
		selectedInput.clear();
	assert(newestIdx >= 0);
	//cout << newestIdx << endl;
	Vector3d firstV = input[ap.apID][newestIdx].first;
	selectedInput.push_back(input[ap.apID][newestIdx]);
	int searchIdx = newestIdx - 1;
	int count = 0;
	if(searchIdx < 0)
		searchIdx += DATA_SIZE;
	while(searchIdx != newestIdx)
	{
		count++;
		Vector3d v = input[ap.apID][searchIdx].first;
		selectedInput.push_back(input[ap.apID][searchIdx]);
		if(fabs(v.dot(firstV) - cos(0) ) <= 0.01 && count > min(50, DATA_SIZE/4))
		{
			//cout << "v:\n" << v << "\nfirstV:\n" << firstV << endl;
			break;
		}
		searchIdx--;
		if(searchIdx < 0)
		{
			searchIdx += DATA_SIZE;
		}
	}
	if(searchIdx == newestIdx)
	{
		//cout << "\n!!!Did not find a semi-circular vector!!!\n" << endl;
		selectedInput.clear();
		return false;
	}
	else
	{
		//cout << "Newest index: " << newestIdx << endl;
		int i = newestIdx;
		do
		{
			//cout << "input[" << i << "]:\n" << input[ap.apID][i].first << endl;
			--i;
			if(i < 0)
				i += DATA_SIZE;
		}while(i != newestIdx);
		return true;
	}
}

bool SAR::checkData()
{
	if(input_count[ap.apID] < DATA_SIZE)
	{
		return false;
	}
	//cout << "Check data" << endl;
	int size = DATA_SIZE;

	//check max interval
	double maxInterval = 0;
	Vector3d pre = input[ap.apID][0].first;
	Vector3d intervalPre;
	Vector3d intervalNext;
	int nextid = 0;
	for(int i = 1; i < size; ++i)
	{
		Vector3d next = input[ap.apID][i].first;
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
		int checkSubcarrierNum = (int) input[ap.apID][0].second.pairVector.size();
		for (int i = 1; i < size; ++i)
		{
			int subcarrierNum = (int) input[ap.apID][i].second.pairVector.size();
			if(subcarrierNum != checkSubcarrierNum)
			{
				cout << "!Data inconsistant!" << checkSubcarrierNum << "--" << subcarrierNum << endl;
				return false;
			}
		}
		return true;
	}
}

double SAR::powerCalculation(Vector3d dr_std)
{
	double ret = 0;
	//int div = (int) input[ap.apID][0].second.pairVector.size();
	int div = (int) selectedInput[0].second.pairVector.size();
	//int div = 30;
	for(int i = 0; i < div; ++i)
	{
		complex<double> avgCsiHat(0, 0);
		for(int j = 0; j < (int) selectedInput.size(); ++j)
		{
			Vector3d directionV = selectedInput[j].first;
			complex<double> input_csi1 = selectedInput[j].second.pairVector[i].first;
			complex<double> input_csi2 = selectedInput[j].second.pairVector[i].second;
			double theta = (2*PI/Landa) * R * directionV.dot(dr_std);
			assert(directionV.norm() == 1);
			double real_tmp = cos(theta);
			double image_tmp = sin(theta);
			complex<double> tmp (real_tmp, image_tmp);
			avgCsiHat += input_csi1 * conj(input_csi2) * tmp;
			//if(i == 0 && dr_std.x() == 1)
			//	cout << avgCsiHat << ", ";
		}
		//if(i == 0 && dr_std.x() == 1)
		//	cout << endl;
		avgCsiHat /= selectedInput.size();
		ret += avgCsiHat.real() * avgCsiHat.real() + avgCsiHat.imag() * avgCsiHat.imag();
	}
	ret /= div;
	return ret;
}

double SAR::finerResolution(int c_yaw)
{
	double ret = 0;
	double maxPower = 0;
	double resolution = min(0.1, 1.0/RSL);
	for(double alpha = c_yaw-1; alpha < c_yaw+1; alpha += resolution)
	{
		Vector3d dr (cos(degreeToRadian(alpha) ), sin(degreeToRadian(alpha) ), 0);
		double powtmp = powerCalculation(dr);
		if(maxPower < powtmp)
		{
			maxPower= powtmp;
			ret = alpha;
		}
	}
	return ret;
}

double SAR::SAR_Profile_2D()
{
	int ret_yaw = 0;
	int resolution = STEP_SIZE;      //search resolution
	double maxPower = 0;
	++round_count;
	myfile << "#" << round_count << endl;
	for(int alpha = 0; alpha < 360; alpha += resolution)
	{
		Vector3d dr (cos(degreeToRadian((double)alpha) ), sin(degreeToRadian((double)alpha) ), 0);
		double powtmp = powerCalculation(dr);
		if(maxPower < powtmp)
		{
			maxPower= powtmp;
			ret_yaw = alpha;
		}
		myfile << powtmp << endl;
	}
	printf("round:%d,maxPow:%0.3f,", round_count, maxPower);
	maxPow = maxPower;
	//debug
	//if(ret_yaw >= 180)
	//	ret_yaw -= 180;
	//finer resolution searching
	double finer_yaw = finerResolution(ret_yaw);
	return finer_yaw;
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
	printf("round:%d,maxPow:%0.3f,", round_count, maxPower);
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
	printf("round:%d,maxPow:%0.3f,", round_count, maxPower);
	ret.push_back(ret_yaw);
    ret.push_back(ret_pitch);
    return ret;
}

void SAR::switchAP()
{
    switch(ap.apID)
    {
    case 0:
    	//Switch to from AP1 to AP3
        ap.apID = (ap.apID+1)%ap.apNum;
        system("pkill -n ping");   //kill the child process first
        system("iwconfig wlan1 essid TP5G4");
        printf("Switch to TP5G4 and start ping\n");
        ap.mysystem("ping -q -n -i 0.01 192.168.0.5");
		Landa = 0.05186;		//channel 157, 5785MHz
        break;
	case 1:
	    ap.apID = (ap.apID+1)%ap.apNum;
        system("pkill -n ping");
        system("iwconfig wlan1 essid TP5G5");
        printf("Switch to TP5G5 and start ping\n");
        ap.mysystem("ping -q -n -i 0.01 192.168.0.6");
		Landa = 0.0515;		//channel 165, frequency 5825MHz
        break;
	case 2:
	    ap.apID = (ap.apID+1)%ap.apNum;
        system("pkill -n ping");
        system("iwconfig wlan1 essid TP5G3");
        printf("Switch to TP5G3 and start ping\n");
        ap.mysystem("ping -q -n -i 0.01 192.168.0.4");
		Landa = 0.05222;		//channel 149, frequency 5745MHz
   		break;
    }
}


//AP
AP::AP():autoSwitch(0), apID(0), apNum(AP_NUM)
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
    system("iwconfig wlan1 essid TP5G5");
    printf("iwconfig to TP5G5\n");
    system("dhclient wlan1");
    printf("dhclient from TP5G5 completed\n");

    system("iwconfig wlan1 essid TP5G4");
    printf("iwconfig to TP5G4\n");
    system("dhclient wlan1");
    printf("dhclient from TP5G4 completed\n");

    system("iwconfig wlan1 essid TP5G3");
    printf("iwconfig to TP5G3\n");
    system("dhclient wlan1");
    printf("dhclient from TP5G3 completed\n");

    mysystem("ping -q -n -i 0.01 192.168.0.4");
}

//MOTOR
/*
bool MOTOR::nearStartPoint()
{
	if(stdYaw == 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}
*/
