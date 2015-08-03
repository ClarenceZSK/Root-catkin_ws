/*
 * (c) 2008-2011 Daniel Halperin <dhalperi@cs.washington.edu>
 * (c) 2015 Shengkai Zhang <shengkai.zhang@gmail.com> modified
 */
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "sar_localization/Csi.h"
#include <ros/console.h>
//#include <sstream>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <linux/socket.h>
#include <linux/netlink.h>
#include <linux/connector.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <stdint.h>

#include "/opt/eigen/Eigen/Dense"
#include <complex>
#include <vector>
#include <math.h>
#include <fstream>
#include <map>
#include <glib.h>
#include <unistd.h>

//add FFT and IFFT to mitigate multipath fading effect
#include <valarray>
#include <fftw3.h>
typedef std::complex<double> Complex;
typedef std::valarray<Complex> CArray;
//////////////////////////////////////////////////////

#define CN_NETLINK_USERS		11	/* Highest index + 1 */
#define CN_IDX_IWLAGN   (CN_NETLINK_USERS + 0xf)
#define CN_VAL_IWLAGN   0x1

#define MAX_PAYLOAD 2048
#define SLOW_MSG_CNT 1

sig_atomic_t volatile g_request_shutdown = 0;
int sock_fd = -1;							// the socket
//FILE* out = NULL;

//void check_usage(int argc, char** argv);

//FILE* open_file(char* filename, char* spec);

void caught_signal(int sig);

void exit_program(int code);
void exit_program_err(int code, char* func);

using namespace std;
using namespace Eigen;

ros::Publisher          csi_pub;

void setWindow(fftw_complex *x)
{
	bool up = false;
	double peakValue = 0;
	int peakIndex = 0;
	for(int k = 0; k < 29; ++k)
	{
		int kp = k+1;
		Complex xk (x[k][0], x[k][1]);
		Complex xkp (x[kp][0], x[kp][1]);
		if(abs(xk) < abs(xkp) )
		{
			up = true;
		}
		if(up)
		{
			if(abs(xk) > abs(xkp) )
			{
				peakValue = abs(xk);
				peakIndex = k;
				break;
			}
		}
	}
	if(!up)
	{
		cout << "No peak found!" << endl;
	}
	else
	{
		//cout << "Peak value:" << peakValue << endl;
		//set trunction window
		int upIndex = 29, downIndex = 0;
		for(int k = peakIndex+1; k < 30; ++k)
		{
			Complex xk (x[k][0], x[k][1]);
			if(abs(xk) < 0.5*peakValue)
			{
				upIndex = k;
				break;
			}
		}
		for(int k = peakIndex-1; k >= 0; --k)
		{
			Complex xk (x[k][0], x[k][1]);
			if(abs(xk) < 0.5*peakValue)
			{
				downIndex = k;
				break;
			}
		}
		//cout << "The window is: (" << downIndex << ", " << upIndex << ")" << endl;
		//set 0 to data out of the window
		for(int k = 0; k < 30; ++k)
		{
			if(k < downIndex)
			{
				x[k][0] = 0;
				x[k][1] = 0;
			}
			if(k > upIndex)
			{
				x[k][0] = 0;
				x[k][1] = 0;
			}
		}
	}
}

Eigen::MatrixXcd preprocessingCSI(Eigen::MatrixXcd csi)
{	
	Eigen::MatrixXcd smoothedCsi(csi.rows(), csi.cols());
	for(int i = 0; i < csi.rows(); ++i)
	{
		int cols = csi.cols();
		fftw_complex *in, *out;
		fftw_plan p;

		in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * cols );
		out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * cols );
		p = fftw_plan_dft_1d(cols, in, out, FFTW_BACKWARD, FFTW_ESTIMATE);

		for(int k = 0; k < cols; ++k)
		{
			in[k][0] = csi(i,k).real();
			in[k][1] = csi(i,k).imag();
		}
		/*
		cout << "Origin: " << endl;
		for(int k = 0; k < cols; ++k)
		{
			Complex ink (in[k][0], in[k][1]);
			cout << abs(ink) << endl;
			//cout << "(" << in[k][0] << ", " << in[k][1] << ")" << endl;
		}
		*/
		fftw_execute(p);
		/*
		cout << "ifft:" << endl;
		for(int k = 0; k < cols; ++k)
		{
			Complex outk (out[k][0], out[k][1]);
			cout << abs(outk) << endl;
			//cout << "(" << out[k][0] << ", " << out[k][1] << ")"  << endl;
		}
		*/
		for(int k = 0; k < cols; ++k)
		{
			in[k][0] = out[k][0];
			in[k][1] = out[k][1];
		}
		p = fftw_plan_dft_1d(cols, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

		//search first peak
		setWindow(in);

		//cout << "after filtering in time domain:" << endl;
		/*
		for(int k = 0; k < cols; ++k)
		{
			Complex ink (in[k][0], in[k][1]);
			cout << abs(ink) << endl;
		}
		*/
		fftw_execute(p);
		//fft back to CSI
		//cout << "fft:" << endl;
		for(int k = 0; k < cols; ++k)
		{
			//Complex outk (out[k][0]/cols, out[k][1]/cols);
			//cout << abs(outk) << endl;
			//cout << "(" << out[k][0]/cols << ", " << out[k][1]/cols << ")"  << endl;
			Complex newCsi (out[k][0]/cols, out[k][1]/cols);
			smoothedCsi(i,k) = newCsi;
		}
		fftw_destroy_plan(p);
		fftw_free(in); fftw_free(out);
	}		

	return smoothedCsi;
}

int main(int argc, char** argv)
{
	/*init ros*/
	ros::init(argc, argv, "csi_publisher");
	ros::NodeHandle handle;
	csi_pub = handle.advertise<sar_localization::Csi>("csi", 1000);
	//ros::Rate listen_rate(100);
	/* Local variables */
	struct sockaddr_nl proc_addr, kern_addr;	// addrs for recv, send, bind
	struct cn_msg *cmsg;
	char buf[4096];
	int ret;
	unsigned short l;	//l2;
	int count = 0;

	/* Setup the socket */
	sock_fd = socket(PF_NETLINK, SOCK_DGRAM, NETLINK_CONNECTOR);
	if (sock_fd == -1)
		exit_program_err(-1, "socket");

	/* Initialize the address structs */
	memset(&proc_addr, 0, sizeof(struct sockaddr_nl));
	proc_addr.nl_family = AF_NETLINK;
	proc_addr.nl_pid = getpid();			// this process' PID
	proc_addr.nl_groups = CN_IDX_IWLAGN;
	memset(&kern_addr, 0, sizeof(struct sockaddr_nl));
	kern_addr.nl_family = AF_NETLINK;
	kern_addr.nl_pid = 0;					// kernel
	kern_addr.nl_groups = CN_IDX_IWLAGN;

	/* Now bind the socket */
	if (bind(sock_fd, (struct sockaddr *)&proc_addr, sizeof(struct sockaddr_nl)) == -1)
		exit_program_err(-1, "bind");

	/* And subscribe to netlink group */
	{
		int on = proc_addr.nl_groups;
		ret = setsockopt(sock_fd, 270, NETLINK_ADD_MEMBERSHIP, &on, sizeof(on));
		if (ret)
			exit_program_err(-1, "setsockopt");
	}

	/* Set up the "caught_signal" function as this program's sig handler */
	signal(SIGINT, caught_signal);
	map<int, int> phaseMap;
	map<int, int> phaseMapSmooth;
	ofstream csiFile;
	ofstream csiFileSmooth;
	ofstream fftTestFile;
	csiFile.open("CSI_DIS.txt");
	csiFileSmooth.open("CSI_DIS_SMOOTH.txt");
	fftTestFile.open("FFT_TEST.txt");
	/* Poll socket forever waiting for a message */
	while (!g_request_shutdown)
	{
		/* Receive from socket with infinite timeout */
		ret = recv(sock_fd, buf, sizeof(buf), 0);
		//printf("ret = %d\n", ret);
		if (ret == -1)
			exit_program_err(-1, "recv");
		/* Pull out the message portion and print some stats */
		cmsg = (cn_msg*) NLMSG_DATA(buf);
		/*
		if (count % SLOW_MSG_CNT == 0)
			printf("received %d bytes: id: %d val: %d seq: %d clen: %d\n", cmsg->len, cmsg->id.idx, cmsg->id.val, cmsg->seq, cmsg->len);
		*/
		/* Log the data to file */
		l = (unsigned short) cmsg->len;
		//l2 = htons(l);
		//fwrite(&l2, 1, sizeof(unsigned short), out);
		//ret = fwrite(cmsg->data, 1, l, out);
		char *pt = (char*) cmsg->data;
		unsigned char data_code = (unsigned char) *pt;
		///////////////////////////////////
		sar_localization::Csi msg;
		msg.header.stamp = ros::Time::now();
		///////////////////////////////////
		if(data_code == 187)	//Get beamforming
		{
			//printf("Enter data decode\n");
			unsigned long timestamp_low = pt[1] + (pt[2]<<8) + (pt[3]<<16) + (pt[4]<<24);
			unsigned short bfee_count = pt[5] + (pt[6]<<8);
			//printf("bfee_count = %d\n", bfee_count);
			uint8_t Nrx = pt[9];
			//printf("Nrx = %d, ", Nrx);
			uint8_t Ntx = pt[10];
			//printf("Ntx = %d\n", Ntx);
			uint8_t rssi_a = pt[11];
			uint8_t rssi_b = pt[12];
			uint8_t rssi_c = pt[13];
			//printf("rssi_a,b,c = %d, %d, %d\n", rssi_a, rssi_b, rssi_c);
			char noise = pt[14];
			uint8_t agc = pt[15];
			uint8_t antenna_sel = pt[16];
			unsigned int len = (uint8_t) pt[17] + (pt[18]<<8);
			//for (int mi = 0; mi <=212; ++mi)
			//printf("pt[17]=%u, pt[18]=%u\n", (uint8_t) pt[17],(uint8_t) pt[18]);
			//if(len > 400)
			//{
			//	printf("pt[17] is %d, pt[18] is %d\n", pt[17], pt[18]);
			//}
			//printf("len is %u, ", len);
			unsigned int fake_rate_n_flags = pt[19] + (pt[20]<<8);
			unsigned int calc_len = (30 * (Nrx * Ntx * 8 * 2 + 3) + 7)/8;
			//printf("calc_len is %d\n", calc_len);
			unsigned int i, j, k;
			unsigned int index = 0, remainder;
			unsigned char *payload = (unsigned char*) &pt[21];
			char tmp;
			int size[] = {Ntx, Nrx, 30};
			//Eigen::Matrix<std::vector<complex<double> >, Ntx, Nrx> csi_entry;
			//Note!!! We must use all transmitters' csi_entry, but only need two receivers' csi
			Eigen::MatrixXcd csi1_entry(Ntx, 30);
			Eigen::MatrixXcd csi2_entry(Ntx, 30);
			Eigen::MatrixXcd csi3_entry(Ntx, 30);
			Eigen::MatrixXcd csi1_ordered_entry(Ntx, 30);
			Eigen::MatrixXcd csi2_ordered_entry(Ntx, 30);
			Eigen::MatrixXcd csi3_ordered_entry(Ntx, 30);
			Eigen::Matrix3i perm;
			//Check that length matches what it should
			bool check_csi = false;
			if(len != calc_len)
			{

				printf("len = %d, calc_len = %d\n", (unsigned int) len, calc_len);
				perror("Wrong beamforming matrix size");
				check_csi = true;
			}

			for(i = 0; i < 30; ++i)
			{
				index += 3;
				remainder = index % 8;
				//complex<double> tmpComplex;
				for(j = 0; j < Nrx; ++j)
				{
					for(k = 0; k < Ntx; ++k)
					{
						tmp = (payload[index/8]>>remainder) | (payload[index/8+1] << (8-remainder));
						double real = (double) tmp;
						tmp = (payload[index/8+1]>>remainder) | (payload[index/8+2]<<(8-remainder));
						double image = (double) tmp;
						complex<double> tmpCpmlex(real, image);
						if(j == 0)
						{
							csi1_entry(k, i) = tmpCpmlex;
							//cout << "CSI1: " << csi1_entry << endl;
						}
						else if(j == 1)
						{
							csi2_entry(k, i) = tmpCpmlex;
						}
						else if(j == 2)
						{
							csi3_entry(k, i) = tmpCpmlex;
						}
						index += 16;
					}
				}
			}
			perm(0) = ((antenna_sel) & 0x3) + 1;
			perm(1) = ((antenna_sel >> 2) & 0x3) + 1;
			perm(2) = ((antenna_sel >> 4) & 0x3) + 1;
			//cout << "perm: " << perm(0) << ", " << perm(1) << ", " << perm(2) << endl;
			//Get ordered entry. From antenna A to C
			if(perm(0) == 1)
			{
				csi1_ordered_entry = csi1_entry;
				if(perm(1) == 2)
				{
					csi2_ordered_entry = csi2_entry;
					csi3_ordered_entry = csi3_entry;
				}
				else if(perm(1) == 3)
				{
					csi2_ordered_entry = csi3_entry;
					csi3_ordered_entry = csi2_entry;
				}
				else
				{
					perror("perm wrong!\n");
				}
			}
			else if(perm(0) == 2)
			{
				csi1_ordered_entry = csi2_entry;
				if(perm(1) == 1)
				{
					csi2_ordered_entry = csi1_entry;
					csi3_ordered_entry = csi3_entry;
				}
				else if(perm(1) == 3)
				{
					csi2_ordered_entry = csi3_entry;
					csi3_ordered_entry = csi1_entry;
				}
				else
				{
					perror("perm wrong!\n");
				}
			}
			else if(perm(0) == 3)
			{
				csi1_ordered_entry = csi3_entry;
				if(perm(1) == 1)
				{
					csi2_ordered_entry = csi1_entry;
					csi3_ordered_entry = csi2_entry;
				}
				else if(perm(1) == 2)
				{
					csi2_ordered_entry = csi2_entry;
					csi3_ordered_entry = csi1_entry;
				}
				else
				{
					perror("perm wrong!\n");
				}
			}
			else
			{
				perror("perm wrong!\n");
			}

			//Get scaled entry
	        Eigen::MatrixXcd csi1_ordered_entry_conj(Ntx, 30);
	        Eigen::MatrixXcd csi2_ordered_entry_conj(Ntx, 30);
	        Eigen::MatrixXcd csi3_ordered_entry_conj(Ntx, 30);
	        Eigen::MatrixXcd csi1_ordered_entry_sq(Ntx, 30);  //dot product with csi_entry's conjugate
	        Eigen::MatrixXcd csi2_ordered_entry_sq(Ntx, 30);  //dot product with csi_entry's conjugate
	        Eigen::MatrixXcd csi3_ordered_entry_sq(Ntx, 30);  //dot product with csi_entry's conjugate
        	csi1_ordered_entry_conj = csi1_ordered_entry.conjugate();
        	csi2_ordered_entry_conj = csi2_ordered_entry.conjugate();
        	csi3_ordered_entry_conj = csi3_ordered_entry.conjugate();
            csi1_ordered_entry_sq = csi1_ordered_entry.array() * csi1_ordered_entry_conj.array();
            csi2_ordered_entry_sq = csi2_ordered_entry.array() * csi2_ordered_entry_conj.array();
            csi3_ordered_entry_sq = csi3_ordered_entry.array() * csi3_ordered_entry_conj.array();
	        double csi1_ordered_entry_pwr = csi1_ordered_entry_sq.sum().real();
	        double csi2_ordered_entry_pwr = csi2_ordered_entry_sq.sum().real();
	        double csi3_ordered_entry_pwr = csi3_ordered_entry_sq.sum().real();
			double csi_ordered_entry_pwr = csi1_ordered_entry_pwr + csi2_ordered_entry_pwr + csi3_ordered_entry_pwr;
	        //Compute total rssi
	        double rssi_mag = 0;
	        if(rssi_a != 0)
	        {
	        	rssi_mag = rssi_mag + pow(10, (rssi_a/10.0) );
	        }
	        if(rssi_b != 0)
	        {
	            rssi_mag = rssi_mag + pow(10, (rssi_b/10.0) );
	        }
	        if(rssi_c != 0)
	        {
	            rssi_mag = rssi_mag + pow(10, (rssi_c/10.0) );
	        }
	        double rssi_total = 10*log10(rssi_mag) - 44 - agc;
	        double rssi_pwr = pow(10, (rssi_total/10.0) );
	        double scale = rssi_pwr / (csi_ordered_entry_pwr / 30);
	        double noise_db = 0;
	        if( (double) noise == -127)
	        {
	            noise_db = -92;
	        }
	        else
	        {
	        	noise_db = (double) noise;
	        }
	        double thermal_noise_pwr = pow(10, (noise_db/10) );
	        double quant_error_pwr = scale * (Nrx * Ntx);
	        double total_noise_pwr = thermal_noise_pwr + quant_error_pwr;
			Eigen::MatrixXcd csi1(Ntx, 30);
			Eigen::MatrixXcd csi2(Ntx, 30);
			Eigen::MatrixXcd csi3(Ntx, 30);
			csi1 = csi1_ordered_entry * sqrt(scale / total_noise_pwr);
			csi2 = csi2_ordered_entry * sqrt(scale / total_noise_pwr);
			csi3 = csi3_ordered_entry * sqrt(scale / total_noise_pwr);
			if(Ntx == 2)
			{
				csi1 = csi1*sqrt(2);
				csi2 = csi2*sqrt(2);
				csi3 = csi3*sqrt(2);
			}
			else if(Ntx == 3)
			{
				csi1 = csi1*1.6788;
				csi2 = csi2*1.6788;
				csi3 = csi3*1.6788;
			}
			//End of CSI extraction

			//Next
			//Start to publish CSI by ROS Msg
			//sar_localization::Csi msg;
			//msg.header.stamp = ros::Time::now();
			msg.Ntx = Ntx;
			//initialization
			msg.csi1_real.data.clear();
			msg.csi1_image.data.clear();
			msg.csi2_real.data.clear();
			msg.csi2_image.data.clear();
			//bool test = 0;
			Complex hatCSI;
			Complex hatCSISmoothed;
			//double amplitudeHatCSI = 0;
			//test fft effect
			Eigen::MatrixXcd smoothedCsi1(Ntx, 30);
			Eigen::MatrixXcd smoothedCsi2(Ntx, 30);
			//Eigen::MatrixXcd smoothedCsi3(Ntx, 30);
			smoothedCsi1 = preprocessingCSI(csi1);
			smoothedCsi2 = preprocessingCSI(csi2);

			
			for (int i = 0; i < Ntx; ++i)
			{
				for (int j = 0; j < 30; ++j)
				{
					//amplitudeHatCSI += abs(csi1(i,j)*conj(csi2(i,j)) );
					hatCSI += csi1(i,j)*conj(csi2(i,j));
					hatCSISmoothed += smoothedCsi1(i,j)*conj(smoothedCsi2(i,j));
				}
			}
			//amplitudeHatCSI /= Ntx*30.0;
			hatCSI /= Ntx*30.0;
			hatCSISmoothed /= Ntx*30.0;
			//double orientation = acos( (arg(hatCSI)+M_PI)*0.05168/(2*M_PI*0.24) );
			//cout << "hatCSI:  " << abs(hatCSI) << ", phase:" << arg(hatCSI)*180/M_PI << endl; //", orientation: " << orientation << endl;
			if(abs(hatCSISmoothed) < 10 || abs(abs(hatCSI)-abs(hatCSISmoothed) ) > 20)
			{
				//cout << "No LOS signal!!! Drop the CSI!" << endl;
				//int v = phaseMap[arg(hatCSI)*180/M_PI];
				//phaseMap[arg(hatCSI)*180/M_PI] = 1+v;
				continue;
			}
			else
			{	
				cout << "s_hatCSI:" << abs(hatCSISmoothed) << ", phase:" << arg(hatCSISmoothed)*180/M_PI << endl;
				//int v = phaseMap[arg(hatCSI)*180/M_PI];
				//int vpr = phaseMapSmooth[arg(hatCSISmoothed)*180/M_PI];
				//int v = phaseMap[orientation];
				//phaseMap[arg(hatCSI)*180/M_PI] = 1+v;
				//phaseMapSmooth[arg(hatCSISmoothed)*180/M_PI] = 1+vpr;
				//phaseMap[orientation] = 1+v;
				//cout << "Phase map size:" << phaseMap.size() << endl;
				//cout << "Smooth Phase map size:" << phaseMapSmooth.size() << endl;
			
				for (int i = 0; i < Ntx; ++i)
				{
					for (int j = 0; j < 30; ++j)
					{
						msg.csi1_real.data.push_back(csi1(i, j).real() );
						msg.csi1_image.data.push_back(csi1(i, j).imag() );
						msg.csi2_real.data.push_back(csi2(i, j).real() );
						msg.csi2_image.data.push_back(csi2(i, j).imag() );
					}
				}
				msg.check_csi = check_csi;
				csi_pub.publish(msg);
				++count;
				if (count % 100 == 0)
					printf("receive %d bytes [msgcnt=%u]\n", ret, count);
			}
		}	//End of if(code)
		ros::spinOnce();
		//printf("Exit decoder\n");

		//++count;
		//printf("ret = %d, l = %d\n", ret, l);
		//if (ret != l)
		//	exit_program_err(1, "fwrite");
	}
	map<int, int>::iterator iter1 = phaseMap.begin();
	map<int, int>::iterator iter2 = phaseMapSmooth.begin();
	while(iter1 != phaseMap.end() )
	{
		csiFile << iter1->first << " " << iter1->second << endl;
		//cout << iter1->first << ", " << iter1->second << "; " << iter2->first << ", " << iter2->second << endl;
		++iter1;
	}
	while(iter2 != phaseMapSmooth.end())
	{
		csiFileSmooth << iter2->first << " " << iter2->second << endl;
		++iter2;
	}
	csiFile.close();
	csiFileSmooth.close();
	fftTestFile.close();
	exit_program(0);
	return 0;
}

/*
void check_usage(int argc, char** argv)
{
	if (argc != 2)
	{
		fprintf(stderr, "Usage: log_to_file <output_file>\n");
		exit_program(1);
	}
}

FILE* open_file(char* filename, char* spec)
{
	FILE* fp = fopen(filename, spec);
	if (!fp)
	{
		perror("fopen");
		exit_program(1);
	}
	return fp;
}
*/
void caught_signal(int sig)
{
	fprintf(stderr, "Caught signal %d\n", sig);
	g_request_shutdown = 1;
	//exit_program(0);
}

void exit_program(int code)
{
	//if (out)
	//{
	//	fclose(out);
	//	out = NULL;
	//}
	if (sock_fd != -1)
	{
		close(sock_fd);
		sock_fd = -1;
	}
	exit(code);
}

void exit_program_err(int code, char* func)
{
	perror(func);
	exit_program(code);
}
