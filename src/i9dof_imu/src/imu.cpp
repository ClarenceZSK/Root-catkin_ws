#include    "ros/ros.h"
#include    <iostream>
#include    <cmath>
#include    "sensor_msgs/Imu.h"
#include    "include/usbserial.h"
#include    <cstdlib>
#include    <errno.h>
#include    "i9dof_receive.c"
using namespace std;
ros::Publisher  imu_pub;

sensor_msgs::Imu    imu;
bool    silent      = false;              ///< Wether console output should be enabled
bool    verbose     = false;             ///< Enable verbose output
bool    debug       = false;               ///< Enable debug functions and output
int     fd;

void UART2_CommandRoute(void)
{
    if (RC_Flag & b_rx_over)
    {
        RC_Flag &= ~b_rx_over;
        if (Sum_check())
        {
            if (rx_buffer[1] == 0xA2)
            {
                UART2_Get_IMU();
                ros::Time  imuTimeNoDelay(ros::Time::now().toSec() - 0.00104);
                imu.header.stamp = imuTimeNoDelay;
                //if(ax<100 && ax > -100 && ay<100 && ay > -100)
                //{
                //    cout << "ax: " << ax <<  "  \t  \t  ay: " << ay << " \t  \t  az: " << az << "  \t \t a:" << sqrt(ax*ax + ay*ay + az*az)<< endl;
                //}
                imu.linear_acceleration.x = 9.8 * ax / 8200.0;
                imu.linear_acceleration.y = 9.8 * (ay + 65) / 8185.0;
                imu.linear_acceleration.z = 9.8 * (az + 350) / 8350.0;
                imu.angular_velocity.x  = M_PI * (gx + 170) / 11790;
                imu.angular_velocity.y  = M_PI * (gy - 5) / 11790;
                imu.angular_velocity.z  = M_PI * (gz - 31) / 11790;
                imu_pub.publish(imu);
                //double  d = imu.linear_acceleration.x * imu.linear_acceleration.x + imu.linear_acceleration.y * imu.linear_acceleration.y + imu.linear_acceleration.z * imu.linear_acceleration.z;
                //cout << "d: " << sqrt(d) << endl;
            }
            if (rx_buffer[1] == 0xA1)
            {
                UART2_Get_Motion();
            }
        }
    }
}

void Decode_frame(unsigned char data)
{
    if (data == 0xa5)
    {
        RC_Flag |= b_uart_head;
        rx_buffer[rx_wr_index++] = data;
    }
    else if (data == 0x5a)
    {
        if (RC_Flag & b_uart_head)
        {
            rx_wr_index = 0;
            RC_Flag &= ~b_rx_over;
        }
        else
            rx_buffer[rx_wr_index++] = data;
        RC_Flag &= ~b_uart_head;
    }
    else
    {
        rx_buffer[rx_wr_index++] = data;
        RC_Flag &= ~b_uart_head;
        if (rx_wr_index == rx_buffer[0])
        {
            RC_Flag |= b_rx_over;
            UART2_CommandRoute();
        }
    }
    if (rx_wr_index == RX_BUFFER_SIZE)
        rx_wr_index--;
}

void serial_wait( void* serial_ptr, ros::NodeHandle& n)
{
    int fd  = *((int*)serial_ptr);
    while (n.ok())
    {
        uint8_t cp;
        if (read(fd, &cp, 1) > 0)
        {
            Decode_frame(cp);
        }
        ros::spinOnce();
    }
}

int main(int argc,  char** argv)
{
    ros::init( argc,  argv,  "i9dof_imu" );
    ros::NodeHandle     imu("~");
    imu_pub      = imu.advertise<sensor_msgs::Imu>("imu",    1000);

    /* default values for arguments */
    char *uart_name = (char*)"/dev/imu_board";
    int baudrate = 921600;
    const char *commandline_usage = "\tusage: %s -d <devicename> -b <baudrate> [-v/--verbose] [--debug]\n\t\tdefault: -d %s -b %i\n";
    /* read program arguments */
    int i;
    for (i = 1; i < argc; i++)   /* argv[0] is "mavlink" */
    {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
        {
            printf(commandline_usage, argv[0], uart_name, baudrate);
            return 0;
        }
        /* UART device ID */
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0)
        {
            if (argc > i + 1)
            {
                uart_name = argv[i + 1];

            }
            else
            {
                printf(commandline_usage, argv[0], uart_name, baudrate);
                return 0;
            }
        }
        /* baud rate */
        if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0)
        {
            if (argc > i + 1)
            {
                baudrate = atoi(argv[i + 1]);

            }
            else
            {
                printf(commandline_usage, argv[0], uart_name, baudrate);
                return 0;
            }
        }
        /* terminating MAVLink is allowed - yes/no */
        if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0)
        {
            verbose = true;
        }
        if (strcmp(argv[i], "--debug") == 0)
        {
            debug = true;
        }
    }

    // SETUP SERIAL PORT
    // Exit if opening port failed
    // Open the serial port.
    if (!silent) printf("Trying to connect to %s.. ", uart_name);
    fflush(stdout);
    fd = open_port(uart_name);
    if (fd == -1)
    {
        if (!silent) printf("failure, could not open port.\n");
        exit(EXIT_FAILURE);
    }
    else
    {
        if (!silent) printf("success.\n");
    }
    if (!silent) printf("Trying to configure %s.. ", uart_name);
    bool setup;
    setup = setup_port(fd, baudrate);


    int noErrors = 0;
    if (fd == -1 || fd == 0)
    {
        if (!silent) fprintf(stderr, "Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", uart_name, baudrate);
        exit(EXIT_FAILURE);
    }
    else
    {
        if (!silent) fprintf(stderr, "\nConnected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
    }
    if (fd < 0)
    {
        exit(noErrors);
    }

    // Run indefinitely while the serial loop handles data
    if (!silent) printf("\nREADY, waiting for serial data.\n");

    int* fd_ptr = &fd;

    if (fd < 0)
    {
        exit(noErrors);
    }

    serial_wait(fd_ptr, imu);

    close_port(fd);

    return 0;
}
