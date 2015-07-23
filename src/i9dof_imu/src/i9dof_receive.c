/*
float   yaw,
        pitch,
        roll,
        alt,
        tempr,
        press;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t hx, hy, hz;
------------------------------------
*/

#include    <stdio.h>
#include    <stdint.h>
//uart reicer flag
#define b_uart_head  0x80
#define b_rx_over    0x40
// USART Receiver buffer
#define RX_BUFFER_SIZE 100

void Decode_frame(unsigned char data);
volatile unsigned char rx_buffer[RX_BUFFER_SIZE];
volatile unsigned char rx_wr_index;
volatile unsigned char RC_Flag;

float   yaw,
        pitch,
        roll,
        alt,
        tempr,
        press;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t hx, hy, hz;

float GPS_Altitude ,
      Latitude_GPS ,
      Longitude_GPS ,
      Speed_GPS ,
      Course_GPS ;
unsigned char GPS_STA_Num ;

void UART2_Get_Motion(void)
{
    int16_t temp;

    temp = 0;
    temp = rx_buffer[2];
    temp <<= 8;
    temp |= rx_buffer[3];
    if (temp & 0x8000)
    {
        temp = 0 - (temp & 0x7fff);
    }
    else temp = (temp & 0x7fff);
    yaw = (float)temp / 10.0f;

    temp = 0;
    temp = rx_buffer[4];
    temp <<= 8;
    temp |= rx_buffer[5];
    if (temp & 0x8000)
    {
        temp = 0 - (temp & 0x7fff);
    }
    else temp = (temp & 0x7fff);
    pitch = (float)temp / 10.0f;

    temp = 0;
    temp = rx_buffer[6];
    temp <<= 8;
    temp |= rx_buffer[7];
    if (temp & 0x8000)
    {
        temp = 0 - (temp & 0x7fff);
    }
    else temp = (temp & 0x7fff);
    roll = (float)temp / 10.0f;

    temp = 0;
    temp = rx_buffer[8];
    temp <<= 8;
    temp |= rx_buffer[9];
    if (temp & 0x8000)
    {
        temp = 0 - (temp & 0x7fff);
    }
    else temp = (temp & 0x7fff);
    alt = (float)temp / 10.0f;

    temp = 0;
    temp = rx_buffer[10];
    temp <<= 8;
    temp |= rx_buffer[11];
    if (temp & 0x8000)
    {
        temp = 0 - (temp & 0x7fff);
    }
    else temp = (temp & 0x7fff);
    tempr = (float)temp / 10.0f;

    temp = 0;
    temp = rx_buffer[12];
    temp <<= 8;
    temp |= rx_buffer[13];
    if (temp & 0x8000)
    {
        temp = 0 - (temp & 0x7fff);
    }
    else temp = (temp & 0x7fff);
    press = (float)temp * 10.0f;

}

void UART2_Get_IMU(void)
{
    int16_t temp;

    temp = 0;
    temp = rx_buffer[2];
    temp <<= 8;
    temp |= rx_buffer[3];
    if (temp & 0x8000)
    {
        temp = 0 - (temp & 0x7fff);
    }
    else temp = (temp & 0x7fff);
    ax = temp;

    temp = 0;
    temp = rx_buffer[4];
    temp <<= 8;
    temp |= rx_buffer[5];
    if (temp & 0x8000)
    {
        temp = 0 - (temp & 0x7fff);
    }
    else temp = (temp & 0x7fff);
    ay = temp;

    temp = 0;
    temp = rx_buffer[6];
    temp <<= 8;
    temp |= rx_buffer[7];
    if (temp & 0x8000)
    {
        temp = 0 - (temp & 0x7fff);
    }
    else temp = (temp & 0x7fff);
    az = temp;

    temp = 0;
    temp = rx_buffer[8];
    temp <<= 8;
    temp |= rx_buffer[9];
    if (temp & 0x8000)
    {
        temp = 0 - (temp & 0x7fff);
    }
    else temp = (temp & 0x7fff);
    gx = temp;

    temp = 0;
    temp = rx_buffer[10];
    temp <<= 8;
    temp |= rx_buffer[11];
    if (temp & 0x8000)
    {
        temp = 0 - (temp & 0x7fff);
    }
    else temp = (temp & 0x7fff);
    gy = temp;

    temp = 0;
    temp = rx_buffer[12];
    temp <<= 8;
    temp |= rx_buffer[13];
    if (temp & 0x8000)
    {
        temp = 0 - (temp & 0x7fff);
    }
    else temp = (temp & 0x7fff);
    gz = temp;

    temp = 0;
    temp = rx_buffer[14];
    temp <<= 8;
    temp |= rx_buffer[15];
    if (temp & 0x8000)
    {
        temp = 0 - (temp & 0x7fff);
    }
    else temp = (temp & 0x7fff);
    hx = temp;

    temp = 0;
    temp = rx_buffer[16];
    temp <<= 8;
    temp |= rx_buffer[17];
    if (temp & 0x8000)
    {
        temp = 0 - (temp & 0x7fff);
    }
    else temp = (temp & 0x7fff);
    hy = temp;

    temp = 0;
    temp = rx_buffer[18];
    temp <<= 8;
    temp |= rx_buffer[19];
    if (temp & 0x8000)
    {
        temp = 0 - (temp & 0x7fff);
    }
    else temp = (temp & 0x7fff);
    hz = temp;
}
unsigned char Sum_check(void)
{
    unsigned char i;
    unsigned int checksum = 0;
    for (i = 0; i < rx_buffer[0] - 2; i++)
        checksum += rx_buffer[i];
    if ((checksum % 256) == rx_buffer[rx_buffer[0] - 2])
        return (0x01); //Checksum successful
    else
        return (0x00); //Checksum error
}

//------------------------------------------------------

