/*---------------------------------------------------------------------------*/
/**
  @file		serial.h
  @brief	Serial API header file

  Serial utility functions, it helps programmer easily operating serial port.
  It provides mostly standard functions we usually use, and SerialSetMode()
  is the private function can be use in UC box.
 */
/*---------------------------------------------------------------------------*/

#ifndef SERIAL_H
#define SERIAL_H

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h> //g++ compiler need
#include <asm/ioctls.h>
#include "myLib.h"
#include "DataGather.h"

#define PORT1 0
#define PORT2 1
#define PORT3 2
#define PORT4 3
#define PORT5 4
#define PORT6 5
#define PORT7 6
#define PORT8 7
#define PORT9 8
#define PORT10 9
#define PORT11 10
#define PORT12 11
#define PORT13 12
#define PORT14 13
#define PORT15 14
#define PORT16 15

#define Max_Port_Num (16)

#define NO_FLOW_CONTROL 0
#define HW_FLOW_CONTROL 1
#define SW_FLOW_CONTROL 2

#define SERIAL_OK 0
#define SERIAL_ERROR_FD -1        ///< Could not find the fd in the map, device not opened
#define SERIAL_ERROR_OPEN -2      ///< Could not open the port or port has been opened
#define SERIAL_PARAMETER_ERROR -3 ///< Not available parameter

typedef struct _SerialPara
{
  int iBaudRate;
  int iDataBits;
  int iStopBits;
  int iParity;
  int iMode;
} structSerialPara;

static structSerialPara g_SerialParaSetting[Max_Port_Num];

int SerialOpen(const char *device, int port);
int SerialFlush(int port);
int SerialWrite(int iPort, void *pvData, int iLen);
int SerialNonBlockRead(int port, char *buf, int len);
int SerialBlockRead(int port, char *buf, int len);
int SerialRead(int port, uchar_t *buf, int len, int waitms, int timeoutms);
int SerialClose(int port);
int SerialDataInInputQueue(int port);
int SerialDataInOutputQueue(int port);
int SerialFlowControl(int port, int control);
int SerialSetSpeed(int port, unsigned int speed);
int SerialSetMode(int port, unsigned int mode);
int SerialSetParam(int port, int databits, int stopbit, int parity);
int SerailSetRts(int port, bool set);
int FindFD(int port);

int jSetCommPara(int iPort, uchar_t ucAddr, uchar_t ucCmd);
int jSerialRead(int iPort, uchar_t *pucBuf, int iLen, int iTimeOut);
int jSerialRead2(int iPort, uchar_t *pucBuf, int iLen, int iTimeOut);

#endif
