#ifndef _DATAGATHER_H_
#define _DATAGATHER_H_

#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <asm/ioctls.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/types.h>
#include <sys/shm.h>
#include <sys/ipc.h>
#include <resolv.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>
#include <iostream>
#include <signal.h>
#include "include/Exception.h"
#include "include/Parse.h"
#include "include/TcpSocket.h"
#include "include/SocketHandler.h"
#include "include/ListenSocket.h"
#include "include/config.h"
#include "CModbusTcp.h"
#include "serial.h"
#include "myLib.h"
#include "CD_network.h"
#include "matrix500.h"
#include "xini_file.h"
#include "log.h"
#include "NDIO.h"

// #define Max_Port_Num  8    define in serial.h
/* BEGIN:added by wangjiafan 20140319 */
#ifndef IN
#define IN
#endif

#ifndef OUT
#define OUT
#endif

#ifndef INOUT
#define INOUT
#endif

#define MaxData_BufferLen 6000
#define ConfigFile "DG-M.ini"

// PLC写地址

#define PlcWAdr_Command 10
#define PlcWAdr_RunStop 11

#define PlcWAdr_FunctionSelect 20
#define PlcWAdr_Com_Enable 21
#define PlcWAdr_RunColumnSel 22
#define PlcWAdr_OpColumnSel 23
#define PlcWAdr_ReadCount 24
#define PlcWAdr_WTDealy 25
#define PlcWAdr_WTTime 26
#define PlcWAdr_TavValueRange 27

// 8列40-56
#define PlcWAdr_Column1_LTAV 40
#define PlcWAdr_Column1_HTAV 41
#define PlcWAdr_ColumnEnable_TAV 56
//
// PLC读地址
#define PlcRAdr_IDSignal 100
#define PlcRAdr_ComRun 101
#define PlcRAdr_RunColumnSel 102
#define PlcRAdr_OpColumnSel 103
#define PlcRAdr_RunStopStatus 104
#define PlcRAdr_ErrorCode1 105
#define PlcRAdr_ErrorCode2 106
#define PlcRAdr_ErrorCode3 107
#define PlcRAdr_ErrorCode4 108
#define PlcRAdr_ErrorCode5 109
//
#define PlcRAdr_ReadCount 110
#define PlcRAdr_WTDealy 111
#define PlcRAdr_WTTime 112
#define PlcRAdr_TavValueRange 113

//
#define PlcRAdr_USBStatus 115
#define PlcRAdr_FunctionSelect 116
// 8列120-139
#define PlcRAdr_ReadWeightStatus 120
#define PlcRAdr_Column1_LWeight 122
#define PlcRAdr_Column1_HWeight 123
#define PlcRAdr_ReadTavStatus 156
#define PlcRAdr_ZeroRangeStatus 157
#define PlcRAdr_Column1_LTAV 158
#define PlcRAdr_Column1_HTAV 159

// 8列160
#define PlcRAdr_ReadBdrStatus 192
#define PlcRAdr_Column1_LBDR 194
#define PlcRAdr_Column1_HBDR 195
// 8列180
#define PlcRAdr_ReadLdwStatus 228
#define PlcRAdr_Column1_LLDW 229
#define PlcRAdr_Column1_HLDW 230
// 8列200-219
#define PlcRAdr_ReadLwtStatus 264
#define PlcRAdr_Column1_LLWT 266
#define PlcRAdr_Column1_HLWT 267
//
#define Weight_RunCode 101
#define Weight_StopCode 201
#define Weight_ResetCode 301
//
#define Serial_ReadMaxLen 100
#define Serial_MaxLen 1024
#define Serial_Read_WaitTime 1000 /* waiting time for serial port reading */
#define Serial_Read_TimeOut 40    /* timeout for serial port reading */
/* define function return values here */
#define Return_HighLevel 3
#define Return_LowLevel 2
#define Return_Success 1
#define Return_NotRecv 0
#define Return_Failed -1
#define Return_Invalid_Para -2
#define Return_OpenSerialError -3
#define Return_SerialInvalid -4
#define Return_SerialTimeOut -5
#define Return_SerialRecvLenError -6
#define Return_SerialCmdError -7

/* define invallied data here to initialize local vars  */
#define Int_Initial 0xffffffff
#define UShort_Initial 0xffff

/* define serial operating data here */
#define MaxIo_Nums 21

#define AD104_Cyc 5
// declare function
int jProInit(void);
int jReadFileInit(void);
int jWriteInit(int iPort);
int jGPIoInit(void);
int jProcWSysPara(void);
int jProcRWPara(void);
int jProcWColumnSel(void);
void jProcReset(void);
int jGet_di_state(unsigned char id);
int jGet_do_state(unsigned char id);

void *jProcCommOperation(void *pv);
void *jProcComServer(void *pv);
void *jProcModbusCommandJudge(void *pv);
void *jProcModbusTcpServer(void *pv);
void *jProcIoMsv(void *pv);
void *jProcIoMsvOut(void *pv);
void *jProcloRun(void *pv);
void *jProcIoReadWrite(void *pv);
void jSemCreat(void);

int jDiCheckPositive(int i_gpio, bool OverEnable, long OverTime);
int jDiCheckNegative(int i_gpio, bool OverEnable, long OverTime);
// int jDoSet(int igpio);
// int jDoClear(int igpio);
int jProc_SendSerialCommand(int i_port, void *p_data, void *msv_len);
int jProc_ReceiveSerialCom(int i_port, void *p_data, void *msv_len);

// data buf function
int jWriteDataBuf(ushort_t *pusTarget, char *pcSsource, int iStartAddr, int iNum);
int jReadDataBuf(char *pcRet, ushort_t *pusBuffer, int iStartAddr, int iNum);

#endif
