#ifndef _MYLIB_H_
#define _MYLIB_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h> //g++ compiler need
#include <fcntl.h>
#include <asm/types.h>
#include <linux/watchdog.h>
#include <sys/mount.h>
#include <errno.h>
#include <asm-generic/errno-base.h> //comm para

typedef unsigned int uint_t;
typedef unsigned char uchar_t;
typedef unsigned short ushort_t;
typedef unsigned long ulong_t;
struct ST_CommPara
{
    char tSerialPort[100];
    int iEnable;
    int iPort;
    int iBaud;
    int iDataBits;
    int iStopBits;
    int iParity;
    int iTimeInterVal;
};
enum AD104_Cmd
{
    /* avaliable in running mode */
    Cmd_Msv1Tav = 1,
    Cmd_Msv45 = 2,
    Cmd_Msv200 = 3,
    Cmd_Msvn = 4,
    Cmd_Tav = 5,
    Cmd_Ldw = 6,
    Cmd_Lwt = 7,
    Cmd_Tar = 8,
    Cmd_Tdd1 = 9,
    Cmd_RTav = 10,
    Cmd_RBdrLdwLwt = 11,
    Cmd_Initial = 12,
    cmd_msv200Test = 13,
    cmd_msvnTav = 14,
    Cmd_RBdr = 15,
    Cmd_InitialTav = 16,
};
void jWaitMs(ulong_t ulTime); // wait ms
long jGetTick(void);
int loginput(char *logcontent);
void jPselect(unsigned long milliseconds);
long timestamp();
/*调试打印*/
void hex_dump(const unsigned char *data, int data_len, int width);
#endif
