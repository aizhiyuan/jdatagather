/*---------------------------------------------------------------------------*/
/**
  @file		serial.c
  @brief	Serial API define file

  Serial utility functions, it helps programmer easily operating serial port.
  It provides mostly standard functions we usually use, and SerialSetMode()
  is the private function can be use in UC box.

 */
/*---------------------------------------------------------------------------*/

#include "serial.h"

static struct termios oldtio[Max_Port_Num], newtio[Max_Port_Num];
static int fd_map[Max_Port_Num] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}; ///< -1 means SERIAL_ERROR_FD

// add by gxm
static char m_acAddr[Max_Port_Num] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}; // addr
static char m_acCmd[Max_Port_Num] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};  // cmd
static int m_aiState[Max_Port_Num] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}; // states

/*---------------------------------------------------------------------------*/
/**
  @brief	use port number to find out the fd of the port
  @param	port	port number
  @return	fd which is opened by using SerialOpen()
 */
/*---------------------------------------------------------------------------*/
int FindFD(int port)
{
    if (fd_map[port] == -1)
        return SERIAL_ERROR_FD;

    return fd_map[port];
}

// set Comm Para
int jSetCommPara(int iPort, uchar_t ucAddr, uchar_t ucCmd)
{
    m_acAddr[iPort] = ucAddr;
    m_acCmd[iPort] = ucCmd;
}

/*---------------------------------------------------------------------------*/
/**
  @brief	open serial port
  @param	port		port number
  @return	return fd for success, on error return error code
 */
/*---------------------------------------------------------------------------*/
int SerialOpen(const char *device, int port)
{
    int fd;
    // char device[80];
    struct termios tio;

    if (fd_map[port] != -1) ///< port already opened
        return SERIAL_ERROR_OPEN;

    // sprintf(device, "/dev/ttyS%d", port + 1);
    fd = open(device, O_RDWR | O_NOCTTY);
    if (fd < 0)
        return SERIAL_ERROR_OPEN;

    fd_map[port] = fd;

    // bzero( &tio, sizeof(tio));		///< clear struct for new port settings
    memset(&tio, 0, sizeof(tio));

    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_cflag = B9600 | CS8 | CREAD | CLOCAL;
    tio.c_lflag = 0;

    tio.c_cc[VTIME] = 0; ///< inter-character timer unused
    tio.c_cc[VMIN] = 1;  ///< blocking read until 1 character arrives

    tcgetattr(fd, &(oldtio[port])); ///< save current serial port settings
    newtio[port] = tio;

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio[port]);

    return fd;
}

int SerialFlush(int port)
{
    int iRet;
    int fd = FindFD(port);

    if (fd < 0) ///< error
        return fd;

    iRet = tcflush(fd, TCIFLUSH);
    // if(iRet < 0)  iRet = write( fd, str, len);  //200805 microlee  try again
    return iRet;
}

/*---------------------------------------------------------------------------*/
/**
  @brief	write to serial port
  @param	port		port number
  @param	str		string to write
  @param	len		length of str
  @return	return length of str for success, on error return error code
 */
/*---------------------------------------------------------------------------*/
int SerialWrite(int iPort, void *pvData, int iLen)
{
    int iRet;
    int iFd = FindFD(iPort);

    if (iFd < 0) ///< error
        return iFd;

    char *pcTmp = (char *)pvData;

    jSetCommPara(iPort, *pcTmp, *(pcTmp + 1));

    iRet = write(iFd, pvData, iLen);
    // if(iRet < 0)  iRet = write( fd, str, len);  //200805 microlee  try again
    return iRet;
}

/*---------------------------------------------------------------------------*/
/**
  @brief	non-block read from serial port
  @param	port		port number
  @param	buf		input buffer
  @param	len		buffer length
  @return	return length of read str for success,
        on error return error code
 */
/*---------------------------------------------------------------------------*/
int SerialNonBlockRead(int port, char *buf, int len)
{
    int res = 0;
    int bytes = 0;
    int fd = FindFD(port);

    if (fd < 0) ///< error
        return fd;

    fcntl(fd, F_SETFL, FNDELAY);
    res = read(fd, buf, len);
    return res;
}

/*---------------------------------------------------------------------------*/
/**
  @brief	block read from serial port
  @param	port		port number
  @param	buf		input buffer
  @param	len		buffer length
  @return	return length of read str for success,
        on error return error code
 */
/*---------------------------------------------------------------------------*/
int SerialBlockRead(int port, char *buf, int len)
{
    int res = 0;
    int bytes = 0;
    int fd = FindFD(port);

    if (fd < 0) ///< error
        return fd;

    fcntl(fd, F_SETFL, 0);
    res = read(fd, buf, len);
    return res;
}

/*---------------------------------------------------------------------------*/
/**
  @brief	read from serial port
  @param	port		port number
  @param	buf		input buffer
  @param	len		buffer length
  @param	waitms		waiting ms then bigen to read
  @param	timeoutms
  @return	return length of read str for success,
        on error return error code
 */
/*---------------------------------------------------------------------------*/
int SerialRead(int port, uchar_t *buf, int len, int waitms, int timeoutms)
{
    int retval;
    fd_set rfds;
    struct timeval tv;
    int ret;
    int pos;
    int fd = FindFD(port);

    if (fd < 0) ///< error
        return fd;

    fcntl(fd, F_SETFL, FNDELAY);

    tv.tv_sec = waitms / 1000; // wait 2.5s
    tv.tv_usec = waitms % 1000;
    tv.tv_usec = tv.tv_usec * 1000;
    pos = 0; // point to rceeive buf
    while (1)
    {
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        retval = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (retval == -1)
        {
            // perror("select()");
            break;
        }
        else if (retval)
        {
            // pan duan shi fou hai you shu ju
            ret = read(fd, buf + pos, len - pos);
            pos += ret;
            if (pos < len)
            {
                tv.tv_sec = timeoutms / 1000; // waiting frame timeout
                tv.tv_usec = timeoutms % 1000;
                tv.tv_usec = tv.tv_usec * 1000;
                continue;
            }
            else
            {
                break;
            }
        }
        else
        {
            // printf("No data\n");
            break;
        }
    }
    return pos;
}

// jSerialRead
int jSerialRead(int iPort, uchar_t *pucBuf, int iLen, int vTimeOut)
{
    int retval;
    long lngStart, lngEnd;
    uchar_t aucBufs[256];
    int intPackLen;
    int intReadLen;
    int intRtn;
    fd_set rfds;
    struct timeval tv;

    int fd = FindFD(iPort);

    fcntl(fd, F_SETFL, FNDELAY);

    tv.tv_sec = 200 / 1000; // wait 2.5s
    tv.tv_usec = 200 % 1000;
    tv.tv_usec = tv.tv_usec * 1000;

    if (fd < 0) ///< error
        return fd;

    fcntl(fd, F_SETFL, 0);
    lngStart = jGetTick();

    m_aiState[iPort] = -1;
    intRtn = 0;

    while (1)
    {
        lngEnd = jGetTick();
        if ((lngEnd - lngStart) > vTimeOut)
        {
            // printf("lngEnd = %ld lngStart = %ld  time out = %ld timeOut = %d \n",lngEnd, lngStart, lngEnd - lngStart, vTimeOut);
            return intRtn;
        }

        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        retval = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (retval == -1)
            continue;
        if (retval == 0)
            continue;

        intReadLen = read(fd, aucBufs, 256);

        for (int i = 0; i < intReadLen; i++)
        {
            switch (m_aiState[iPort])
            {
            case -1: // wait addr
                if (aucBufs[i] == m_acAddr[iPort])
                {
                    intRtn = 0;
                    *(pucBuf + intRtn) = aucBufs[i];
                    intRtn++;
                    m_aiState[iPort] = 1;
                }
                break;
            case 1: // wait cmd
                if (aucBufs[i] == m_acCmd[iPort])
                {
                    *(pucBuf + intRtn) = aucBufs[i];
                    intRtn++;

                    switch (m_acCmd[iPort])
                    {
                    case 0x01:
                    case 0x02:
                    case 0x03:
                    case 0x04:
                        m_aiState[iPort] = 2;
                        break;
                    case 0x05:
                    case 0x06:
                    case 0x10:
                        intPackLen = 8;
                        m_aiState[iPort] = 3;
                    }
                }
                else
                {
                    m_aiState[iPort] = -1;
                }
                break;
            case 2: // wait length
                *(pucBuf + intRtn) = aucBufs[i];
                intRtn++;
                intPackLen = aucBufs[i] + 5;
                m_aiState[iPort] = 3;
                break;
            case 3: // read data
                *(pucBuf + intRtn) = aucBufs[i];
                intRtn++;
                if (intPackLen == intRtn)
                {
                    return intRtn;
                }
                break;
            default:
                m_aiState[iPort] = -1;
            }
        }
    }
    return 0;
}

// jSerialRead read modbus cmd
int jSerialRead2(int iPort, uchar_t *pucBuf, int iLen, int vTimeOut)
{
    int retval;
    long lngStart, lngEnd;
    uchar_t aucBufs[256];
    int intPackLen;
    int intReadLen;
    int intRtn;
    fd_set rfds;
    struct timeval tv;

    int fd = FindFD(iPort);

    fcntl(fd, F_SETFL, FNDELAY);

    tv.tv_sec = 200 / 1000; // wait 2.5s
    tv.tv_usec = 200 % 1000;
    tv.tv_usec = tv.tv_usec * 1000;

    if (fd < 0) ///< error
        return fd;

    fcntl(fd, F_SETFL, 0);
    lngStart = jGetTick();

    m_aiState[iPort] = -1;
    intRtn = 0;

    while (1)
    {
        lngEnd = jGetTick();
        if ((lngEnd - lngStart) > vTimeOut)
        {
            // printf("lngEnd = %ld lngStart = %ld  time out = %ld timeOut = %d \n",lngEnd, lngStart, lngEnd - lngStart, vTimeOut);
            return intRtn;
        }

        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        retval = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (retval == -1)
            continue;
        if (retval == 0)
            continue;

        intReadLen = read(fd, aucBufs, 256);
        if (iPort == 3)
        {
            printf("============== Rtu send rcv len = %d  Addr = %d  cmd = %d \n", intReadLen, m_acAddr[iPort], m_acCmd[iPort]);
        }
        for (int i = 0; i < intReadLen; i++)
        {
            printf("============== %2x", aucBufs[i]);
            switch (m_aiState[iPort])
            {
            case -1: // wait addr
                if (aucBufs[i] == m_acAddr[iPort])
                {
                    intRtn = 0;
                    *(pucBuf + intRtn) = aucBufs[i];
                    intRtn++;
                    m_aiState[iPort] = 1;
                }
                break;
            case 1: // wait cmd
                if (aucBufs[i] == m_acCmd[iPort])
                {
                    *(pucBuf + intRtn) = aucBufs[i];
                    intRtn++;

                    switch (m_acCmd[iPort])
                    {
                    case 0x01:
                    case 0x02:
                    case 0x03:
                    case 0x04:
                        intPackLen = 8;
                        m_aiState[iPort] = 3;
                        break;
                    case 0x05:
                    case 0x06:
                    case 0x10:
                        intPackLen = 8;
                        m_aiState[iPort] = 3;
                    }
                }
                else
                {
                    m_aiState[iPort] = -1;
                }
                break;
            case 2: // wait length
                *(pucBuf + intRtn) = aucBufs[i];
                intRtn++;
                intPackLen = aucBufs[i] + 5;
                m_aiState[iPort] = 3;
                break;
            case 3: // read data
                *(pucBuf + intRtn) = aucBufs[i];
                intRtn++;
                if (intPackLen == intRtn)
                {
                    return intRtn;
                }
                break;
            default:
                m_aiState[iPort] = -1;
            }
        }
    }
    return 0;
}

/*---------------------------------------------------------------------------*/
/**
  @brief	close serial port
  @param	port		port number
  @return	return SERIAL_OK for success, on error return error code
 */
/*---------------------------------------------------------------------------*/
int SerialClose(int port)
{
    int fd = FindFD(port);

    if (fd < 0) ///< error
        return fd;

    tcsetattr(fd, TCSANOW, &(oldtio[port]));
    close(fd);

    fd_map[port] = -1;

    return SERIAL_OK;
}

/*---------------------------------------------------------------------------*/
/**
  @brief	test how much data in input queue
  @param	port		port number
  @return	return number of data to be read for success,
        on error return error code
 */
/*---------------------------------------------------------------------------*/
int SerialDataInInputQueue(int port)
{
    int bytes = 0;
    int fd = FindFD(port);

    if (fd < 0) ///< error
        return fd;

    ioctl(fd, FIONREAD, &bytes);
    return bytes;
}

/*---------------------------------------------------------------------------*/
/**
  @brief	test how much data in output queue
  @param	port		port number
  @return	return number of data to be write for success,
        on error return error code
 */
/*---------------------------------------------------------------------------*/
int SerialDataInOutputQueue(int port)
{
    int bytes = 0;
    int fd = FindFD(port);

    if (fd < 0) ///< error
        return fd;

    ioctl(fd, TIOCOUTQ, &bytes);
    return bytes;
}
/*---------------------------------------------------------------------------*/
/**
  @brief	set flow control
  @param	port		port number
  @param	control		NO_FLOW_CONTROL/HW_FLOW_CONTROL/SW_FLOW_CONTROL
  @return	return SERIAL_OK for success, on error return error code
 */
/*---------------------------------------------------------------------------*/
int SerialFlowControl(int port, int control)
{
    int fd = FindFD(port);

    if (fd < 0) ///< error
        return fd;

    if (control == NO_FLOW_CONTROL)
    {
        newtio[port].c_cflag &= !CRTSCTS;
        newtio[port].c_iflag &= !(IXON | IXOFF | IXANY);
    }
    else if (control == HW_FLOW_CONTROL)
        newtio[port].c_cflag |= CRTSCTS;
    else if (control == SW_FLOW_CONTROL)
        newtio[port].c_iflag |= (IXON | IXOFF | IXANY);
    else
        return SERIAL_PARAMETER_ERROR;

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio[port]);

    return SERIAL_OK;
}

/*---------------------------------------------------------------------------*/
/**
  @brief	set serial speed and make changes now
  @param	port		port number
  @param	speed		unsigned integer for new speed
  @return	return SERIAL_OK for success, on error return error code
 */
/*---------------------------------------------------------------------------*/
int SerialSetSpeed(int port, unsigned int speed)
{
    int i, table_size = 23;
    int speed_table1[] = {0, 50, 75, 110, 134, 150, 200, 300,
                          600, 1200, 1800, 2400, 4800, 9600,
                          19200, 38400, 57600, 115200}; /*, 230400,
                           460800, 500000, 576000, 921600};*/
    int speed_table2[] = {B0, B50, B75, B110, B134, B150, B200, B300,
                          B600, B1200, B1800, B2400, B4800, B9600,
                          B19200, B38400, B57600, B115200}; /*, B230400,
                           B460800, B500000, B576000, B921600};*/
    int fd = FindFD(port);

    if (fd < 0) ///< error
        return fd;

    for (i = 1; i < table_size; i++) ///< i start from 1, bellow 50 will be set to B0
        if (speed_table1[i] >= speed)
            break;

    cfsetispeed(&newtio[port], speed_table2[i]);
    cfsetospeed(&newtio[port], speed_table2[i]);
    tcsetattr(fd, TCSANOW, &newtio[port]);

    return SERIAL_OK;
}

///*---------------------------------------------------------------------------*/
///**
//  @brief	set serial port mode for RS232/RS422/RS485
//  @param	port		port number
//  @param	mode		serial port mode
//  		{RS232_MODE/RS485_2WIRE_MODE/RS422_MODE/RS485_4WIRE_MODE}
//  @return	return SERIAL_OK for success, on error return error code
// */
///*---------------------------------------------------------------------------*/
// int     SerialSetMode( int port, unsigned int mode)
//{
//         char device[ 80];
//	int ret= 0, fd= FindFD( port);
//	if( fd < 0)			///< error
//	{
//		sprintf( device, "/dev/ttyM%d", port);
//		fd = open( device, O_RDWR|O_NOCTTY);
//		if( fd <0)
//	                return SERIAL_ERROR_OPEN;
//	}
//
//	ret= ioctl( fd, MOXA_SET_OP_MODE, &mode);
//	if( FindFD( port) < 0)
//		close( fd);
//
//	return ret;
// }

/*---------------------------------------------------------------------------*/
/**
  @brief	set serial port parameter
  @param	port		port number
  @param	parity		parity check, 0: none, 1: odd, 2: even, 3: space, 4: mark
  @param	databits	data bits
  @param	stopbit		stop bit
  @return	return fd for success, on error return error code
 */
/*---------------------------------------------------------------------------*/
int SerialSetParam(int port, int databits, int stopbit, int parity)
{
    int fd = FindFD(port);
    if (fd < 0) ///< error
        return fd;

    if (parity == 0)
    {
        newtio[port].c_cflag &= ~PARENB;
        newtio[port].c_iflag &= ~INPCK;
    }
    else if (parity == 1)
    {
        newtio[port].c_cflag |= PARENB;
        newtio[port].c_cflag |= PARODD;
        newtio[port].c_iflag |= INPCK;
    }
    else if (parity == 2)
    {
        newtio[port].c_cflag |= PARENB;
        newtio[port].c_cflag &= ~PARODD;
    }
    else if (parity == 3)
    {
        newtio[port].c_cflag &= ~PARENB;
        newtio[port].c_cflag &= ~CSTOPB;
    }
    else if (parity == 4)
    {
        newtio[port].c_cflag |= CSTOPB;
        newtio[port].c_cflag &= ~PARENB;
        newtio[port].c_iflag &= ~INPCK;
    }

    if (databits == 5)
        newtio[port].c_cflag |= CS5;
    else if (databits == 6)
        newtio[port].c_cflag |= CS6;
    else if (databits == 7)
        newtio[port].c_cflag |= CS7;
    else if (databits == 8)
        newtio[port].c_cflag |= CS8;

    if (stopbit == 1)
        newtio[port].c_cflag &= ~CSTOPB;
    else if (stopbit == 2)
        newtio[port].c_cflag |= CSTOPB;

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio[port]);

    return SERIAL_OK;
}

int SerailSetRts(int port, bool set)
{
    int status;
    int fd = FindFD(port);
    if (fd < 0) ///< error
        return fd;

    ioctl(fd, TIOCMGET, &status);
    if (set)
    {
        status |= TIOCM_RTS;
    }
    else
    {
        status &= ~TIOCM_RTS;
    }
    ioctl(fd, TIOCMSET, &status);
    return 1;
}
