#include "CModbusTcp.h"

// extern
extern ushort_t ModbusDataBuf[MaxData_BufferLen]; // Reg
extern sem_t semModbusTcp;

//
int m_iModbusTcpPort = 0; // the modbus tcp port number
int m_iModbusTcpID = 0;   // the modbus tcp device id
//
CModbusTcp::CModbusTcp(ISocketHandler &h) : TcpSocket(h)
{
}

// rcv data
void CModbusTcp::OnRawData(const char *pcBuf, size_t size)
{
    int iAddr, iNum;
    int iLen;
    char acSendBuf[500];
    int i;

    if (*(pcBuf + 6) != m_iModbusTcpID)
    {
        return;
    }

    for (int i = 0; i < 12; i++)
    {
        acSendBuf[i] = *(pcBuf + i);
    }
    // read Holding Register
    if ((*(pcBuf + 5) == 0x06) && (*(pcBuf + 7) == 0x03))
    {
        iAddr = *(pcBuf + 8);
        iAddr = (iAddr << 8) + *(pcBuf + 9);
        iNum = *(pcBuf + 10);
        iNum = (iNum << 8) + *(pcBuf + 11);
        iLen = jReadDataBuf(&acSendBuf[9], ModbusDataBuf, iAddr, iNum);
        acSendBuf[5] = iLen + 3;
        acSendBuf[8] = iLen;
        iLen += 9;
        SendBuf(acSendBuf, iLen, 0);
        /*printf("=====================");
        printf("CModbusTcp::OnRawDataSendBuf=");
        for(i=0;i<iLen;i++)
        {
            printf(" %2x", acSendBuf[i]);
        }
        printf("=====================\n");*/
    }

    // write single register写AD104通讯命令使用06 写单个寄存器
    if ((*(pcBuf + 5) == 0x06) && (*(pcBuf + 7) == 0x06))
    {
        iAddr = *(pcBuf + 8);
        iAddr = (iAddr << 8) + *(pcBuf + 9);
        jWriteDataBuf(ModbusDataBuf, (char *)(pcBuf + 10), iAddr, 1);
        sem_post(&semModbusTcp);
        acSendBuf[5] = 0x06;
        SendBuf(acSendBuf, 12, 0);
    }

    // write multi register
    if (*(pcBuf + 7) == 0x10)
    {
        iAddr = *(pcBuf + 8);
        iAddr = (iAddr << 8) + *(pcBuf + 9);
        iNum = *(pcBuf + 10);
        iNum = (iNum << 8) + *(pcBuf + 11);
        jWriteDataBuf(ModbusDataBuf, (char *)(pcBuf + 13), iAddr, iNum);
        acSendBuf[5] = 0x06;
        SendBuf(acSendBuf, 12, 0);
    }
}
