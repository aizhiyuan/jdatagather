#include "DataGather.h"
// 串口/dev/ttyS[] 开始
#define Start_Port 0
// 串口数量
#define End_Port 16

// global
#pragma pack(4)
ushort_t ModbusDataBuf[MaxData_BufferLen]; // Reg //short to long
#pragma pack()

// staitc
static int m_iDisLog;
static int m_iDisLogComm;

// 串口参数
static const int m_iCommTimeOut = 200;
static ST_CommPara m_STCommPara[End_Port];

// AD104默认参数
static int fmd;
static int asf;
static int hsm;
static int icr;
static int cof;
static int nov;

// IO稳定时间
static int m_duration;
static int IO_Start;

// 运行参数
static int ReadCount;
static int WTTime;
static int WTDelay;
static int TavValueRange;
static ushort_t Column_SelectStatus;
long TrigeStart_Time, TrigeEnd_Time, TrigeTimes, RunTimes;

// 第一循环写参数标志
static bool firstrun_Initialize[End_Port];
static bool firstrun_Bdr[End_Port];
static bool firstrun_RTav[End_Port];
static bool firstCycle;
static bool IoMsvProcess;
static bool MaterialSignal, StartWait;

// 触发串口信息
static ushort_t IoMsvComEnable;

// 读写MODBUS
static int WTav_Value[16];
static int RBdr_Value[16];
static int RWeight_Value[16];
static int RLdw_Value[16];
static int RLwt_Value[16];
static int RTav_Value[16];
static ushort_t RWeight_Status;
static ushort_t RBdr_Status;
static ushort_t RTav_Status;
static ushort_t RLdw_Status;
static ushort_t RLwt_Status;
static ushort_t RZeroRange_Status;

static bool StickRun;
static bool LogWrite_Enable;
static ushort_t FunctionCode;
static ushort_t FunctionSelect;
static ushort_t ComRun_Status;
static ushort_t MSV200_Status;
static int RunCmd_Code;

/* keep the last run flag */
// alarm
static int ReadFileStatus;
static int WriteSysFileStatus;
static int ReadWriteParaStatus;
static int RunColumnSelStatus;
static int OpColumnSelStatus;
static int ColumnSelStatusTAV;
static int GpIoStatus;
static int RunStatus;
static int SerialOpenStatus[End_Port];
static int SerialWriteStatus[End_Port];
static int SerialReadStatus[End_Port];
static ushort_t AlarmCode1;
static ushort_t AlarmCode2;
static ushort_t AlarmCode3;
static ushort_t AlarmCode4;
static ushort_t AlarmCode5;

// 信号建立
sem_t semModbusTcp;
sem_t semComStart[End_Port];
static int iGPIOFD;

// extern CModbusSocket.c
static int m_iServerEnable;
extern int m_iModbusTcpPort; // the modbus tcp port number
extern int m_iModbusTcpID;   // the modbus tcp device id

unsigned char g_uc_di_status_current;
unsigned char g_uc_do_status_current;
unsigned char g_uc_do_set_value[4];

/* global point, pointing to the defined modbus address */
/* the global point is ulong_t type convert the ushort buf into ulong */

// main
int main(IN int argc, IN char *argv[])
{
    int i;
    int i_indexbase = Int_Initial;
    int i_enable = Int_Initial;
    int i_res = Int_Initial;
    int ai_arg[End_Port];
    char ac_temp[50];
    char log_char[500];
    pthread_t threadModbusTcp;
    pthread_t threadCommOperation;
    pthread_t threadSem;
    pthread_t threadIOMsv;
    pthread_t threadIOMsvOut;
    pthread_t threadIORun;
    pthread_t aThreadSerialCom[End_Port];
    pthread_t threadIOReadWrite;
    void *pvThreadModbusTcpResult = NULL;
    void *pvThreadCommOperationResult = NULL;
    void *pvThreadSemResult = NULL;
    void *pvThreadIOMSVResult = NULL;
    void *pvThreadIOMSVOutResult = NULL;
    void *pvThreadIORunResult = NULL;
    void *apvThreadComResult[End_Port] = {NULL};
    void *pvThreadIOReadWrite = NULL;

    // 开机运行延时
    jPselect(1000);
    try
    {
        /* clear the modbus buffer, this must be done before the init() function */
        /*init */
        jProInit();
        ReadFileStatus = jReadFileInit();
        // 读写IO的线程
        pthread_create(&threadIOReadWrite, NULL, jProcIoReadWrite, NULL);
        GpIoStatus = jGPIoInit();
        jSemCreat();

        // create semaphore thread
        pthread_create(&threadCommOperation, NULL, jProcCommOperation, NULL);
        pthread_create(&threadSem, NULL, jProcModbusCommandJudge, NULL);
        //

        pthread_create(&threadIORun, NULL, jProcloRun, NULL);
        pthread_create(&threadIOMsv, NULL, jProcIoMsv, NULL);
        // pthread_create(&threadIOMsvOut, NULL, jProcIoMsvOut, NULL);

        // create com thread
        for (i = Start_Port; i < End_Port; i++)
        {
            i_enable = m_STCommPara[i].iEnable;
            if (i_enable == 1)
            {
                LOG_INFO("main:create pthread ComServer%d ...\n", i);
                // 运行列反馈
                ComRun_Status = ComRun_Status | (ushort_t)(pow(2, i));
                pthread_create(&aThreadSerialCom[i], NULL, jProcComServer, &m_STCommPara[i]);
            }
        }
        // create modbus tcp thread
        if (m_iServerEnable == 1)
        {
            LOG_INFO("main:create pthread ModbusTcp Server!\n");
            pthread_create(&threadModbusTcp, NULL, jProcModbusTcpServer, NULL);
        }
        // join thread jProcCommOperation
        pthread_join(threadCommOperation, &pvThreadCommOperationResult);
        // join thread semaphore
        pthread_join(threadSem, &pvThreadSemResult);
        // join thread io msv
        pthread_join(threadIORun, &pvThreadIORunResult);
        pthread_join(threadIOMsv, &pvThreadIOMSVResult);
        // pthread_join(threadIOMsvOut, &pvThreadIOMSVOutResult);
        pthread_join(threadIOReadWrite, &pvThreadIOReadWrite);

        // join thread rtu
        for (i = Start_Port; i < End_Port; i++)
        {
            i_enable = m_STCommPara[i].iEnable;
            if (i_enable == 1)
            {
                pthread_join(aThreadSerialCom[i], &apvThreadComResult[i]);
            }
        }
        // join thread tcp
        if (m_iServerEnable == 1)
        {
            pthread_join(threadModbusTcp, &pvThreadModbusTcpResult);
        }
        for (i = Start_Port; i < End_Port; i++)
        {
            sem_destroy(&semComStart[i]);
        }
        sem_destroy(&semModbusTcp);
        LOG_SUCCESS("main:Exit Program Run!\n");
    }
    catch (const Exception &e)
    {
        std::cout << e.ToString() << std::endl;
    }
    return 0;
}
void *jProcCommOperation(IN void *pv)
{
    int i;
    char log_char[500];
    int WTav_addr, RWeight_addr, RTav_addr, RLdw_addr, RLwt_addr, RBdr_addr;
    int Result, Result1;
    int ai_gpio = Int_Initial;
    ushort_t RunStopCode = 0;
    ai_gpio = 5;
    while (1)
    {
        // 运行判断
        if (ModbusDataBuf[PlcWAdr_RunStop] > RunStopCode || ModbusDataBuf[PlcWAdr_RunStop] < RunStopCode)
        {
            RunStopCode = ModbusDataBuf[PlcWAdr_RunStop];
            LOG_SUCCESS("jProcCommOperation:Device receive run cmd is %d!\n", ModbusDataBuf[PlcWAdr_RunStop]);
            switch (ModbusDataBuf[PlcWAdr_RunStop])
            {
            case 101:
                LOG_SUCCESS("jProcCommOperation:Device receive run!\n");
                // jDoSet(ai_gpio);
                // jSet_do_state(0x04, 1);
                g_uc_do_set_value[2] = 1;
                jPselect(20);
                if (jGet_do_state(0x04))
                {
                    LOG_SUCCESS("DO-3 set 1 success!\n");
                }
                else
                {
                    LOG_ERROR("DO-3 set 1 fail!\n");
                }
                StickRun = 1;
                LOG_INFO("Io Start Ok!\n");
                break;
            case 201:
                LOG_ERROR("jProcCommOperation:Device receive stop!\n");
                IoMsvProcess = 0;
                // jDoClear(ai_gpio);
                // jSet_do_state(0x04, 0);
                g_uc_do_set_value[2] = 0;
                jPselect(20);
                if (!jGet_do_state(0x04))
                {
                    LOG_SUCCESS("DO-3 set 0 success!\n");
                }
                else
                {
                    LOG_ERROR("DO-3 set 0 fail!\n");
                }
                StickRun = 0;
                LOG_INFO("Io Stop Ok!\n");
                break;
            case 301:
                LOG_WARN("jProcCommOperation:Device receive reset!\n");
                if (StickRun == 0)
                {
                    jProcReset();
                }
                break;
            default:
                break;
            }
            ModbusDataBuf[PlcWAdr_RunStop] = 0;
        }
        // 功选择1usb记录2写系统文件3写参数4读参数
        FunctionSelect = ModbusDataBuf[PlcWAdr_FunctionSelect];
        // usb记录功能
        LogWrite_Enable = FunctionSelect & (ushort_t)(pow(2, 0));
        if ((FunctionSelect & (ushort_t)(pow(2, 0))))
        {
            ModbusDataBuf[PlcRAdr_FunctionSelect] = (ushort_t)(pow(2, 1));
        }
        // 端口使用写文件
        if ((FunctionSelect & (ushort_t)(pow(2, 1))))
        {
            WriteSysFileStatus = jProcWSysPara();
            ModbusDataBuf[PlcWAdr_FunctionSelect] = FunctionSelect & (0xffff - (ushort_t)(pow(2, 1)));
            ModbusDataBuf[PlcRAdr_FunctionSelect] = (ushort_t)(pow(2, 1));
        }
        // 参数读写

        if ((FunctionSelect & (ushort_t)(pow(2, 2))) || (FunctionSelect & (ushort_t)(pow(2, 3))))
        {
            ReadWriteParaStatus = jProcRWPara();
            if (FunctionSelect & (ushort_t)(pow(2, 2)))
            {
                ModbusDataBuf[PlcRAdr_FunctionSelect] = (ushort_t)(pow(2, 2));
            }
            if (FunctionSelect & (ushort_t)(pow(2, 3)))
            {
                ModbusDataBuf[PlcRAdr_FunctionSelect] = (ushort_t)(pow(2, 3)); // 记录操作
            }
            ModbusDataBuf[PlcWAdr_FunctionSelect] = FunctionSelect & (0xffff - (ushort_t)(pow(2, 2)) - (ushort_t)(pow(2, 3))); // 复位
        }

        // 运行列选择
        if ((FunctionSelect & (ushort_t)(pow(2, 4))))
        {
            RunColumnSelStatus = jProcWColumnSel();
            ModbusDataBuf[PlcWAdr_FunctionSelect] = FunctionSelect & (0xffff - (ushort_t)(pow(2, 4)));
            ModbusDataBuf[PlcRAdr_FunctionSelect] = (ushort_t)(pow(2, 4));
        }
        // 去皮、置零、标定列选择
        if ((FunctionSelect & (ushort_t)(pow(2, 5))))
        {
            OpColumnSelStatus = ModbusDataBuf[PlcWAdr_OpColumnSel];
            ModbusDataBuf[PlcWAdr_FunctionSelect] = FunctionSelect & (0xffff - (ushort_t)(pow(2, 5)));
            ModbusDataBuf[PlcRAdr_FunctionSelect] = (ushort_t)(pow(2, 5));
        }
        if (!StickRun)
        {
            // 写皮重单元
            WTav_addr = PlcWAdr_Column1_LTAV;
            ColumnSelStatusTAV = ModbusDataBuf[PlcWAdr_ColumnEnable_TAV];
            for (i = Start_Port; i < End_Port; i++)
            {
                if (ModbusDataBuf[WTav_addr + i * 2 + 1] >= 0x7fff)
                {
                    WTav_Value[i] = (-1 * (0xffff - ModbusDataBuf[WTav_addr + i * 2 + 1])) * 32767 + (-1 * (0xffff - ModbusDataBuf[WTav_addr + i * 2]));
                }
                else
                {
                    if ((ModbusDataBuf[WTav_addr + i * 2 + 1] == 0) && (ModbusDataBuf[WTav_addr + i * 2] >= 0x7fff))
                    {
                        WTav_Value[i] = -1 * (0xffff - ModbusDataBuf[WTav_addr + i * 2]);
                    }
                    else
                    {
                        WTav_Value[i] = ModbusDataBuf[WTav_addr + i * 2 + 1] * 32767 + ModbusDataBuf[WTav_addr + i * 2];
                    }
                }
            }
        }
        //
        ModbusDataBuf[PlcRAdr_ComRun] = ComRun_Status;
        ModbusDataBuf[PlcRAdr_RunColumnSel] = Column_SelectStatus;
        ModbusDataBuf[PlcRAdr_OpColumnSel] = OpColumnSelStatus;
        if (StickRun)
        {
            ModbusDataBuf[PlcRAdr_RunStopStatus] = 101;
        }
        else
        {
            ModbusDataBuf[PlcRAdr_RunStopStatus] = 201;
        }
        AlarmCode1 = 0;
        AlarmCode2 = 0;
        AlarmCode3 = 0;
        AlarmCode4 = 0;
        AlarmCode5 = 0;
        if (ReadFileStatus < 0)
            AlarmCode1 += (ushort_t)(pow(2, 0));
        if (WriteSysFileStatus < 0)
            AlarmCode1 += (ushort_t)(pow(2, 1));
        if (ReadFileStatus < 0)
            AlarmCode1 += (ushort_t)(pow(2, 2));
        if (ReadWriteParaStatus < 0)
            AlarmCode1 += (ushort_t)(pow(2, 3));
        if (RunColumnSelStatus < 0)
            AlarmCode1 += (ushort_t)(pow(2, 4));
        if (GpIoStatus < 0)
            AlarmCode1 += (ushort_t)(pow(2, 5));
        for (i = Start_Port; i < End_Port; i++)
        {
            if (SerialOpenStatus[i] == Return_OpenSerialError)
                AlarmCode2 += (ushort_t)(pow(2, i));
            if (SerialWriteStatus[i] == Return_SerialCmdError)
                AlarmCode3 += (ushort_t)(pow(2, i));
            if (SerialReadStatus[i] == Return_SerialTimeOut)
                AlarmCode4 += (ushort_t)(pow(2, i));
            if (SerialReadStatus[i] == Return_SerialRecvLenError)
                AlarmCode5 += (ushort_t)(pow(2, i));
        }
        ModbusDataBuf[PlcRAdr_ErrorCode1] = AlarmCode1;
        ModbusDataBuf[PlcRAdr_ErrorCode2] = AlarmCode2;
        ModbusDataBuf[PlcRAdr_ErrorCode3] = AlarmCode3;
        ModbusDataBuf[PlcRAdr_ErrorCode4] = AlarmCode4;
        ModbusDataBuf[PlcRAdr_ErrorCode5] = AlarmCode5;
        if (LogWrite_Enable)
        {
            ModbusDataBuf[PlcRAdr_USBStatus] = ModbusDataBuf[PlcRAdr_USBStatus] || 0x01;
        }
        else
        {
            ModbusDataBuf[PlcRAdr_USBStatus] = ModbusDataBuf[PlcRAdr_USBStatus] && 0xFE;
        }

        if (!StickRun)
        {
            ModbusDataBuf[PlcRAdr_ReadWeightStatus] = RWeight_Status;
            ModbusDataBuf[PlcRAdr_ReadBdrStatus] = RBdr_Status;
            ModbusDataBuf[PlcRAdr_ReadTavStatus] = RTav_Status;
            ModbusDataBuf[PlcRAdr_ReadLdwStatus] = RLdw_Status;
            ModbusDataBuf[PlcRAdr_ReadLwtStatus] = RLwt_Status;
            ModbusDataBuf[PlcRAdr_IDSignal] = m_iModbusTcpID;
            RWeight_addr = PlcRAdr_Column1_LWeight;
            RTav_addr = PlcRAdr_Column1_LTAV;
            RLdw_addr = PlcRAdr_Column1_LLDW;
            RLwt_addr = PlcRAdr_Column1_LLWT;
            RBdr_addr = PlcRAdr_Column1_LBDR;
            for (i = Start_Port; i < End_Port; i++)
            {
                // weight
                ModbusDataBuf[RWeight_addr + i * 2] = RWeight_Value[i] & 0xffff;
                ModbusDataBuf[RWeight_addr + i * 2 + 1] = RWeight_Value[i] >> 16;
                // tav
                ModbusDataBuf[RTav_addr + i * 2] = RTav_Value[i] & 0xffff;
                ModbusDataBuf[RTav_addr + i * 2 + 1] = RTav_Value[i] >> 16;
                // ldw
                ModbusDataBuf[RLdw_addr + i * 2] = RLdw_Value[i] & 0xffff;
                ModbusDataBuf[RLdw_addr + i * 2 + 1] = RLdw_Value[i] >> 16;
                // lwt
                ModbusDataBuf[RLwt_addr + i * 2] = RLwt_Value[i] & 0xffff;
                ModbusDataBuf[RLwt_addr + i * 2 + 1] = RLwt_Value[i] >> 16;
                // bdr
                ModbusDataBuf[RBdr_addr + i * 2] = RBdr_Value[i] & 0xffff;
                ModbusDataBuf[RBdr_addr + i * 2 + 1] = RBdr_Value[i] >> 16;
                ModbusDataBuf[60 + i * 2] = WTav_Value[i] & 0xffff;
                ModbusDataBuf[60 + i * 2 + 1] = WTav_Value[i] >> 16;
            }
        }

        // 开机初始化
        if (firstCycle)
        {
            jPselect(100);
            // 模块判断
            for (i = Start_Port; i < End_Port; i++)
            {
                if ((Column_SelectStatus & (ushort_t)(pow(2, i))) > 0 && m_STCommPara[i].iEnable)
                {
                    firstrun_Bdr[i] = 1;
                    firstrun_Initialize[i] = 0;
                    firstrun_RTav[i] = 0;
                    sem_post(&semComStart[i]);
                }
            }
            jPselect(1200);
            for (i = Start_Port; i < End_Port; i++)
            {
                /*判断是否为使能*/
                if ((Column_SelectStatus & (ushort_t)(pow(2, i))) > 0 && m_STCommPara[i].iEnable)
                {
                    firstrun_Bdr[i] = 0;
                    firstrun_Initialize[i] = 1;
                    firstrun_RTav[i] = 0;
                    sem_post(&semComStart[i]);
                }
            }
            jPselect(300);
            for (i = Start_Port; i < End_Port; i++)
            {
                // 判断是否为使能
                if ((Column_SelectStatus & (ushort_t)(pow(2, i))) > 0 && m_STCommPara[i].iEnable)
                {
                    firstrun_Bdr[i] = 0;
                    firstrun_Initialize[i] = 0;
                    firstrun_RTav[i] = 1;
                    sem_post(&semComStart[i]);
                }
            }
            jPselect(500);
            firstCycle = 0;
        }
    }
    pthread_exit(NULL);
}
// thread modbus tcp server
void *jProcModbusTcpServer(IN void *pv)
{
    char log_char[500];
    try
    {
        SocketHandler ModbusTcpIp;
        ListenSocket<CModbusTcp> ListenModbus(ModbusTcpIp);
        ListenModbus.Bind(m_iModbusTcpPort);
        ModbusTcpIp.Add(&ListenModbus);
        LOG_SUCCESS("jProcModbusTcpServer:create pthread modbus TCP success!\n");
        while (1)
        {
            ModbusTcpIp.Select();
        }
    }
    catch (const Exception &e)
    {
        std::cout << e.ToString() << std::endl;
    }
    pthread_exit(NULL);
}

void *jProcModbusCommandJudge(IN void *pv)
{
    int i;
    char log_char[500];
    while (1)
    {
        LOG_INFO("jProcModbusCommandJudge:Wait semmodbus tcp!\n");
        sem_wait(&semModbusTcp);
        LOG_INFO("jProcModbusCommandJudge:Receive Modbus Command is%d!\n", ModbusDataBuf[PlcWAdr_Command]);
        /* after reading the status, clear */
        if (!StickRun && !firstCycle)
        {
            /*设置串口sem*/
            if ((Cmd_Msv1Tav <= ModbusDataBuf[PlcWAdr_Command]) && (ModbusDataBuf[PlcWAdr_Command] <= Cmd_InitialTav))
            {
                LOG_INFO("jProcModbusCommandJudge:Plc Write Muodule cmd is %d,Channel Enable %d!\n", ModbusDataBuf[PlcWAdr_Command], Column_SelectStatus);
                for (i = Start_Port; i < End_Port; i++)
                {
                    /*判断是否为使能*/

                    if (ModbusDataBuf[PlcWAdr_Command] == Cmd_Ldw || ModbusDataBuf[PlcWAdr_Command] == Cmd_Lwt || ModbusDataBuf[PlcWAdr_Command] == Cmd_Tar)
                    {
                        if ((OpColumnSelStatus & (ushort_t)(pow(2, i))) > 0 && m_STCommPara[i].iEnable)
                        {
                            sem_post(&semComStart[i]);
                        }
                    }
                    else
                    {
                        if (ModbusDataBuf[PlcWAdr_Command] == Cmd_Tav)
                        {
                            if ((ColumnSelStatusTAV & (ushort_t)(pow(2, i))) > 0 && m_STCommPara[i].iEnable)
                            {
                                sem_post(&semComStart[i]);
                            }
                        }
                        else
                        {
                            if (ModbusDataBuf[PlcWAdr_Command] == Cmd_InitialTav)
                            {
                                if (m_STCommPara[i].iEnable)
                                {
                                    sem_post(&semComStart[i]);
                                }
                            }
                            else
                            {
                                if ((Column_SelectStatus & (ushort_t)(pow(2, i))) > 0 && m_STCommPara[i].iEnable)
                                {
                                    sem_post(&semComStart[i]);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    pthread_exit(NULL);
}
// 串口运行
void *jProcComServer(IN void *pv)
{
    struct ST_CommPara commpara = *(struct ST_CommPara *)pv;
    // LOG_INFO("串口运行: SerialPort: %s,Port: %d,Enable: %d,Baud: %d,DataBits: %d,StopBits: %d,Parity: %d,TimeInterVal: %d!\n", commpara.tSerialPort, commpara.iPort, commpara.iEnable, commpara.iBaud, commpara.iDataBits, commpara.iStopBits, commpara.iParity, commpara.iTimeInterVal);
    int i_portnum = Int_Initial;
    int i_rtnval = Int_Initial;
    int i_cmd = Int_Initial;
    int i_RecvLen = Int_Initial;
    int i_MsvNum = Int_Initial;
    int i_SendStatus;
    int i;
    char log_char[500];
    i_portnum = commpara.iPort;
    /* check wheather the port is enabled */
    if (commpara.iEnable == 1)
    {
        // open port, set port
        i_rtnval = SerialOpen(commpara.tSerialPort, i_portnum);
        if (i_rtnval < 0)
        {
            LOG_ERROR("jProcComServer: open serial port(%d) failed(%d)!\n", i_portnum, i_rtnval);
        }
        else
        {
            i_rtnval = SerialSetSpeed(i_portnum, m_STCommPara[i_portnum].iBaud);
            if (i_rtnval < 0)
            {
                LOG_ERROR("jProcComServer:set serial(%d) speed failed(%d)!\n", i_portnum, i_rtnval);
            }
            else
            {
                i_rtnval = SerialSetParam(i_portnum, m_STCommPara[i_portnum].iDataBits, m_STCommPara[i_portnum].iStopBits, m_STCommPara[i_portnum].iParity);
                if (i_rtnval < 0)
                {
                    LOG_ERROR("jProcComServer:set serial(%d) parameters failed(%d)!\n", i_portnum, i_rtnval);
                }
            }
        }
        SerialOpenStatus[i_portnum] = Return_Success;
        if (i_rtnval < 0)
        {
            SerialOpenStatus[i_portnum] = Return_OpenSerialError;
            pthread_exit(NULL);
        }
        LOG_SUCCESS("jProcComServer:Create com %d success!\n", i_portnum);
        while (1)
        {
            i_rtnval = 0;
            sem_wait(&semComStart[i_portnum]);
            if (firstCycle)
            {
                if (firstrun_Bdr[i_portnum])
                {
                    i_cmd = Cmd_RBdrLdwLwt;
                    firstrun_Bdr[i_portnum] = 0;
                }
                if (firstrun_Initialize[i_portnum] & (RBdr_Status & (ushort_t)(pow(2, i))) > 0)
                {
                    i_cmd = Cmd_Initial;
                    firstrun_Initialize[i_portnum] = 0;
                }
                if (firstrun_RTav[i_portnum] & (RBdr_Status & (ushort_t)(pow(2, i))) > 0)
                {
                    i_cmd = Cmd_RTav;
                    firstrun_RTav[i_portnum] = 0;
                }
            }
            else
            {
                if (StickRun)
                {
                    i_cmd = RunCmd_Code;
                }
                else
                {
                    i_cmd = ModbusDataBuf[PlcWAdr_Command];
                }
            }
            i_MsvNum = 90;
            if (ReadCount > 0)
                i_MsvNum = ReadCount;
            LOG_INFO("jProcComServer:Write comport is %d,cmd is%d,msvnum is %d!\n", i_portnum, i_cmd, i_MsvNum);
            i_RecvLen = 0;
            i_SendStatus = 0;
            switch (i_cmd)
            {
            case Cmd_RBdrLdwLwt:
                RBdr_Status = 0;
                RLdw_Status = 0;
                RLwt_Status = 0;
                break;
            case Cmd_RBdr:
                RBdr_Status = 0;
                ;
                break;
            case Cmd_RTav:
            case Cmd_Tav:
            case Cmd_Tar:
            case Cmd_InitialTav:
                RTav_Status = 0;
                break;
            case Cmd_Msv1Tav:
                RWeight_Status = 0;
                RTav_Status = 0;
                break;
            case Cmd_Msv45:
            case Cmd_Msv200:
            case cmd_msv200Test:
            case Cmd_Msvn:
                RWeight_Status = 0;
                break;
            case cmd_msvnTav:
                RWeight_Status = 0;
                RTav_Status = 0;
                RZeroRange_Status = 0;
                break;
            default:
                break;
            }

            i_SendStatus = jProc_SendSerialCommand(i_portnum, (void *)&i_cmd, (void *)&i_MsvNum);
            if (i_SendStatus < 0)
            {
                SerialWriteStatus[i_portnum] = i_SendStatus;
            }
            else
            {
                i_RecvLen = i_SendStatus;
            }
            if (i_RecvLen > 0)
            {
                switch (i_cmd)
                {
                case Cmd_RBdrLdwLwt:
                    jPselect(30);
                    break;
                case Cmd_RTav:
                case Cmd_Tar:
                case Cmd_Tav:
                case Cmd_RBdr:
                case Cmd_Msv1Tav:
                case Cmd_Msv45:
                case Cmd_Msv200:
                case cmd_msv200Test:
                case Cmd_Msvn:
                case cmd_msvnTav:
                case Cmd_InitialTav:
                    jPselect(30);
                    break;
                case Cmd_Tdd1:
                    jPselect(100);
                    break;
                case Cmd_Ldw:
                case Cmd_Lwt:
                    jPselect(4200);
                    break;
                default:
                    break;
                }
                LOG_INFO("jProcComServer:Receive com port is %d , recvlen is %d ,msvnum is %d !\n", i_portnum, i_RecvLen, i_MsvNum);
                SerialReadStatus[i_portnum] = jProc_ReceiveSerialCom(i_portnum, (void *)&i_RecvLen, (void *)&i_MsvNum);
            }
        }
        jPselect(100);
        SerialClose(i_portnum);
        pthread_exit(NULL);
    }
}
int jProc_SendSerialCommand(IN int i_port, IN void *p_data, IN void *msv_len)
{
    int i_portnum = Int_Initial;
    int i_cmd = Int_Initial;
    int i_MsvNum = Int_Initial;
    int ReceiveLenBuff;
    char Com_SendBuf[100];
    char log_char[500];
    int WTav_addr;

    /* keep the incoming parameters into the local vars */
    i_portnum = i_port;
    i_cmd = *(int *)p_data;
    i_MsvNum = *(int *)msv_len;
    /*-----*/
    LOG_INFO("jProc_SendSerialCommand:comport is %d,cmd is%d,msvnum is %d!\n", i_portnum, i_cmd, i_MsvNum);
    /*-----*/
    memset(&Com_SendBuf[0], 0x00, sizeof(Com_SendBuf));
    switch (i_cmd)
    {
    case Cmd_Msv1Tav:
        sprintf(Com_SendBuf, "Msv?1;Tav?;", 11);
        ReceiveLenBuff = 16;
        RWeight_Status = RWeight_Status & (0xffff - (ushort_t)(pow(2, i_portnum)));
        RTav_Status = RTav_Status & (0xffff - (ushort_t)(pow(2, i_portnum)));
        FunctionCode = 1;
        break;
        /* output 45 measured value */
    case Cmd_Msv45:
        sprintf(Com_SendBuf, "Msv?45;", 7);
        ReceiveLenBuff = 182;
        RWeight_Status = RWeight_Status & (0xffff - (ushort_t)(pow(2, i_portnum)));
        break;
        /* output 200 measured value */
    case Cmd_Msv200:
        sprintf(Com_SendBuf, "Msv?200;", 8);
        ReceiveLenBuff = 802;
        MSV200_Status = 0;
        RWeight_Status = RWeight_Status & (0xffff - (ushort_t)(pow(2, i_portnum)));
        break;
    case Cmd_Msvn:
        sprintf(Com_SendBuf, "Msv?%3d;", i_MsvNum, 8);
        ReceiveLenBuff = i_MsvNum * 4 + 2;
        RWeight_Status = RWeight_Status & (0xffff - (ushort_t)(pow(2, i_portnum)));
        break;
    case Cmd_Tav:
        // 写皮重单元结束读皮重
        sprintf(Com_SendBuf, "Tav%7d;Tdd1;Tav?;", WTav_Value[i_portnum], 21);
        ReceiveLenBuff = 16;
        FunctionCode = 2;
        RTav_Status = RTav_Status & (0xffff - (ushort_t)(pow(2, i_portnum)));
        break;
    case Cmd_Ldw:
        sprintf(Com_SendBuf, "dpw\"aed\";spw\"aed\";Ldw;", 26);
        ReceiveLenBuff = Return_NotRecv;
        break;
    case Cmd_Lwt:
        sprintf(Com_SendBuf, "Lwt;tdd1;", 4);
        ReceiveLenBuff = Return_NotRecv;
        break;
    case Cmd_Tar:
        sprintf(Com_SendBuf, "Tar;Tdd1;Tav?;", 14);
        ReceiveLenBuff = 16;
        FunctionCode = 2;
        RTav_Status = RTav_Status & (0xffff - (ushort_t)(pow(2, i_portnum)));
        break;
    case Cmd_Tdd1:
        sprintf(Com_SendBuf, "Tdd1;", 5);
        ReceiveLenBuff = Return_NotRecv;
        break;
    case Cmd_RTav:
        sprintf(Com_SendBuf, "Tav?;", 5);
        ReceiveLenBuff = 10;
        RTav_Status = RTav_Status & (0xffff - (ushort_t)(pow(2, i_portnum)));
        break;
    case Cmd_RBdrLdwLwt:
        sprintf(Com_SendBuf, "bdr?;Ldw?;Lwt?;", 15);
        ReceiveLenBuff = 29;
        RBdr_Status = RBdr_Status & (0xffff - (ushort_t)(pow(2, i_portnum)));
        RLdw_Status = RLdw_Status & (0xffff - (ushort_t)(pow(2, i_portnum)));
        RLwt_Status = RLwt_Status & (0xffff - (ushort_t)(pow(2, i_portnum)));
        break;
    case Cmd_Initial:
        sprintf(Com_SendBuf, "dpw\"aed\";spw\"aed\";Fmd%1d;Asf%1d;Hsm%1d;Icr%1d;Cof%1d;Nov%7d;Tdd1;", fmd, asf, hsm, icr, cof, nov, 59);
        ReceiveLenBuff = Return_NotRecv;
        break;
    case cmd_msv200Test:
        sprintf(Com_SendBuf, "Msv?200;", 8);
        ReceiveLenBuff = 802;
        MSV200_Status = 1;
        RWeight_Status = RWeight_Status & (0xffff - (ushort_t)(pow(2, i_portnum)));
        break;
    case cmd_msvnTav:
        sprintf(Com_SendBuf, "Msv?%3d;Tav?;", i_MsvNum, 13);
        ReceiveLenBuff = i_MsvNum * 4 + 2 + 10;
        RWeight_Status = RWeight_Status & (0xffff - (ushort_t)(pow(2, i_portnum)));
        RTav_Status = RTav_Status & (0xffff - (ushort_t)(pow(2, i_portnum)));
        RZeroRange_Status = RZeroRange_Status & (0xffff - (ushort_t)(pow(2, i_portnum)));
        break;
    case Cmd_RBdr:
        sprintf(Com_SendBuf, "bdr?;", 5);
        ReceiveLenBuff = 9;
        RBdr_Status = RBdr_Status & (0xffff - (ushort_t)(pow(2, i_portnum)));
        break;
    case Cmd_InitialTav:
        // 写皮重单元结束读皮重
        sprintf(Com_SendBuf, "Tav0000000;Tdd1;TAV?;", 16);
        ReceiveLenBuff = 16;
        FunctionCode = 2;
        break;
    default:
    {
        LOG_ERROR("jProc_SendSerialCommand:AD104 Command Error com port is %d,cmd is %d!\n", i_portnum, i_cmd);
        return Return_SerialCmdError;
        break;
    }
    }
    SerialFlush(i_portnum);
    SerialWrite(i_portnum, Com_SendBuf, strlen(Com_SendBuf));
    /* dbg info */
    LOG_INFO("jProc_SendSerialCommand:serial(%d) write, cmd is:%s ,need receive len is%d!\n", i_portnum, Com_SendBuf, ReceiveLenBuff);
    return ReceiveLenBuff;
}
int jProc_ReceiveSerialCom(IN int i_port, void *p_data, IN void *msv_len)
{
    int i_portnum = Int_Initial;
    int i_NeedRecvLen = Int_Initial;
    int i_RecvDataLen = Int_Initial;
    int i_Readtimes = Int_Initial;
    int i_ReadLen = Int_Initial;
    int i_Offset = Int_Initial;
    int i_MsvNum = Int_Initial;
    int i, j, m, n;
    int x, y, z;
    long Sum, Data[400], DataMax[40], DataMin[40];
    uchar_t RecBuffer[Serial_MaxLen];
    uchar_t TAvRecBuffer[15];
    char Com_SendBuf[100];
    char log_char[500];
    char Cmd_name[20];
    int Cmd_Value;
    int cmd_Status;
    int Cmd_Value1;
    int cmd_Status1;
    int Cmd_Value2;
    int cmd_Status2;
    bool TAV_ValueTrue[End_Port];
    i_portnum = i_port;
    i_NeedRecvLen = *(int *)(p_data);
    i_MsvNum = *(int *)msv_len;
    memset(&RecBuffer[0], 0x00, sizeof(RecBuffer));
    memset(&TAvRecBuffer[0], 0x00, sizeof(TAvRecBuffer));
    memset(&Com_SendBuf[0], 0x00, sizeof(Com_SendBuf));
    if (i_NeedRecvLen > 0)
    {
        i_Offset = 0;
        i_Readtimes = i_NeedRecvLen / Serial_ReadMaxLen;
        if ((i_NeedRecvLen % Serial_ReadMaxLen) > 0)
        {
            i_Readtimes = i_Readtimes + 1;
        }
        for (i = 0; i < i_Readtimes; i++)
        {
            i_ReadLen = SerialRead(i_portnum, RecBuffer + i_Offset, Serial_ReadMaxLen, Serial_Read_WaitTime, Serial_Read_TimeOut);
            i_Offset += i_ReadLen;
            if (i_ReadLen == 0)
            {
                LOG_INFO("jProc_ReceiveSerialCom:Serial Port(%d) read time out%d!\n", i_portnum, i);
                return Return_SerialTimeOut;
            }
        }
        i_RecvDataLen = i_Offset;
        if (i_NeedRecvLen == i_RecvDataLen)
        {
            LOG_INFO("jProc_ReceiveSerialCom:Serial Port(%d)receive data len is (%d) true!\n", i_portnum, i_RecvDataLen);
            Cmd_Value1 = 0;
            cmd_Status1 = 0;
            Cmd_Value2 = 0;
            cmd_Status2 = 0;
            switch (i_RecvDataLen)
            {
            case 3:
                break;
            case 6:
                RWeight_Value[i_portnum] = RecBuffer[0] * 65536 + RecBuffer[1] * 256 + RecBuffer[2];
                if (RWeight_Value[i_portnum] >= 0x7fffff)
                    RWeight_Value[i_portnum] = 1 - (0xffffff - RWeight_Value[i_portnum]);
                RWeight_Status = RWeight_Status | (ushort_t)(pow(2, i_portnum));
                sprintf(Cmd_name, "msv1", 4);
                Cmd_Value = RWeight_Value[i_portnum];
                cmd_Status = RWeight_Status;
                break;
            case 9:
                RBdr_Value[i_portnum] = (RecBuffer[0] - 0x30) * 10000 + (RecBuffer[1] - 0x30) * 1000 + (RecBuffer[2] - 0x30) * 100 + (RecBuffer[3] - 0x30) * 10 + (RecBuffer[4] - 0x30);
                RBdr_Status = RBdr_Status | (ushort_t)(pow(2, i_portnum));
                sprintf(Cmd_name, "bdr", 3);
                Cmd_Value = RBdr_Value[i_portnum];
                cmd_Status = RBdr_Status;
                break;
            case 29:
                RBdr_Value[i_portnum] = (RecBuffer[0] - 0x30) * 10000 + (RecBuffer[1] - 0x30) * 1000 + (RecBuffer[2] - 0x30) * 100 + (RecBuffer[3] - 0x30) * 10 + (RecBuffer[4] - 0x30);
                RBdr_Status = RBdr_Status | (ushort_t)(pow(2, i_portnum));
                sprintf(Cmd_name, "bdr;ldw;lwt;", 12);
                Cmd_Value = RBdr_Value[i_portnum];
                cmd_Status = RBdr_Status;
                // ldw零点值
                m = 0;
                n = 0;
                j = 16;
                for (j = 16; j > 9; j--)
                {
                    m += (RecBuffer[j] - 0x30) * (int)(pow(10, n));
                    n++;
                }
                if (RecBuffer[9] == 0x2d)
                    m = m * (-1);
                RLdw_Value[i_portnum] = m;
                RLdw_Status = RLdw_Status | (ushort_t)(pow(2, i_portnum));
                Cmd_Value1 = RLdw_Value[i_portnum];
                cmd_Status1 = RLdw_Status;
                // lwt标定值
                m = 0;
                n = 0;
                j = 26;
                for (j = 26; j > 19; j--)
                {
                    m += (RecBuffer[j] - 0x30) * (int)(pow(10, n));
                    n++;
                }
                if (RecBuffer[19] == 0x2d)
                    m = m * (-1);
                RLwt_Value[i_portnum] = m;
                RLwt_Status = RLwt_Status | (ushort_t)(pow(2, i_portnum));
                Cmd_Value2 = RLwt_Value[i_portnum];
                cmd_Status2 = RLwt_Status;
                break;
            case 10:
                m = 0;
                n = 0;
                j = 0;
                for (j = 7; j > 0; j--)
                {
                    m += (RecBuffer[j] - 0x30) * (int)(pow(10, n));
                    n++;
                }
                if (RecBuffer[0] == 0x2d)
                    m = m * (-1);
                // 皮重值
                RTav_Value[i_portnum] = m;
                RTav_Status = RTav_Status | (ushort_t)(pow(2, i_portnum));
                sprintf(Cmd_name, "Tav", 3);
                Cmd_Value = RTav_Value[i_portnum];
                cmd_Status = RTav_Status;
                break;
            case 16:
                if (FunctionCode == 1)
                {
                    RWeight_Value[i_portnum] = RecBuffer[0] * 65536 + RecBuffer[1] * 256 + RecBuffer[2];
                    if (RWeight_Value[i_portnum] >= 0x7fffff)
                        RWeight_Value[i_portnum] = 1 - (0xffffff - RWeight_Value[i_portnum]);
                    RWeight_Status = RWeight_Status | (ushort_t)(pow(2, i_portnum));
                    sprintf(Cmd_name, "msv1?TAV?", 9);
                    Cmd_Value = RWeight_Value[i_portnum];
                    cmd_Status = RWeight_Status;
                    m = 0;
                    n = 0;
                    j = 0;
                    for (j = 13; j > 6; j--)
                    {
                        m += (RecBuffer[j] - 0x30) * (int)(pow(10, n));
                        n++;
                    }
                    if (RecBuffer[6] == 0x2d)
                        m = m * (-1);
                    // 皮重值
                    RTav_Value[i_portnum] = m;
                    RTav_Status = RTav_Status | (ushort_t)(pow(2, i_portnum));
                    Cmd_Value1 = RTav_Value[i_portnum];
                    cmd_Status1 = RTav_Status;
                }
                else if (FunctionCode == 2)
                {
                    m = 0;
                    n = 0;
                    j = 0;
                    for (j = 13; j > 6; j--)
                    {
                        m += (RecBuffer[j] - 0x30) * (int)(pow(10, n));
                        n++;
                    }
                    if (RecBuffer[6] == 0x2d)
                        m = m * (-1);
                    // 皮重值
                    RTav_Value[i_portnum] = m;
                    RTav_Status = RTav_Status | (ushort_t)(pow(2, i_portnum));
                    sprintf(Cmd_name, "Tav", 3);
                    Cmd_Value = RTav_Value[i_portnum];
                    cmd_Status = RTav_Status;
                }
                break;
            case 182: //
                Sum = 0;
                RWeight_Status = RWeight_Status | (ushort_t)(pow(2, i_portnum));
                for (x = 0; x < 45; x++)
                {
                    Data[x] = 65536 * (long)RecBuffer[4 * x] + 256 * (long)RecBuffer[4 * x + 1] + (long)RecBuffer[4 * x + 2];
                    if (Data[x] >= 0x7fffff)
                        Data[x] = 1 - (0xffffff - Data[x]);
                    Sum += Data[x];
                }
                RWeight_Value[i_portnum] = (int)(Sum / 45);
                sprintf(Cmd_name, "msv45", 5);
                Cmd_Value = RWeight_Value[i_portnum];
                cmd_Status = RWeight_Status;
                break;
            case 802: // 去皮使用
                Sum = 0;
                RWeight_Status = RWeight_Status | (ushort_t)(pow(2, i_portnum));
                for (x = 0; x < 200; x++)
                {
                    Data[x] = 65536 * (long)RecBuffer[4 * x] + 256 * (long)RecBuffer[4 * x + 1] + (long)RecBuffer[4 * x + 2];
                    if (Data[x] >= 0x7fffff)
                        Data[x] = 1 - (0xffffff - Data[x]);
                    Sum += Data[x];
                    if (x == 0)
                    {
                        DataMax[i_portnum] = Data[x];
                        DataMin[i_portnum] = Data[x];
                    }
                    if (Data[x] > DataMax[i_portnum])
                        DataMax[i_portnum] = Data[x];
                    if (Data[x] < DataMin[i_portnum])
                        DataMin[i_portnum] = Data[x];
                }
                if (MSV200_Status == 0)
                {
                    RWeight_Value[i_portnum] = (int)(Sum / 200);
                    sprintf(Cmd_name, "msv200", 6);
                }
                if (MSV200_Status == 1)
                {
                    RWeight_Value[i_portnum] = DataMax[i_portnum] - DataMin[i_portnum];
                    sprintf(Cmd_name, "msv200Test", 6);
                }
                Cmd_Value = RWeight_Value[i_portnum];
                cmd_Status = RWeight_Status;
                break;
            default:
                // msv触发称重
                if (i_RecvDataLen == (i_MsvNum * 4 + 2))
                {
                    RWeight_Status = RWeight_Status | (ushort_t)(pow(2, i_portnum));
                    for (x = 0; x < i_MsvNum; x++)
                    {
                        Data[x] = 65536 * (long)RecBuffer[4 * x] + 256 * (long)RecBuffer[4 * x + 1] + (long)RecBuffer[4 * x + 2];
                        if (Data[x] >= 0x7fffff)
                            Data[x] = 1 - (0xffffff - Data[x]);
                    }
                    Sum = 0;
                    for (j = (int)(WTDelay / 5); j < i_MsvNum; j++)
                    {
                        Sum += Data[j];
                    }
                    RWeight_Value[i_portnum] = (int)(Sum / (long)(i_MsvNum - (int)(WTDelay / 5)));
                    Cmd_Value = RWeight_Value[i_portnum];
                    cmd_Status = RWeight_Status;
                    sprintf(Cmd_name, "msv%3d", i_MsvNum, 6);
                    RZeroRange_Status = 0;
                }
                else
                {
                    // 无物料重量进行皮重处理
                    // 重量处理
                    if (i_RecvDataLen == (i_MsvNum * 4 + 2 + 10))
                    {
                        RWeight_Status = RWeight_Status | (ushort_t)(pow(2, i_portnum));
                        for (x = 0; x < i_MsvNum; x++)
                        {
                            Data[x] = 65536 * (long)RecBuffer[4 * x] + 256 * (long)RecBuffer[4 * x + 1] + (long)RecBuffer[4 * x + 2];
                            if (Data[x] >= 0x7fffff)
                                Data[x] = 1 - (0xffffff - Data[x]);
                        }
                        Sum = 0;
                        for (j = (int)(WTDelay / 5); j < i_MsvNum; j++)
                        {
                            Sum += Data[j];
                        }
                        RWeight_Value[i_portnum] = (int)(Sum / (long)(i_MsvNum - (int)(WTDelay / 5)));
                        Cmd_Value = RWeight_Value[i_portnum];
                        cmd_Status = RWeight_Status;
                        sprintf(Cmd_name, "msv%3d", i_MsvNum, 6);
                        LOG_INFO("jProc_ReceiveSerialCom:Serial Port(%d) Cmd is %s, value is (%d) ,status(%d)!\n", i_portnum, Cmd_name, Cmd_Value, cmd_Status);
                        j = 0;
                        n = 0;
                        y = i_MsvNum * 4 + 2;
                        z = i_MsvNum * 4 + 2 + 9;
                        for (j = y; j <= z; j++)
                        {
                            TAvRecBuffer[n] = RecBuffer[j];
                            n++;
                        }
                        m = 0;
                        n = 0;
                        j = 0;
                        for (j = 7; j > 0; j--)
                        {
                            m += (TAvRecBuffer[j] - 0x30) * (int)(pow(10, n));
                            n++;
                        }
                        if (TAvRecBuffer[0] == 0x2d)
                            m = m * (-1);
                        // 皮重值
                        RTav_Value[i_portnum] = m;
                        RTav_Status = RTav_Status | (ushort_t)(pow(2, i_portnum));
                        sprintf(Cmd_name, "Tav", 3);
                        Cmd_Value = RTav_Value[i_portnum];
                        cmd_Status = RTav_Status;
                        // 皮重判断是否异常
                        TAV_ValueTrue[i_portnum] = 1;
                        if ((RWeight_Value[i_portnum] > TavValueRange) || (RWeight_Value[i_portnum] < ((-1) * TavValueRange)))
                        {
                            RZeroRange_Status = RZeroRange_Status | (ushort_t)(pow(2, i_portnum));
                            TAV_ValueTrue[i_portnum] = 0;
                        }
                        if (TAV_ValueTrue[i_portnum])
                        {
                            // 皮重值计算
                            WTav_Value[i_portnum] = RWeight_Value[i_portnum] + RTav_Value[i_portnum];
                            // 写皮重值
                            sprintf(Com_SendBuf, "Tav%7d;Tdd1;", WTav_Value[i_portnum], 16);
                            SerialFlush(i_portnum);
                            SerialWrite(i_portnum, Com_SendBuf, strlen(Com_SendBuf));
                            LOG_INFO("jProc_ReceiveSerialCom:serial(%d) write, cmd is:%s,value(%d)!\n", i_portnum, Com_SendBuf, WTav_Value[i_portnum]);
                            TAV_ValueTrue[i_portnum] = 0;
                        }
                        // 皮重值反馈到上位
                        RTav_Value[i_portnum] = WTav_Value[i_portnum];
                    }
                    else
                    {
                        sprintf(Cmd_name, "Error", 5);
                    }
                }
                break;
            }
            LOG_INFO("jProc_ReceiveSerialCom:Serial Port(%d) Cmd is %s, value is (%d) ,status is (%d),value1 is (%d) ,status1 is (%d), value2 is (%d) ,status2 is (%d)\n", i_portnum, Cmd_name, Cmd_Value, cmd_Status, Cmd_Value1, cmd_Status1, Cmd_Value2, cmd_Status2);
            return Return_Success;
        }
        else
        {
            LOG_ERROR("jProc_ReceiveSerialCom:Serial Port(%d)receive data len is (%d) error!\n", i_portnum, i_RecvDataLen);
            LOG_ERROR("jProc_ReceiveSerialCom:Serial Port(%d)receive data  is (%s) !\n", i_portnum, RecBuffer);
            return Return_SerialRecvLenError;
        }
    }
}
void *jProcloRun(IN void *pv)
{
    int ai_gpio[4] = {Int_Initial, Int_Initial, Int_Initial, Int_Initial};
    ai_gpio[0] = 0;
    ai_gpio[1] = 6;
    ai_gpio[2] = 5;
    ai_gpio[3] = 3;

    while (1)
    {
        // 运行停止
        if (StickRun)
        {
            ModbusDataBuf[PlcRAdr_USBStatus] = ModbusDataBuf[PlcRAdr_USBStatus] || 0x02;

            if (IoMsvProcess)
            {
                ModbusDataBuf[PlcRAdr_USBStatus] = ModbusDataBuf[PlcRAdr_USBStatus] || 0x04;
            }
            else
            {
                ModbusDataBuf[PlcRAdr_USBStatus] = ModbusDataBuf[PlcRAdr_USBStatus] && 0xFB;
            }

            if (MaterialSignal)
            {
                ModbusDataBuf[PlcRAdr_USBStatus] = ModbusDataBuf[PlcRAdr_USBStatus] || 0x08;
            }
            else
            {
                ModbusDataBuf[PlcRAdr_USBStatus] = ModbusDataBuf[PlcRAdr_USBStatus] && 0xF7;
            }

            if (jGet_do_state(0x02))
            {
                ModbusDataBuf[PlcRAdr_USBStatus] = ModbusDataBuf[PlcRAdr_USBStatus] || 0x10;
            }
            else
            {
                ModbusDataBuf[PlcRAdr_USBStatus] = ModbusDataBuf[PlcRAdr_USBStatus] && 0xEF;
            }
        }
        else
        {
            ModbusDataBuf[PlcRAdr_USBStatus] = ModbusDataBuf[PlcRAdr_USBStatus] && 0x01;
        }
        if (IO_Start)
        {
            if (jGet_di_state(0x01) && !StickRun)
            {
                StickRun = 1;
                LOG_INFO("jProcloRun:Device receive run,StickRun is 1!\n");
            }
            if (!jGet_di_state(0x01) && StickRun)
            {
                IoMsvProcess = 0;
                StickRun = 0;
                LOG_INFO("jProcloRun:Device receive stop,,StickRun is 0!\n");
            }
            // 复位
            if (jGet_di_state(0x08))
            {
                if (StickRun == 0)
                {
                    jProcReset();
                    LOG_INFO("jProcloRun:Device receive reset!\n");
                }
            }
        }

        //
        if (firstCycle)
        {
            // jDoClear(1);
            // jSet_do_state(0x01, 0);
            g_uc_do_set_value[0] = 0;
            jPselect(20);
            if (!jGet_do_state(0x01))
            {
                LOG_SUCCESS("DO-1 set 0 success!\n");
            }
            else
            {
                LOG_ERROR("DO-1 set 0 fail!\n");
            }
        }
        else
        {
            // jDoSet(1);
            // jSet_do_state(0x01, 1);
            g_uc_do_set_value[0] = 1;
            jPselect(20);
            if (jGet_do_state(0x01))
            {
                LOG_SUCCESS("DO-1 set 1 success!\n");
            }
            else
            {
                LOG_ERROR("DO-1 set 1 fail!\n");
            }
            // 输出运行信号
            if (StickRun)
            {
                if (!(jGet_do_state(0x04)) && StickRun)
                {
                    LOG_INFO("jProcloRunOut:run out!\n");
                }
                // jDoSet(5);
                // jSet_do_state(0x04, 1);
                g_uc_do_set_value[2] = 1;
                jPselect(20);
                if (jGet_do_state(0x01))
                {
                    LOG_SUCCESS("DO-3 set 1 success!\n");
                }
                else
                {
                    LOG_ERROR("DO-3 set 1 fail!\n");
                }
            }
            else
            {
                if (jGet_do_state(0x04) && !StickRun)
                {
                    LOG_INFO("jProcloRunOut:stop out!\n");
                }
                // jDoClear(5);
                // jSet_do_state(0x04, 0);
                g_uc_do_set_value[2] = 0;
                jPselect(20);
                if (!jGet_do_state(0x04))
                {
                    LOG_SUCCESS("DO-3 set 0 success!\n");
                }
                else
                {
                    LOG_ERROR("DO-3 set 0 fail!\n");
                }
            }
        }
    }
    pthread_exit(NULL);
}
// 触发IO
void *jProcIoMsv(IN void *pv)
{
    bool OutSignle = 0;
    int RWeight_addr, i, RTav_addr;
    time_t oil_time, new_time;
    int i_status = 0;
    while (1)
    {
        if (StickRun)
        {
            //  if (jDiCheckPositive(ai_gpio[0], 0, 0) == Return_HighLevel && !IoMsvProcess)
            int di_2_value = jGet_di_state(0x02);
            // LOG_INFO("DO-2 value %d \n", di_2_value);
            if (di_2_value)
            {
                LOG_INFO("jProcIoMsv:Stick Cycle Signal!\n");
                IoMsvComEnable = 0;
                RWeight_Status = 0;
                ModbusDataBuf[PlcRAdr_ReadWeightStatus] = 0;

                TrigeStart_Time = jGetTick();
                TrigeTimes = TrigeTimes + 1;
                i_status = 1;
                // IoMsvProcess = 1;
                // 物料信号
                int di_4_value = jGet_di_state(0x04);
                // LOG_INFO("DO-4 value %d \n", di_4_value);
                if (di_4_value)
                {
                    MaterialSignal = 1;
                }
                else
                {
                    MaterialSignal = 0;
                }
                for (i = Start_Port; i < End_Port; i++)
                {
                    /*判断是否为使能*/
                    if ((Column_SelectStatus & (ushort_t)(pow(2, i))) > 0 && m_STCommPara[i].iEnable)
                    {
                        if (MaterialSignal)
                        {
                            RunCmd_Code = Cmd_Msvn;
                        }
                        else
                        {
                            RunCmd_Code = cmd_msvnTav;
                        }
                        sem_post(&semComStart[i]);
                        IoMsvComEnable += (ushort_t)(pow(2, i));
                    }
                }
            }
            if (i_status)
            {
                jPselect(5);
                // IoMsvProcess = 0;
                StartWait = 1;
                oil_time = timestamp();
                while (StartWait)
                {
                    new_time = timestamp();
                    if ((new_time - oil_time) >= 1000)
                    {
                        IoMsvComEnable = 0;
                        RWeight_Status = 0;
                        int di_4_value = jGet_di_state(0x04);
                        // LOG_INFO("DO-4 value %d \n", di_4_value);
                        if (di_4_value)
                        {
                            MaterialSignal = 1;
                        }
                        else
                        {
                            MaterialSignal = 0;
                        }
                        // OutSignle = 1;
                        for (i = Start_Port; i < End_Port; i++)
                        {
                            /*判断是否为使能*/
                            if ((Column_SelectStatus & (ushort_t)(pow(2, i))) > 0 && m_STCommPara[i].iEnable)
                            {
                                if (MaterialSignal)
                                {
                                    RunCmd_Code = Cmd_Msvn;
                                }
                                else
                                {
                                    RunCmd_Code = cmd_msvnTav;
                                }
                                sem_post(&semComStart[i]);
                                IoMsvComEnable += (ushort_t)(pow(2, i));
                            }
                        }
                        oil_time = timestamp();
                    }
                    if ((IoMsvComEnable == RWeight_Status))
                    {
                        // if (IoMsvProcess)
                        // {
                        TrigeEnd_Time = jGetTick();
                        ModbusDataBuf[80] = TrigeEnd_Time - TrigeStart_Time;
                        RunTimes = RunTimes + 1;
                        ModbusDataBuf[81] = TrigeTimes;
                        ModbusDataBuf[82] = RunTimes;
                        ModbusDataBuf[99] = ModbusDataBuf[98];
                        ModbusDataBuf[98] = ModbusDataBuf[97];
                        ModbusDataBuf[97] = ModbusDataBuf[96];
                        ModbusDataBuf[96] = ModbusDataBuf[95];
                        ModbusDataBuf[95] = ModbusDataBuf[94];
                        ModbusDataBuf[94] = ModbusDataBuf[93];
                        ModbusDataBuf[93] = ModbusDataBuf[92];
                        ModbusDataBuf[92] = ModbusDataBuf[91];
                        ModbusDataBuf[91] = ModbusDataBuf[90];
                        ModbusDataBuf[90] = ModbusDataBuf[80];
                        // }
                        if (StickRun)
                        {
                            ModbusDataBuf[PlcRAdr_ReadWeightStatus] = RWeight_Status;
                            RWeight_addr = PlcRAdr_Column1_LWeight;
                            ModbusDataBuf[PlcRAdr_ReadTavStatus] = RTav_Status;
                            ModbusDataBuf[PlcRAdr_ZeroRangeStatus] = RZeroRange_Status;
                            RTav_addr = PlcRAdr_Column1_LTAV;
                            for (i = Start_Port; i < End_Port; i++)
                            {
                                // weight
                                ModbusDataBuf[RWeight_addr + i * 2] = RWeight_Value[i] & 0xffff;
                                ModbusDataBuf[RWeight_addr + i * 2 + 1] = RWeight_Value[i] >> 16;
                                // tav
                                ModbusDataBuf[RTav_addr + i * 2] = RTav_Value[i] & 0xffff;
                                ModbusDataBuf[RTav_addr + i * 2 + 1] = RTav_Value[i] >> 16;
                            }
                        }
                        // jDoSet(3);
                        // jSet_do_state(0x02, 1);
                        g_uc_do_set_value[1] = 1;
                        jPselect(20);
                        int do_2_value = jGet_do_state(0x02);
                        // LOG_INFO("DO-2 value %d \n", do_2_value);
                        if (do_2_value)
                        {
                            LOG_SUCCESS("DO-2 set 1 success!\n");
                        }
                        else
                        {
                            LOG_ERROR("DO-2 set 1 fail!\n");
                        }
                        jPselect(200);
                        // jPselect(100);
                        // jSet_do_state(0x02, 0);
                        g_uc_do_set_value[1] = 0;
                        jPselect(20);
                        do_2_value = jGet_do_state(0x02);
                        // LOG_INFO("DO-2 value %d \n", do_2_value);
                        if (!do_2_value)
                        {
                            LOG_SUCCESS("DO-2 set 0 success!\n");
                        }
                        else
                        {
                            LOG_ERROR("DO-2 set 0 fail!\n");
                        }
                        StartWait = 0;
                    }

                    jPselect(5);
                }
            }
        }
        else
        {
            jPselect(5);
        }
    }
    pthread_exit(NULL);
}

// void *jProcIoMsvOut(void *pv)
// {
//     // 反馈信号
//     bool OutSignle = 0;
//     int RWeight_addr, i, RTav_addr;
//     while (1)
//     {
//     }
//     pthread_exit(NULL);
// }

int jProcWSysPara(void)
{
    int i, j, k;
    ushort_t Com_EnableSysStatus;
    char log_char[500];
    char ac_temp[20];
    // rude::Config config;
    Com_EnableSysStatus = 0;
    xini_file_t ini(ConfigFile);
    // if (config.load(ConfigFile))
    // {
    for (i = Start_Port; i < End_Port; i++)
    {
        memset(&ac_temp[0], 0x00, sizeof(ac_temp));
        sprintf(&ac_temp[0], "com%d Setting", i + 1);
        //
        // 系统参数地址
        j = 0;
        // if(config.setSection(&ac_temp[0]))
        // {

        Com_EnableSysStatus = ModbusDataBuf[PlcWAdr_Com_Enable];
        if (Com_EnableSysStatus & (ushort_t)(pow(2, i)))
        {
            ini[&ac_temp[0]]["Enable"] = 1;
            // config.setIntValue("Enable",1);
            j = 1;
        }
        else
        {
            ini[&ac_temp[0]]["Enable"] = 0;
            // config.setIntValue("Enable",0);
            j = 0;
        }
        sprintf(ac_temp, "Write", 5);

        // 打印-----------
        LOG_INFO("jProcRWSysPara: %s m_STCommPara No %d, Enable%d!\n", ac_temp, i, j);

        // 打印-----------
    }
    return Return_Success;
}
//
int jProcRWPara(void)
{
    int i, j;
    char log_char[500];
    char ac_temp[20];
    xini_file_t ini(ConfigFile);

    j = 0;

    if (FunctionSelect & (ushort_t)(pow(2, 2)))
    {
        ReadCount = (int)ModbusDataBuf[PlcWAdr_ReadCount];
        WTDelay = (int)ModbusDataBuf[PlcWAdr_WTDealy];
        WTTime = (int)ModbusDataBuf[PlcWAdr_WTTime];
        TavValueRange = (int)ModbusDataBuf[PlcWAdr_TavValueRange];

        ini["Para"]["ReadCount"] = ReadCount;
        ini["Para"]["WTDelay"] = WTDelay;
        ini["Para"]["WTTime"] = WTTime;
        ini["Para"]["TavValueRange"] = TavValueRange;
        sprintf(ac_temp, "Write", 5);
    }
    if (FunctionSelect & (ushort_t)(pow(2, 3)))
    {
        // ad104 para
        ReadCount = ini["Para"]["ReadCount"];
        WTTime = ini["Para"]["WTDelay"];
        WTDelay = ini["Para"]["WTTime"];
        TavValueRange = ini["Para"]["TavValueRange"];
        ModbusDataBuf[PlcRAdr_ReadCount] = (ushort_t)ReadCount;
        ModbusDataBuf[PlcRAdr_WTDealy] = (ushort_t)WTDelay;
        ModbusDataBuf[PlcRAdr_WTTime] = (ushort_t)WTTime;
        ModbusDataBuf[PlcRAdr_TavValueRange] = (int)TavValueRange;
        sprintf(ac_temp, "read", 4);
    }
    // 打印-----------
    LOG_INFO("jProcRWPara: %s,Para:%d,%d,%d,%d!\n", ac_temp, ReadCount, WTDelay, WTTime, TavValueRange);
    return Return_Success;
    // 打印-----------
}

int jProcWColumnSel(void)
{
    int i, j;
    char log_char[500];
    char ac_temp[20];

    xini_file_t ini(ConfigFile);

    Column_SelectStatus = (int)ModbusDataBuf[PlcWAdr_RunColumnSel];
    ini["Column Sel"]["Column_SelectStatus"] = Column_SelectStatus;
    sprintf(ac_temp, "Write", 5);

    // 打印-----------
    LOG_INFO("jProcWColumnSel:%s Column Sel:%d!\n", ac_temp, Column_SelectStatus);
    return Return_Success;

    // 打印-----------
}
// read buffer data
int jReadDataBuf(char *pcRet, ushort_t *pusBuffer, int iStartAddr, int iNum)
{
    int i, j;
    int iLen = 0;
    ushort_t usTemp;

    if (iNum <= 0)
        return (0);

    iLen = iNum * 2;
    for (i = 0; i < iNum; i++)
    {
        usTemp = *(pusBuffer + iStartAddr + i);
        // printf("reg pos=%d, value=%d ",iStartAdd + i, w_temp);
        *(pcRet++) = usTemp >> 8;
        *(pcRet++) = usTemp & 0xFF;
    }
    return (iLen);
}

// write buffer data
int jWriteDataBuf(ushort_t *pusTarget, char *pcSource, int iStartAddr, int iNum)
{
    int i, j;
    ushort_t usTemp;

    if (iNum <= 0)
        return (0);

    for (i = 0; i < iNum; i++)
    {
        usTemp = *(pcSource + i * 2);
        usTemp = (usTemp << 8) + (*(pcSource + i * 2 + 1));
        *(pusTarget + iStartAddr + i) = usTemp;
    }
    return (iNum);
}
int jProInit(void)
{
    int i;
    TrigeTimes = 0;
    RunTimes = 0;
    StickRun = 0;
    LogWrite_Enable = 1;
    ReadFileStatus = 0;
    GpIoStatus = 0;
    RunStatus = 0;
    WriteSysFileStatus = 0;
    ReadWriteParaStatus = 0;
    RWeight_Status = 0;
    RTav_Status = 0;
    RLdw_Status = 0;
    RLwt_Status = 0;
    RBdr_Status = 0;
    ComRun_Status = 0;
    firstCycle = 1;
    AlarmCode1 = 0;
    AlarmCode2 = 0;
    AlarmCode3 = 0;
    AlarmCode4 = 0;
    AlarmCode5 = 0;
    for (i = Start_Port; i < End_Port; i++)
    {
        SerialOpenStatus[i] = 0;
        SerialWriteStatus[i] = 0;
        SerialReadStatus[i] = 0;
    }

    IoMsvComEnable = 255;
}
void jProcReset(void)
{
    int i;
    StickRun = 0;
    LogWrite_Enable = 1;
    ReadFileStatus = 0;
    GpIoStatus = 0;
    RunStatus = 0;
    WriteSysFileStatus = 0;
    ReadWriteParaStatus = 0;
    RWeight_Status = 0;
    RTav_Status = 0;
    RLdw_Status = 0;
    RLwt_Status = 0;
    RBdr_Status = 0;
    AlarmCode1 = 0;
    AlarmCode2 = 0;
    AlarmCode3 = 0;
    AlarmCode4 = 0;
    AlarmCode5 = 0;
    RZeroRange_Status = 0;
    for (i = Start_Port; i < End_Port; i++)
    {
        SerialOpenStatus[i] = 0;
        SerialWriteStatus[i] = 0;
        SerialReadStatus[i] = 0;
    }
    IoMsvComEnable = Column_SelectStatus;
}

// 文件初始化
int jReadFileInit(void)
{
    int i, j, k;
    char log_char[500];
    char ac_temp[20];
    xini_file_t ini(ConfigFile);

    // modbus para
    m_iServerEnable = ini["General Setting"]["MODBUS_TCP_ENABLE"];
    m_iModbusTcpPort = ini["General Setting"]["MODBUS_TCP_PORT"];
    m_iModbusTcpID = ini["General Setting"]["MODBUS_TCP_ID"];

    // 打印-----------
    LOG_INFO("jReadFile:ModbusTcpIp: Enable%d,TcpPort%d,TcpId%d!\n", m_iServerEnable, m_iModbusTcpPort, m_iModbusTcpID);

    // ad104 para
    fmd = ini["AD104 Para"]["FMD"];
    asf = ini["AD104 Para"]["ASF"];
    hsm = ini["AD104 Para"]["HSM"];
    icr = ini["AD104 Para"]["ICR"];
    nov = ini["AD104 Para"]["NOV"];
    cof = ini["AD104 Para"]["COF"];

    // 打印-----------
    LOG_INFO("jReadFile: Ad104:%d,%d,%d,%d,%d,%d!\n", fmd, asf, hsm, icr, nov, cof);

    // para
    ReadCount = ini["Para"]["ReadCount"];
    WTTime = ini["Para"]["WTTime"];
    WTDelay = ini["Para"]["WTDelay"];
    TavValueRange = ini["Para"]["TavValueRange"];
    ModbusDataBuf[PlcRAdr_ReadCount] = (ushort_t)ReadCount;
    ModbusDataBuf[PlcRAdr_WTDealy] = (ushort_t)WTTime;
    ModbusDataBuf[PlcRAdr_WTTime] = (ushort_t)WTDelay;
    ModbusDataBuf[PlcRAdr_TavValueRange] = (int)TavValueRange;
    // 打印-----------
    LOG_INFO("jReadFile: Para:ReadCount(%d),WTTime(%d),WTDelay(%d),TavValueRange(%d)!\n", ReadCount, WTTime, WTDelay, TavValueRange);

    Column_SelectStatus = ini["Column Sel"]["Column_SelectStatus"];
    ModbusDataBuf[PlcWAdr_RunColumnSel] = (ushort_t)Column_SelectStatus;
    // 打印-----------
    LOG_INFO("jReadFile: Column Sel:Column_SelectStatus(%d)!\n", Column_SelectStatus);

    // 打印-----------
    m_duration = ini["IO"]["IO_Duration"];
    IO_Start = ini["IO"]["IO_Start"];
    LOG_INFO("jReadFile: IO_Duration:%d,IO_Start:%d!\n", m_duration, IO_Start);

    // com para
    ModbusDataBuf[PlcWAdr_Com_Enable] = 0;
    for (i = Start_Port; i < End_Port; i++)
    {
        memset(&ac_temp[0], 0x00, sizeof(ac_temp));
        sprintf(&ac_temp[0], "com%d Setting", i + 1);
        memcpy(&m_STCommPara[i].tSerialPort, ini[&ac_temp[0]]["SerialPort"], sizeof(ini[&ac_temp[0]]["SerialPort"]));
        m_STCommPara[i].iPort = i;
        m_STCommPara[i].iEnable = ini[&ac_temp[0]]["Enable"];
        m_STCommPara[i].iBaud = ini[&ac_temp[0]]["BaudRate"];
        m_STCommPara[i].iDataBits = ini[&ac_temp[0]]["DataBits"];
        m_STCommPara[i].iStopBits = ini[&ac_temp[0]]["StopBits"];
        m_STCommPara[i].iParity = ini[&ac_temp[0]]["Parity"];
        m_STCommPara[i].iTimeInterVal = ini[&ac_temp[0]]["TimeInterval"];
        ModbusDataBuf[PlcWAdr_Com_Enable] += m_STCommPara[i].iEnable * (ushort_t)pow(2, i);
        // 打印-----------
        LOG_INFO("jReadFile: SerialPort: %s,Port: %d,Enable: %d,Baud: %d,DataBits: %d,StopBits: %d,Parity: %d,TimeInterVal: %d!\n", m_STCommPara[i].tSerialPort, m_STCommPara[i].iPort, m_STCommPara[i].iEnable, m_STCommPara[i].iBaud, m_STCommPara[i].iDataBits, m_STCommPara[i].iStopBits, m_STCommPara[i].iParity, m_STCommPara[i].iTimeInterVal);
        // 打印-----------
    }
    return Return_Success;
}
// IO初始化
int jGPIoInit(void)
{
    int ai_gpio[MaxIo_Nums] = {0};
    // 初始化IO
    write_do_control(0x00);
    /*gpio0 used to receive run signal from PLC 运行信号*/
    ai_gpio[0] = jGet_di_state(0x01);
    LOG_INFO("运行信号 DI-1:%d\n", ai_gpio[0]);
    // /* gpio1 used to notify PLC the weight is run 模块准备好信号*/
    ai_gpio[1] = jGet_do_state(0x01);
    LOG_INFO("模块准备好信号 DO-1:%d\n", ai_gpio[1]);
    // /* gpio2 used to receive trigger signal from PLC 称重触发信号*/
    ai_gpio[2] = jGet_di_state(0x02);
    LOG_INFO("称重触发信号 DI-2:%d\n", ai_gpio[2]);
    // /* gpio3 used to notify PLC the measurement is complete 称重触发信号返回*/
    ai_gpio[3] = jGet_do_state(0x02);
    LOG_INFO("称重触发信号返回 DO-2:%d\n", ai_gpio[3]);
    // /* gpio4 used to receive   from 物料信号*/
    ai_gpio[4] = jGet_di_state(0x04);
    LOG_INFO("物料信号 DI-3:%d\n", ai_gpio[4]);
    // /* gpio5 used to notify PLC 模块进入运行状态 */
    ai_gpio[5] = jGet_do_state(0x04);
    LOG_INFO("模块进入运行状态 DO-3:%d\n", ai_gpio[5]);
    // /* gpio4 used to receive   from PLC 复位信号*/
    ai_gpio[6] = jGet_di_state(0x08);
    LOG_INFO("复位信号 DI-4:%d\n", ai_gpio[6]);
    // /* gpio5 used to notify PLC  */
    ai_gpio[7] = jGet_do_state(0x08);
    LOG_INFO(" DO-4:%d\n", ai_gpio[7]);
    return 0;
}

// 信号量建立
void jSemCreat(void)
{
    int res;
    int i;
    char log_char[500];
    // create sys Run semaphore
    for (i = Start_Port; i < End_Port; i++)
    {
        res = sem_init(&semComStart[i], 0, 0);
        if (res != 0)
        {
            // 打印-----------
            LOG_ERROR("jSemCreat:semaphore Com START intitialization failed!\n");
            // 打印-----------
            exit(EXIT_FAILURE);
        }
    }

    // create TCP semaphore
    res = sem_init(&semModbusTcp, 0, 0);
    if (res != 0)
    {
        LOG_ERROR("jSemCreat:semaphore TCP intitialization failed\n");
        exit(EXIT_FAILURE);
    }
}
int jDiCheckPositive(IN int i_gpio, IN bool OverEnable, IN long OverTime)
{
    int i;
    char log_char[500];
    int i_gpionum = Int_Initial;
    int Signal, signalCheck;
    long Time1, Time2, Time_DValue;
    bool TimeEnable;
    i_gpionum = i_gpio;
    Time_DValue = OverTime;
    TimeEnable = OverEnable;
    Time1 = jGetTick();
    LOG_INFO("jDiCheckPositive:i_gpionum num is %d!\n", i_gpionum);
    if (Time_DValue < 2 * (long)m_duration)
        Time_DValue = 2 * (long)m_duration;
    Signal = jGet_di_state(i_gpionum);
    LOG_INFO("jDiCheckPositive:Io num %d, signal %d!\n", i_gpionum, Signal);
    if ((i_gpionum < 0) || (i_gpionum >= MaxIo_Nums))
    {
        LOG_ERROR("jDiCheckPositive : DiCheckPositive Num%d error! \n", i_gpionum);
        return Return_Invalid_Para;
    }
    while (1)
    {
        if (Signal ^ jGet_di_state(i_gpionum))
        {
            jPselect(10);
            LOG_INFO("jDiCheckPositive:Judge Io num %d,  signal %d,%d!\n", i_gpionum, Signal, ioctl(iGPIOFD, GPIO_IOCTL_GET, &i_gpionum));
            if (jGet_di_state(i_gpionum) == 1)
            {
                LOG_INFO("jDiCheckPositive:Judge signal is True!\n");
                jPselect(m_duration);
                if (jGet_di_state(i_gpionum) == 1)
                {
                    LOG_INFO("jDiCheckPositive :Di Check%d is high!\n", i_gpionum);
                    return Return_HighLevel;
                }
                else
                {
                    LOG_ERROR("jDiCheckPositive :Di Check%d is error high_to_low!\n", i_gpionum);
                    return Return_Failed;
                }
            }
            else
            {
                LOG_INFO("jDiCheckPositive :Di Check%d is low!\n", i_gpionum);
                return Return_Failed;
            }
        }
        else
        {
            if (TimeEnable)
            {
                Time2 = jGetTick();
                if ((Time2 - Time1) > Time_DValue)
                {
                    LOG_INFO("jDiCheckPositive :Di Check%d is Overtime!\n", i_gpionum);
                    return Return_Failed;
                }
            }
        }
    }
    return Return_Failed;
}
int jDiCheckNegative(IN int i_gpio, IN bool OverEnable, IN long OverTime)
{
    int i;
    char log_char[500];
    int i_gpionum = Int_Initial;
    int Signal, signalCheck;
    long Time1, Time2, Time_DValue;
    bool TimeEnable;
    i_gpionum = i_gpio;
    Time_DValue = OverTime;
    TimeEnable = OverEnable;
    Time1 = jGetTick();
    if (Time_DValue < 2 * (long)m_duration)
        Time_DValue = 2 * (long)m_duration;
    Signal = ioctl(iGPIOFD, GPIO_IOCTL_GET, &i_gpionum);
    LOG_INFO("jDiCheckNegative:Io num %d, signal %d!\n", i_gpionum, Signal);
    if ((i_gpionum < 0) || (i_gpionum >= MaxIo_Nums))
    {
        LOG_ERROR("jDiCheckNegative : DiCheckNegative Num%d error! \n", i_gpionum);
        return Return_Invalid_Para;
    }
    while (1)
    {
        if (Signal ^ jGet_di_state(i_gpionum))
        {
            jPselect(20);
            LOG_INFO("jDiCheckNegative:Judge Io num %d,  signal %d,%d!\n", i_gpionum, Signal, ioctl(iGPIOFD, GPIO_IOCTL_GET, &i_gpionum));
            if (jGet_di_state(i_gpionum) == 0)
            {
                LOG_INFO("jDiCheckNegative:Judge signal is True!\n");
                jPselect(m_duration);
                if (jGet_di_state(i_gpionum) == 0)
                {
                    LOG_INFO("jDiCheckNegative :Di Check%d is low!\n", i_gpionum);
                    return Return_LowLevel;
                }
                else
                {
                    LOG_ERROR("jDiCheckNegative :Di Check%d is error low_to_high!\n", i_gpionum);
                    return Return_Failed;
                }
            }
            else
            {
                LOG_INFO("jDiCheckNegative :Di Check%d is High!\n", i_gpionum);
                return Return_Failed;
            }
        }
        else
        {
            if (TimeEnable)
            {
                Time2 = jGetTick();
                if ((Time2 - Time1) > Time_DValue)
                {
                    LOG_INFO("jDiCheckNegative :Di Check%d is Overtime!\n", i_gpionum);
                    return Return_Failed;
                }
            }
        }
    }
    return Return_Failed;
}

void *jProcIoReadWrite(void *pv)
{
    dio_init();
    jPselect(100);
    unsigned char uc_do_temp;
    while (1)
    {
        g_uc_di_status_current = read_di_status();
        // printf("g_uc_di_status_current:%d\n", g_uc_di_status_current);
        g_uc_do_status_current = read_do_status();
        // printf("g_uc_do_status_current:%d\n", g_uc_do_status_current);
        uc_do_temp = 0;
        int i;
        for (i = 0; i < 4; i++)
        {
            uc_do_temp = uc_do_temp | ((g_uc_do_set_value[i] & 0x01) << i);
        }
        if (uc_do_temp != g_uc_do_status_current)
        {
            write_do_control(uc_do_temp);
        }
        jPselect(5);
    }
}

int jGet_di_state(unsigned char id)
{
    unsigned char di_state = (g_uc_di_status_current)&id ? 1 : 0;
    return di_state;
}

int jGet_do_state(unsigned char id)
{
    unsigned char do_state = (g_uc_do_status_current)&id ? 1 : 0;
    return do_state;
}
