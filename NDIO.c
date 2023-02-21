/*
 * @Author: your name
 * @Date: 2021-11-07 15:00:59
 * @LastEditTime: 2021-11-07 15:11:24
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: /can程序处理/NDIO.c
 */
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <mcp23017.h>

#include "NDIO.h"

#ifdef __cplusplus
extern "C"
{
#endif

    static int i2c_dio_fd;

    /* ＤＩ　ＤＯ　环境初始化　*/
    bool dio_init()
    {
        wiringPiSetup();
        i2c_dio_fd = wiringPiI2CSetupInterface("/dev/i2c-0", 0x20);
        if (i2c_dio_fd <= 0)
        {
            return false;
        }
        wiringPiI2CWriteReg8(i2c_dio_fd, 0x01, 0x00);
        return true;
    }

    /* 获取全部ＤＩ状态　*/
    unsigned char read_di_status()
    {
        return wiringPiI2CReadReg8(i2c_dio_fd, 0x12);
    }

    /* 获取单个ＤＩ状态　*/
    bool get_di_state(unsigned char id)
    {
        return wiringPiI2CReadReg8(i2c_dio_fd, 0x12) & id ? 1 : 0;
    }

    /* 获取ＤO状态 */
    unsigned char read_do_status()
    {
        return wiringPiI2CReadReg8(i2c_dio_fd, 0x13);
    }

    /*获取单个DO状态*/
    bool get_do_state(unsigned char id)
    {
        return wiringPiI2CReadReg8(i2c_dio_fd, 0x13) & id ? 1 : 0;
    }

    /* DO寄存器控制　*/
    void write_do_control(unsigned char id)
    {
        wiringPiI2CWriteReg8(i2c_dio_fd, 0x13, id);
    }

    /* 多个ＤＯ控制　*/
    void set_do_control(unsigned char id, bool mode)
    {
        unsigned char do_status = read_do_status();

        if (mode)
        {
            do_status |= id;
        }
        else
        {
            do_status &= ~(id);
        }

        wiringPiI2CWriteReg8(i2c_dio_fd, 0x13, do_status);
    }

    /*　打开全部　ＤＯ　*/
    void open_do_all()
    {
        wiringPiI2CWriteReg8(i2c_dio_fd, 0x13, 0xFF);
    }

    /* 关闭全部　ＤＯ　*/
    void close_do_all()
    {
        wiringPiI2CWriteReg8(i2c_dio_fd, 0x13, 0x00);
    }

#ifdef __cplusplus
}
#endif
