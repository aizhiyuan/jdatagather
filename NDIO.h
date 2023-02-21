/*
 * @Author: your name
 * @Date: 2021-11-07 15:00:59
 * @LastEditTime: 2021-11-07 15:08:38
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: /can程序处理/NDIO.h
 */
#ifndef _NDIO_H_
#define _NDIO_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DI_01           0x01
#define DI_02           0x02
#define DI_03           0x04
#define DI_04           0x08
#define DI_05           0x10
#define DI_06           0x20
#define DI_07           0x40
#define DI_08           0x80

#define DI_ALL          0xFF

#define DO_01           0x01
#define DO_02           0x02
#define DO_03           0x04
#define DO_04           0x08
#define DO_05           0x10
#define DO_06           0x20
#define DO_07           0x40
#define DO_08           0x80

#define DO_ALL          0xFF

#define IO_STATE_ON     0x01
#define IO_STATE_OFF    0x00

#define IO_CONTROL_ON   0x01
#define IO_CONTROL_OFF  0x00


/* ＤＩ　ＤＯ　环境初始化　*/
bool dio_init();

/* 获取全部ＤＩ状态　*/
unsigned char read_di_status();

/* 获取单个ＤＩ状态　*/
bool get_di_state(unsigned char id);

/* 获取全部ＤO状态 */
unsigned char read_do_status();

/*获取单个DO状态*/
bool get_do_state(unsigned char id);

/* DO寄存器控制　*/
void write_do_control(unsigned char id);

/* 多个ＤＯ控制　*/
void set_do_control(unsigned char id, bool mode);

/*　打开全部　ＤＯ　*/
void open_do_all();

/* 关闭全部　ＤＯ　*/
void close_do_all();

#ifdef __cplusplus
}
#endif

#endif
