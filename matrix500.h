/*  Header File for Artial Matrix500
 *
 *  Copyright (C) 2006 Artila Electronics,Ltd.
 *
 *  Purpose: For Artila's specific devices
 *  For example, GPIO, 3in1 serial port setting, EEPRORM
 *  read/write......and so on.
 *
 *  Note 1: Other devices are compliant to standard Linux's API
 *  Note 2: For details, please refer to the examples which are
 *          provided by Artila.
 */

#ifndef _MATRIX500_H_
#define _MATRIX500_H_

/* /dev/gpio, GPIO IOCTL Commands */
#define GPIO_IOCTL_COUNT 0
#define GPIO_IOCTL_DIPSW 1
#define GPIO_IOCTL_GET 2
#define GPIO_IOCTL_SET 3
#define GPIO_IOCTL_CLEAR 4
#define GPIO_IOCTL_OUTPUT 5
#define GPIO_IOCTL_INPUT 6
#define GPIO_IOCTL_MODE 7

/* /dev/ttyS1, Serial Port IOCTL Commands */
#define UC500_GET_UART_TYPE 0xe001
#define UC500_SET_UART_TYPE 0xe002
/* Parameters */
#define UC500_UART_TYPE_232 232
#define UC500_UART_TYPE_422 422
#define UC500_UART_TYPE_485 485

/* EEPROM is accessed via Linux's sysfs scheme */
/* Please refer to Matrix500 example and Linux documentation */

#endif
