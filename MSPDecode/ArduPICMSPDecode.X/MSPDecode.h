/*
 * 文件名:   Sample.h
 * 作者  :   skypup
 * 说明  :   Sample
 *
 * 2014.12.09 Alpha 6.00 靶机气垫项目
 * 2014.10.26 Alpha 5.01 PWM 分辨率从 1000 增加到 4000
 * 2014.10.26 Alpha 5.00 修改 PWM 生成程序
 * 2014.10.25 Alpha 4.17 换 v9.83 PICC 编译器
 *                       修正计算 nServo -= 时, 会溢出为负数的严重BUG
 * 2014.10.14 Alpha 4.13 去除了全部 Warning
 *                       使用 Lite 模式编译
 * 2014.10.13 Alpha 4.12 在 GetPWM() 之后，等待下一个中断之后再运行程序
 * 2014.10.12 Alpha 4.11 修改 GetPWM(), 适应第2批 PIC16F688 芯片
 * 2014.10.08 Alpha 4.10 速度改为 10
 * 2014.10.07 Alpha 4.7  修正EEPROM初始化时，自动校验次数为 1 的BUG，改为 CHECK_TIMES
 *                       gnServer[] 数值在 setup 时初始化
 * 2014.10.07 Alpha 4.6  修改解锁逻辑的代码, 之前的代码写反了, 容易让人误解
 * 2014.10.07 Alpha 4.5  创建
 */

#ifndef SAMPLE_H
#define	SAMPLE_H

#include <stdio.h>
#include <stdlib.h>
#include "ArduPIC/ArduPIC.h"
#include "Board/InterfaceV4.h"

#define VERSION             "Alpha 6.00"

// MSP
#define MSP_ARDUPIC_SET_PORT0   10
#define MSP_ARDUPIC_SET_PORT1   11
#define MSP_ARDUPIC_SET_PORT2   12
#define MSP_ARDUPIC_SET_PORT3   13
#define MSP_ARDUPIC_SET_PORT4   14
#define MSP_ARDUPIC_SET_PORT5   15
#define MSP_ARDUPIC_SET_PORT6   16
#define MSP_ENCODE              220 // Skypup 2015.07.26

#define CHANNEL                 2

void SetNewState(unsigned char);

unsigned char read8();
void serialCom();
void evaluateCommand(unsigned char c);

#endif	/* SAMPLE_H */

