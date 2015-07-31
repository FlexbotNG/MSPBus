/*
 * 文件名:   BoardV4.h
 * 作者  :   skypup
 * 说明  :   ArduPIC NG alpha 1.1
 *
 * 2015.05.08 ArduPIC NG alpha 1.1 使用 16M 外置晶振
 * 2014.10.06 创建 Alpha 1.0
 */

#ifndef BOARDV4_H
#define	BOARDV4_H

#include <htc.h>
#include "../ArduPIC/ArduPIC.h"

#define BOARD_VERSION     "4"

/*
 * ***************************************************************************************************
 * PIC16F688
 * 16MHz
 * 4KWords Flash / 256B SRAM / 256B EEPROM
 * ***************************************************************************************************
 * PIC16F688 14引脚
 * 1 - VDD
 * 2 - RA5/T1CK/OSC1/CLKIN
 * 3 - RA4/AN3/T1G/OSC2/CLKOUT
 * 4 - RA3/MCLR/VPP
 * 5 - RC5/RX/DT
 * 6 - RC4/C2OUT/TX/CK
 * 7 - RC3/AN7
 * 8 - RC2/AN6
 * 9 - RC1/AN5/C2IN-
 * 10 - RC0/AN4/C2IN+
 * 11 - RA2/AN2/T0CK/INT/C1OUT
 * 12 - RA1/AN1/C1IN-/VRef/ISCPCLK
 * 13 - RA0/AN0/C1IN+/ISCPDAT/ULPWU
 * 14 - VSS
 * ***************************************************************************************************
 * CONFIG: 配置字寄存器
 * bit 15 - 12  保留
 * bit 11       FCMEN: 故障保护时钟监控器使能位
 *              1 = 使能故障保护时钟监控器
 *              0 = 禁止故障保护时钟监控器
 * bit 10       IESO: 内/外部切换位
 *              1 = 使能内/外部切换位
 *              0 = 禁止内/外部切换位
 * bit 9 - 8    BOREN<1:0>: 欠压复位选择位
 *              11 = 使能 BOR
 *              10 = 运行时使能 BOR, 休眠时禁止
 *              01 = PCON寄存器的SBOREN位控制BOR
 *              00 = 禁止BOR
 * bit 7        CPD: 数据代码保护位
 *              1 = 禁止数据存储器代码保护
 *              0 = 使能数据存储器代码保护
 * bit 6        CP: 代码保护位
 *              1 = 禁止程序存储器代码保护
 *              0 = 使能程序存储器代码保护
 * bit 5        MCLRE: MCLR引脚功能选择位
 *              1 = MCLR引脚功能为MCLR
 *              0 = MCLR引脚功能为数字输入, MCLR内部连接到VDD
 * bit 4        PWRTE: 上电延时定时器使能位
 *              1 = 禁止PWRT
 *              0 = 使能PWRT
 * bit 3        WDTE: 看门狗定时器使能位
 *              1 = 使能WDT
 *              0 = 禁止WDT, 但可以通过SWDTEN位(WDTCON<0>)使能
 * bit 2 - 0    FOSC<2:0>: 振荡器选择位
 *              111 = RC振荡器: RA4 为 CLKOUT, RA5 连接 RC
 *              110 = RCIO振荡器: RA4 为 I/O, RA5 连接 RC
 *              101 = INTOSC振荡器: RA4 为 CLKOUT, RA5 为 I/O
 *              100 = INTOSCIO振荡器: RA4 为 I/O, RA5 为 I/O
 *              011 = EC: RA4 为 I/O, RA5 为 CLKIN
 *              010 = HS振荡器: RA4 和 RA5 连接高速晶振/谐振器
 *              001 = XT振荡器: RA4 和 RA5 连接晶振/谐振器
 *              000 = LP振荡器: RA4 和 RA5 连接低功耗晶振
 * ***************************************************************************************************
 */

// 管脚定义
#define PWM0            RC3
#define PWM1            RC2
#define PWM2            RC1
#define PWM3            RC0
#define PWM4            RA2
#define PWM5            RA1
#define PWM6            RA0

#define PWM_IN           PWM6

// 管脚I/O设置
#define TRIS_PWM0       TRISC3
#define TRIS_PWM1       TRISC2
#define TRIS_PWM2       TRISC1
#define TRIS_PWM3       TRISC0
#define TRIS_PWM4       TRISA2
#define TRIS_PWM5       TRISA1
#define TRIS_PWM6       TRISA0

#define TRIS_PWM_IN     TRIS_PWM6

void Init();
unsigned int analogReadX(unsigned char);

#endif	/* BOARDV4_H */

