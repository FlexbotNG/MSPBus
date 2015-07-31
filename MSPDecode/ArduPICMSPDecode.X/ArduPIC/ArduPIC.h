/* 
 * 文件名:   ArduPIC.h
 * 作者  :   skypup
 *
 * 2014.10.19 Alpha 2.0 为了减少嵌套层数，去掉 main() 函数
 * 2014.10.07 Alpha 1.1 去掉 BeforeLoop() 和 AfterLoop() 函数
 * 2014.10.06 Alpha 1.0 创建 
 */

#ifndef ARDUPIC_H
#define	ARDUPIC_H

#define ARDUPIC_VERSION     "Alpha 2.0"

#define TRUE                1
#define FALSE               0

#define HIGH                1
#define LOW                 0

#define INPUT               1
#define OUTPUT              0

#define DEFAULT             0
#define VREF                1

// EEPROM 读写函数定义
#define EEPROMRead(tcAddress)           eeprom_read(tcAddress)
#define EEPROMWrite(tcAddress, tcValue) eeprom_write(tcAddress, tcValue)

// *****************************************************************************
// 函数定义
// *****************************************************************************
void Init();

// 仿 Arduino 函数名
void setup();
unsigned char loop();

// 延时函数, MCU会一直等待
void delay(unsigned long);

#endif	/* ARDUPIC_H */