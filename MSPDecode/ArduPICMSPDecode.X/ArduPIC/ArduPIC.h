/* 
 * �ļ���:   ArduPIC.h
 * ����  :   skypup
 *
 * 2014.10.19 Alpha 2.0 Ϊ�˼���Ƕ�ײ�����ȥ�� main() ����
 * 2014.10.07 Alpha 1.1 ȥ�� BeforeLoop() �� AfterLoop() ����
 * 2014.10.06 Alpha 1.0 ���� 
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

// EEPROM ��д��������
#define EEPROMRead(tcAddress)           eeprom_read(tcAddress)
#define EEPROMWrite(tcAddress, tcValue) eeprom_write(tcAddress, tcValue)

// *****************************************************************************
// ��������
// *****************************************************************************
void Init();

// �� Arduino ������
void setup();
unsigned char loop();

// ��ʱ����, MCU��һֱ�ȴ�
void delay(unsigned long);

#endif	/* ARDUPIC_H */