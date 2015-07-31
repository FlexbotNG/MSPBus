/*
 * �ļ���:   InterfaceV4.h
 * ����  :   skypup
 * ˵��  :   Interface Version 4
 *
 * 2015.05.08 Alpha 4.0 ArduPIC NG alpha 1.1
 * 2014.10.26 Alpha 1.2 PWM �ֱ��ʴ� 1000 ���ӵ� 4000
 * 2014.10.12 Alpha 1.1 �޸� GetPWM(), ��Ӧ�� PIC16F688 оƬ
 * 2014.10.07 Alpha 1.0 ����
 */

#ifndef INTERFACEV4_H
#define	INTERFACEV4_H

#include <stdlib.h>
#include "../ArduPIC/ArduPIC.h"
#include "BoardV4.h"

#define INTERFACE_VERSION       "4"

// ����
#define PWM_FIX                 0x4F

#define ADDR_EEPROM_INIT_MARK   0xF0

#define MAX_TIME_WIDTH          500
#define MAX_TIME_SPACE          5000

#define MODE_MASK               0x3F        // ��������ȡģ

#define RX_BUFFER_SIZE          32
extern volatile unsigned char serialHeadRX = 0;
extern volatile unsigned char serialTailRX = 0;
extern volatile unsigned char serialBufferRX[RX_BUFFER_SIZE] = "";

extern volatile unsigned char cHeartBeat = 0;

extern volatile unsigned int nPWM[2] = 0; // ����-������
extern volatile unsigned int nSendingPWM[2] = 0; // ����-���ڷ���

unsigned int GetPWM();
void SetServoX(unsigned char, unsigned int);
void SetServo(unsigned char, unsigned char);
void GetEEPROM();
void EEPROMWriteX(unsigned char, unsigned char);
void SetEEPROM();
unsigned char GetHeartBeat();
unsigned char GetTimesHeartBeat();

unsigned char GetPoint();
void SetSendingPWM();

void SetUSART();
void SetUSARTReceiverEnabled();
void SetUSARTReceiverDisabled();
void Send(const unsigned char tcByte);
void SendString(const unsigned char *);
void SendLine(const unsigned char * tcString);
void NumericSendString(const unsigned char *, int);

unsigned char SerialRead();
unsigned char SerialAvailable();

#endif	/* INTERFACEV4_H */

