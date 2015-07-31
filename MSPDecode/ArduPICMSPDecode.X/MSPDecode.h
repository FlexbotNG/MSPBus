/*
 * �ļ���:   Sample.h
 * ����  :   skypup
 * ˵��  :   Sample
 *
 * 2014.12.09 Alpha 6.00 �л�������Ŀ
 * 2014.10.26 Alpha 5.01 PWM �ֱ��ʴ� 1000 ���ӵ� 4000
 * 2014.10.26 Alpha 5.00 �޸� PWM ���ɳ���
 * 2014.10.25 Alpha 4.17 �� v9.83 PICC ������
 *                       �������� nServo -= ʱ, �����Ϊ����������BUG
 * 2014.10.14 Alpha 4.13 ȥ����ȫ�� Warning
 *                       ʹ�� Lite ģʽ����
 * 2014.10.13 Alpha 4.12 �� GetPWM() ֮�󣬵ȴ���һ���ж�֮�������г���
 * 2014.10.12 Alpha 4.11 �޸� GetPWM(), ��Ӧ��2�� PIC16F688 оƬ
 * 2014.10.08 Alpha 4.10 �ٶȸ�Ϊ 10
 * 2014.10.07 Alpha 4.7  ����EEPROM��ʼ��ʱ���Զ�У�����Ϊ 1 ��BUG����Ϊ CHECK_TIMES
 *                       gnServer[] ��ֵ�� setup ʱ��ʼ��
 * 2014.10.07 Alpha 4.6  �޸Ľ����߼��Ĵ���, ֮ǰ�Ĵ���д����, �����������
 * 2014.10.07 Alpha 4.5  ����
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

