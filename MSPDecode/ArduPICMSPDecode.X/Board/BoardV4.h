/*
 * �ļ���:   BoardV4.h
 * ����  :   skypup
 * ˵��  :   ArduPIC NG alpha 1.1
 *
 * 2015.05.08 ArduPIC NG alpha 1.1 ʹ�� 16M ���þ���
 * 2014.10.06 ���� Alpha 1.0
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
 * PIC16F688 14����
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
 * CONFIG: �����ּĴ���
 * bit 15 - 12  ����
 * bit 11       FCMEN: ���ϱ���ʱ�Ӽ����ʹ��λ
 *              1 = ʹ�ܹ��ϱ���ʱ�Ӽ����
 *              0 = ��ֹ���ϱ���ʱ�Ӽ����
 * bit 10       IESO: ��/�ⲿ�л�λ
 *              1 = ʹ����/�ⲿ�л�λ
 *              0 = ��ֹ��/�ⲿ�л�λ
 * bit 9 - 8    BOREN<1:0>: Ƿѹ��λѡ��λ
 *              11 = ʹ�� BOR
 *              10 = ����ʱʹ�� BOR, ����ʱ��ֹ
 *              01 = PCON�Ĵ�����SBORENλ����BOR
 *              00 = ��ֹBOR
 * bit 7        CPD: ���ݴ��뱣��λ
 *              1 = ��ֹ���ݴ洢�����뱣��
 *              0 = ʹ�����ݴ洢�����뱣��
 * bit 6        CP: ���뱣��λ
 *              1 = ��ֹ����洢�����뱣��
 *              0 = ʹ�ܳ���洢�����뱣��
 * bit 5        MCLRE: MCLR���Ź���ѡ��λ
 *              1 = MCLR���Ź���ΪMCLR
 *              0 = MCLR���Ź���Ϊ��������, MCLR�ڲ����ӵ�VDD
 * bit 4        PWRTE: �ϵ���ʱ��ʱ��ʹ��λ
 *              1 = ��ֹPWRT
 *              0 = ʹ��PWRT
 * bit 3        WDTE: ���Ź���ʱ��ʹ��λ
 *              1 = ʹ��WDT
 *              0 = ��ֹWDT, ������ͨ��SWDTENλ(WDTCON<0>)ʹ��
 * bit 2 - 0    FOSC<2:0>: ����ѡ��λ
 *              111 = RC����: RA4 Ϊ CLKOUT, RA5 ���� RC
 *              110 = RCIO����: RA4 Ϊ I/O, RA5 ���� RC
 *              101 = INTOSC����: RA4 Ϊ CLKOUT, RA5 Ϊ I/O
 *              100 = INTOSCIO����: RA4 Ϊ I/O, RA5 Ϊ I/O
 *              011 = EC: RA4 Ϊ I/O, RA5 Ϊ CLKIN
 *              010 = HS����: RA4 �� RA5 ���Ӹ��پ���/г����
 *              001 = XT����: RA4 �� RA5 ���Ӿ���/г����
 *              000 = LP����: RA4 �� RA5 ���ӵ͹��ľ���
 * ***************************************************************************************************
 */

// �ܽŶ���
#define PWM0            RC3
#define PWM1            RC2
#define PWM2            RC1
#define PWM3            RC0
#define PWM4            RA2
#define PWM5            RA1
#define PWM6            RA0

#define PWM_IN           PWM6

// �ܽ�I/O����
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

