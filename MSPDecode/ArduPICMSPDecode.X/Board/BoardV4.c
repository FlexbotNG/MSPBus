#include "BoardV4.h"

// PIC16F688 оƬ����
__CONFIG(0x00C2);

void Init() {
    // �ж�����
    INTCON = 0; // ����ȫ���ж�
    GIE = 1; // ȫ���ж�����λ
    PEIE = 1; // �����ж�����λ
    // T0IE = 0; // Timer0����ж�����λ
    // INTE = 0; // INT �ⲿ�ж�����λ
    PIE1 = 1;
    TMR1IE = 1; // ���� Timer1 ���
    // TMR1ON = 1; // Enables Timer1
    T1CON = 0x01; // ����TMR1�Ŀ�����; ʹ��Timer1;�ڲ�ʱ��Դ Fosc/4 Ԥ��Ƶ 1

    // ֹͣ Timer1
    TMR1ON = 0;

    // I/O������
    CMCON0 = 0xFF; // �رձȽ���
    ANSEL = 0x00; // �ر�A/D
    PORTA = 0x00;
    TRISA = 0x3F; // PORTA ����
    PORTC = 0x00;
    TRISC = 0x3F; // PORTC ����

    // ������
    nRAPU = 0; // v9.83 PICC
    WPUA = 0b00000001; // RA0 - PWM6

    //    // �ڲ����� 8MHz
    //    IRCF2 = 1;
    //    IRCF1 = 1;
    //    IRCF0 = 1;

    // A/D ��������
    // ADCS<2:0> Ƶ��Ϊ f(osc)/32, ADC ʱ������Ϊ 4��s
    ADCS2 = 0;
    ADCS1 = 1;
    ADCS0 = 0;
    ADON = 1; // ���� A/D ת��ģ��

    // �˿�
    TRIS_PWM0 = OUTPUT;
    TRIS_PWM1 = OUTPUT;
    TRIS_PWM2 = OUTPUT;
    TRIS_PWM3 = OUTPUT;
    TRIS_PWM4 = OUTPUT;
    TRIS_PWM5 = OUTPUT;
    TRIS_PWM6 = OUTPUT;

    PWM0 = LOW;
    PWM1 = LOW;
    PWM2 = LOW;
    PWM3 = LOW;
    PWM4 = LOW;
    PWM5 = LOW;
    PWM6 = LOW;
}

unsigned int analogReadX(unsigned char tnADPort) {
    // 10bit A/D
    unsigned int nResult = 0x0000;

    CHS2 = (tnADPort >> 2) & 0x01;
    CHS1 = (tnADPort >> 1) & 0x01;
    CHS0 = tnADPort & 0x01;
    ADFM = 1; // A/Dת������Ҷ���
    ADCON0 |= 0x02; // GO/DONE = 1
    while (ADCON0 & 0x02);
    nResult = (unsigned int) ((ADRESH << 8) + ADRESL);

    return nResult;
}
