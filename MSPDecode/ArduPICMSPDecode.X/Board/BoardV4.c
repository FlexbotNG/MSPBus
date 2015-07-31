#include "BoardV4.h"

// PIC16F688 芯片配置
__CONFIG(0x00C2);

void Init() {
    // 中断设置
    INTCON = 0; // 禁用全部中断
    GIE = 1; // 全局中断允许位
    PEIE = 1; // 外设中断允许位
    // T0IE = 0; // Timer0溢出中断允许位
    // INTE = 0; // INT 外部中断允许位
    PIE1 = 1;
    TMR1IE = 1; // 允许 Timer1 溢出
    // TMR1ON = 1; // Enables Timer1
    T1CON = 0x01; // 设置TMR1的控制字; 使能Timer1;内部时钟源 Fosc/4 预分频 1

    // 停止 Timer1
    TMR1ON = 0;

    // I/O口设置
    CMCON0 = 0xFF; // 关闭比较器
    ANSEL = 0x00; // 关闭A/D
    PORTA = 0x00;
    TRISA = 0x3F; // PORTA 输入
    PORTC = 0x00;
    TRISC = 0x3F; // PORTC 输入

    // 弱上拉
    nRAPU = 0; // v9.83 PICC
    WPUA = 0b00000001; // RA0 - PWM6

    //    // 内部振荡器 8MHz
    //    IRCF2 = 1;
    //    IRCF1 = 1;
    //    IRCF0 = 1;

    // A/D 功能设置
    // ADCS<2:0> 频率为 f(osc)/32, ADC 时钟周期为 4μs
    ADCS2 = 0;
    ADCS1 = 1;
    ADCS0 = 0;
    ADON = 1; // 启用 A/D 转换模块

    // 端口
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
    ADFM = 1; // A/D转换结果右对齐
    ADCON0 |= 0x02; // GO/DONE = 1
    while (ADCON0 & 0x02);
    nResult = (unsigned int) ((ADRESH << 8) + ADRESL);

    return nResult;
}
