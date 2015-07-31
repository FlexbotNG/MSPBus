#include "ArduPIC.h"

/*
 * main() 主函数
 * 模仿 Arduino 的 setup() 与 loop() 函数。
 */
//void main() {
//    Init();
//    setup();
//
//    for (;;) {
//        if (!loop()) return;
//    }
//}

/*
 * 延时函数, MCU会一直等待
 */
void delay(unsigned long tnDelay) {
    for (; tnDelay > 0; tnDelay--);
}