#include "ArduPIC.h"

/*
 * main() ������
 * ģ�� Arduino �� setup() �� loop() ������
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
 * ��ʱ����, MCU��һֱ�ȴ�
 */
void delay(unsigned long tnDelay) {
    for (; tnDelay > 0; tnDelay--);
}