#include "MSPDecode.h"

//#define TEST

static unsigned char cTimesHeartBeat = 0;
static unsigned int nServo[3] = 0; // = cServo * 16
static unsigned char cStatePWM0 = 0;

void setup() {
    SetUSART();
    SetUSARTReceiverEnabled();
    SendLine("USART Init.");

    // GetEEPROM();

    for (unsigned char i = 0; i < 2; i++) {
        nServo[i] = 6000;
        nPWM[i] = nServo[i]; // 6000 = 1500us
    }

    TMR1H = 0xFF;
    TMR1L = 0xFF;
    TMR1ON = 1; // 开启 Timer1
    SetSendingPWM();

    GetTimesHeartBeat(); // 刷新当前心跳记录

    delay(250000); // 电调自检

    PWM0 = HIGH;
}

#define INBUF_SIZE 64
static unsigned char inBuf[INBUF_SIZE];
static unsigned char checksum, indRX, cmdMSP;

enum MSP_protocol_bytes {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD
};

void serialCom() {
    unsigned char c, cc;
    static unsigned char state, dataSize, offset;

    cc = SerialAvailable();
    while (cc--) {
        c = SerialRead();

        if (state == IDLE) {
            if (c == '$') state = HEADER_START;
        } else if (state == HEADER_START) {
            state = (c == 'M') ? HEADER_M : IDLE;
        } else if (state == HEADER_M) {
            state = ((c == '!') || (c == '<') || (c == '>')) ? HEADER_ARROW : IDLE;
        } else if (state == HEADER_ARROW) {
            if (c > INBUF_SIZE) { // now we are expecting the payload size
                state = IDLE;
                continue;
            }
            dataSize = c;
            checksum = c;
            offset = 0;
            indRX = 0;
            state = HEADER_SIZE; // the command is to follow
        } else if (state == HEADER_SIZE) {
            cmdMSP = c;
            checksum ^= c;
            state = HEADER_CMD;
        } else if (state == HEADER_CMD) {
            if (offset < dataSize) {
                checksum ^= c;
                inBuf[offset++] = c;
            } else {
                if (checksum == c) // compare calculated and transferred checksum
                {
                    evaluateCommand(cmdMSP); // we got a valid packet, evaluate it
                }
                state = IDLE;
                cc = 0; // no more than one MSP per port and per cycle
            }
        } else {
            state = IDLE;
        }
    }
}

void evaluateCommand(unsigned char c) {
    unsigned char cTemp = 0x00;

    // NumericSendString(" c = ", c);
    // SendLine("");

    switch (c) {
        case MSP_ARDUPIC_SET_PORT0:
            cStatePWM0 = read8();
            break;
        case MSP_ARDUPIC_SET_PORT1:
            cTemp = read8();
            if (cTemp == 0)
                PWM1 = LOW;
            else
                PWM1 = HIGH;
            break;
        case MSP_ARDUPIC_SET_PORT2:
            cTemp = read8();
            if (cTemp == 0)
                PWM2 = LOW;
            else
                PWM2 = HIGH;
            break;
        case MSP_ARDUPIC_SET_PORT3:
            cTemp = read8();
            if (cTemp == 0)
                PWM3 = LOW;
            else
                PWM3 = HIGH;
            break;
        case MSP_ARDUPIC_SET_PORT4:
            cTemp = read8();
            if (cTemp == 0)
                PWM4 = LOW;
            else
                PWM4 = HIGH;
            break;
        case MSP_ARDUPIC_SET_PORT5:
            cTemp = read8();
            if (cTemp == 0)
                PWM5 = LOW;
            else
                PWM5 = HIGH;
            break;
        case MSP_ARDUPIC_SET_PORT6:
            cTemp = read8();
            if (cTemp == 0)
                PWM6 = LOW;
            else
                PWM6 = HIGH;
            break;
        case MSP_ENCODE:
        {
            unsigned int nEncodeLow = 0, nEncodeHigh = 0, nEncode = 0;

            //            nEncodeLow = (unsigned int) read8();
            //            nEncodeHigh = (unsigned int) read8();
            for (int nChannel = 0; nChannel < CHANNEL; nChannel++) {
                nEncodeLow = read8();
                nEncodeHigh = read8();
            }
            nEncode = (nEncodeHigh << 8) | nEncodeLow;
            nEncode <<= 2;
            nServo[0] = nEncode;
            nServo[1] = nEncode;
#if defined(TEST)
            NumericSendString(" nEncodeLow = ", nEncodeLow);
            NumericSendString(" nEncodeHigh = ", nEncodeHigh);
            NumericSendString(" nEncode = ", nEncode);
            SendLine("");
#endif
        }
            break;
    }

    // delay(1);
}

unsigned char read8() {
    return inBuf[indRX++]&0xff;
}

void main() {
    Init();
    setup();

    for (;;) {
        serialCom();

        cTimesHeartBeat = GetTimesHeartBeat();

        // Servo PWM
        nPWM[0] = nServo[0];
        nPWM[1] = nServo[1];
        SetSendingPWM();
    }
}
