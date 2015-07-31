#include "InterfaceV4.h"

static unsigned char cOldHeartBeat = 0;
static unsigned char nPoint = 0; // 当前下标
static unsigned int nContTimesKey[2] = 0; // 连续按下时累计计数

void SetSendingPWM() {
    // 舵机PWM信号
    unsigned char nFlag = 0;

    while (nPoint == 0 || nPoint > 7); // 防止在一个周期内更新2次 nSendingPWM
    nFlag = nPoint;
    while (nFlag == nPoint);
    // 这时 nPoint > 1 且 nPoint <= 8
    nSendingPWM[0] = -(nPWM[0] - PWM_FIX);
    nSendingPWM[1] = -(nPWM[1] - PWM_FIX);
}

void interrupt Interrupt() {
    // ==================================================
    // TMR1IF：TMR1寄存器溢出（必须由软件清零）
    // PS: 如果不清零, 程序会假死.
    // ==================================================
    if (TMR1IE & TMR1IF) {
        TMR1IF = 0;
        TMR1ON = 0; // 禁止 Timer1
        if (nPoint == 0) {
            SetUSARTReceiverDisabled();
            PWM1 = 1;
            PWM2 = 1;
            TMR1H = nSendingPWM[0] >> 8;
            TMR1L = nSendingPWM[0];
            nPoint++;
        } else if (nPoint == 1) {
            PWM1 = 0;
            PWM2 = 0;

            serialTailRX = serialHeadRX;
            SetUSARTReceiverEnabled();

            TMR1H = 0xD1;
            TMR1L = 0x1F;
            nPoint++;
        } else if (nPoint >= 2 && nPoint <= 5) {
            PWM1 = 0;
            PWM2 = 0;
            TMR1H = 0xD1;
            TMR1L = 0x1F;
            nPoint++;
        } else {
            PWM1 = 0;
            PWM2 = 0;

            TMR1H = 0xE9;
            TMR1L = 0x90;
            nPoint = 0;

            cHeartBeat++;
            cHeartBeat &= MODE_MASK;
        }
        TMR1ON = 1; // 开启 Timer1
    }

    // ==================================================
    // 串口接收
    // ==================================================
    if (RCIE & RCIF) {
        serialBufferRX[serialHeadRX++] = RCREG;
        if (serialHeadRX >= RX_BUFFER_SIZE)
            serialHeadRX = 0;
    }
}

/* PWM In */
unsigned int GetPWM() {
    unsigned int nWidth = 0;
    unsigned char cOldTRIS = OUTPUT;

    cOldTRIS = TRIS_PWM_IN;
    TRIS_PWM_IN = INPUT;
    nWidth = 0;
    while (PWM_IN == HIGH && nWidth < MAX_TIME_WIDTH) nWidth++;
    nWidth = 0;
    while (PWM_IN == LOW && nWidth < MAX_TIME_SPACE) nWidth++;
    nWidth = 0;
    while (PWM_IN == HIGH && nWidth < MAX_TIME_WIDTH) nWidth++;
    nWidth *= 11; // v9.71a = 10; v9.83 = 11
    TRIS_PWM_IN = cOldTRIS;

    return nWidth;
}

void GetEEPROM() {
    // 如果地址 ADDR_EEPROM_INIT_MARK 的数据不是 TRUE, 则 eeprom 初始化
    if (EEPROMRead(ADDR_EEPROM_INIT_MARK) != TRUE) {
        // EEPROM Init Mark
        EEPROMWrite(ADDR_EEPROM_INIT_MARK, TRUE);

        // 以下初始化 EEPROM
    }

    // 以下读取 EEPROM 并给变量赋值
}

void EEPROMWriteX(unsigned char tcAddr, unsigned char tcValue) {
    unsigned char cTemp;

    cTemp = EEPROMRead(tcAddr);
    if (cTemp != tcValue) EEPROMWrite(tcAddr, tcValue);
}

void SetEEPROM() {
    // 以下使用 EEPROMWriteX 函数写入 EEPROM
}

unsigned char GetTimesHeartBeat() {
    unsigned char cNewHeartBeat;
    unsigned char cTimes = 0;

    cNewHeartBeat = cHeartBeat;
    cTimes = (unsigned char) ((cNewHeartBeat + MODE_MASK + 1 - cOldHeartBeat) & MODE_MASK);
    cOldHeartBeat = cNewHeartBeat;

    return cTimes;
}

/*
 * 串口相关函数
 */
void Send(const unsigned char tcByte) {
    while (!TRMT);
    TXREG = tcByte;
}

void SendString(const unsigned char * tcString) {
    char nSendLinePos = 0;

    while (tcString[nSendLinePos] != 0x00) {
        while (!TRMT);
        TXREG = tcString[nSendLinePos];
        nSendLinePos++;
    }
}

void SendLine(const unsigned char * tcString) {
    SendString(tcString);
    while (!TRMT);
    TXREG = 0x0D;
    while (!TRMT);
    TXREG = 0x0A;
}

void NumericSendString(const unsigned char * tcString, int tnValue) {
    char cPara[10];

    SendString(tcString);
    itoa(cPara, tnValue, 10);
    SendString(cPara);
}

/*
 * 串口参数设置
 */
void SetUSART() {
    // 设置串口
    SPEN = 1; // 串行通信端口打开
    SYNC = 0; // 异步通信模式
    TXEN = 1; // 允许发送数据
    // TXIE = 1; // USART 发送中断允许位
    TRISC4 = 0;
    // CREN = 1; // 允许接收数据
    TRISC5 = 1;
    //BRG16 = 0; // 8位
    BRGH = 1; // 高速波特率发生模式
    SPBRG = 25; // 波特率为 38400，单片机频率 16MHz, 高速模式 f / (16 * 9600) - 1 = 16 * 1000 * 1000 / (16 * 38400) - 1

    // 如果需要中断，将PIE1寄存器中的 RCIE 位和 INTCON 寄存器的 GIE 和 PEIE 位置 1
    GIE = 1; // 全局中断允许位
    PEIE = 1; // 外设中断允许位
    // RCIE = 1; // USART 接收中断允许位
}

void SetUSARTReceiverEnabled() {
    CREN = 1; // 允许接收数据
    RCIE = 1; // USART 接收中断允许位
}

void SetUSARTReceiverDisabled() {
    CREN = 0; // 禁止接收数据
    RCIE = 0; // USART 接收中断允许位
}

unsigned char SerialRead() {
    unsigned char cResult = 0;

    cResult = serialBufferRX[serialTailRX];
    if (serialHeadRX != serialTailRX) {
        if (++serialTailRX >= RX_BUFFER_SIZE) {
            serialTailRX = 0;
        }
    }

    return cResult;
}

unsigned char SerialAvailable() {
    return ((unsigned char) (serialHeadRX - serialTailRX)) % RX_BUFFER_SIZE;
}