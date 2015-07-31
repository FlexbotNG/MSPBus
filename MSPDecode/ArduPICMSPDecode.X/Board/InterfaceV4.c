#include "InterfaceV4.h"

static unsigned char cOldHeartBeat = 0;
static unsigned char nPoint = 0; // ��ǰ�±�
static unsigned int nContTimesKey[2] = 0; // ��������ʱ�ۼƼ���

void SetSendingPWM() {
    // ���PWM�ź�
    unsigned char nFlag = 0;

    while (nPoint == 0 || nPoint > 7); // ��ֹ��һ�������ڸ���2�� nSendingPWM
    nFlag = nPoint;
    while (nFlag == nPoint);
    // ��ʱ nPoint > 1 �� nPoint <= 8
    nSendingPWM[0] = -(nPWM[0] - PWM_FIX);
    nSendingPWM[1] = -(nPWM[1] - PWM_FIX);
}

void interrupt Interrupt() {
    // ==================================================
    // TMR1IF��TMR1�Ĵ��������������������㣩
    // PS: ���������, ��������.
    // ==================================================
    if (TMR1IE & TMR1IF) {
        TMR1IF = 0;
        TMR1ON = 0; // ��ֹ Timer1
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
        TMR1ON = 1; // ���� Timer1
    }

    // ==================================================
    // ���ڽ���
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
    // �����ַ ADDR_EEPROM_INIT_MARK �����ݲ��� TRUE, �� eeprom ��ʼ��
    if (EEPROMRead(ADDR_EEPROM_INIT_MARK) != TRUE) {
        // EEPROM Init Mark
        EEPROMWrite(ADDR_EEPROM_INIT_MARK, TRUE);

        // ���³�ʼ�� EEPROM
    }

    // ���¶�ȡ EEPROM ����������ֵ
}

void EEPROMWriteX(unsigned char tcAddr, unsigned char tcValue) {
    unsigned char cTemp;

    cTemp = EEPROMRead(tcAddr);
    if (cTemp != tcValue) EEPROMWrite(tcAddr, tcValue);
}

void SetEEPROM() {
    // ����ʹ�� EEPROMWriteX ����д�� EEPROM
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
 * ������غ���
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
 * ���ڲ�������
 */
void SetUSART() {
    // ���ô���
    SPEN = 1; // ����ͨ�Ŷ˿ڴ�
    SYNC = 0; // �첽ͨ��ģʽ
    TXEN = 1; // ����������
    // TXIE = 1; // USART �����ж�����λ
    TRISC4 = 0;
    // CREN = 1; // �����������
    TRISC5 = 1;
    //BRG16 = 0; // 8λ
    BRGH = 1; // ���ٲ����ʷ���ģʽ
    SPBRG = 25; // ������Ϊ 38400����Ƭ��Ƶ�� 16MHz, ����ģʽ f / (16 * 9600) - 1 = 16 * 1000 * 1000 / (16 * 38400) - 1

    // �����Ҫ�жϣ���PIE1�Ĵ����е� RCIE λ�� INTCON �Ĵ����� GIE �� PEIE λ�� 1
    GIE = 1; // ȫ���ж�����λ
    PEIE = 1; // �����ж�����λ
    // RCIE = 1; // USART �����ж�����λ
}

void SetUSARTReceiverEnabled() {
    CREN = 1; // �����������
    RCIE = 1; // USART �����ж�����λ
}

void SetUSARTReceiverDisabled() {
    CREN = 0; // ��ֹ��������
    RCIE = 0; // USART �����ж�����λ
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