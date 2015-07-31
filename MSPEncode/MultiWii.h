#ifndef MULTIWII_H_
#define MULTIWII_H_

// #define  VERSION        240
#define  VERSION        230    // 兼容 WinGUI
#define  NAVI_VERSION   7      //This allow sync with GUI

#define MINCHECK 1100
#define MAXCHECK 1900

extern volatile unsigned long timer0_overflow_count;

extern uint32_t currentTime;
extern uint16_t previousTime;
extern uint16_t cycleTime;
extern uint16_t calibratingA;
extern uint16_t calibratingB;
extern uint16_t calibratingG;
extern int16_t  magHold,headFreeModeHold;
extern uint8_t  vbatMin;
extern int32_t  AltHold;
extern int16_t  sonarAlt;
extern int16_t  BaroPID;
extern int16_t  BaroPID_D;
extern int16_t  errorAltitudeI;

extern int16_t  i2c_errors_count;

extern int16_t debug[4];

//extern conf_t conf;
extern int16_t  annex650_overrun_count;
extern uint16_t intPowerTrigger1;

extern int16_t rcData[RC_CHANS];

#endif /* MULTIWII_H_ */
