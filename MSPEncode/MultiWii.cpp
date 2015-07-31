#include <avr/io.h>

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "MultiWii.h"
#include "RX.h"
#include "Serial.h"
#include "Protocol.h"

#include <avr/pgmspace.h>

uint32_t currentTime = 0;
uint16_t previousTime = 0;
uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
uint16_t calibratingB = 0;  // baro calibration = get new ground pressure value
uint16_t calibratingG;
int16_t  magHold,headFreeModeHold; // [-180;+180]
int32_t  AltHold; // in cm
int16_t  sonarAlt;
int16_t  BaroPID = 0;
int16_t  BaroPID_D = 0;
int16_t  errorAltitudeI = 0;

// **************
// gyro+acc IMU
// **************
int16_t gyroZero[3] = {0,0,0};

#if defined(ARMEDTIMEWARNING)
  uint32_t  ArmedTimeWarningMicroSeconds = 0;
#endif

int16_t  debug[4];

int16_t  i2c_errors_count = 0;

#if defined(THROTTLE_ANGLE_CORRECTION)
  int16_t throttleAngleCorrection = 0; // correction of throttle in lateral wind,
  int8_t  cosZ = 100;                  // cos(angleZ)*100
#endif

int16_t rcData[RC_CHANS];    // interval [1000;2000]

// ************************
// EEPROM Layout definition
// ************************
static uint8_t dynP8[2], dynD8[2];

void setup() {
  SerialOpen(0,SERIAL0_COM_SPEED);

  LEDPIN_PINMODE;
  POWERPIN_PINMODE;
  BUZZERPIN_PINMODE;
  STABLEPIN_PINMODE;
  POWERPIN_OFF;

  configureReceiver();
  previousTime = micros();
}

// ******** Main Loop *********
void loop () {
  static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  static uint8_t rcSticks;       // this hold sticks position for command combos
  uint8_t axis,i;
  int16_t error,errorAngle;
  int16_t delta;
  int16_t PTerm = 0,ITerm = 0,DTerm, PTermACC, ITermACC;
  static int16_t lastGyro[2] = {0,0};
  static int16_t errorAngleI[2] = {0,0};
  #if PID_CONTROLLER == 1
  static int32_t errorGyroI_YAW;
  static int16_t delta1[2],delta2[2];
  static int16_t errorGyroI[2] = {0,0};
  #elif PID_CONTROLLER == 2
  static int16_t delta1[3],delta2[3];
  static int32_t errorGyroI[3] = {0,0,0};
  static int16_t lastError[3] = {0,0,0};
  int16_t deltaSum;
  int16_t AngleRateTmp, RateError;
  #endif
  static uint16_t rcTime  = 0;
  static uint16_t rcTimeLoop  = 0;
  int16_t rc;
  int32_t prop = 0;

  if ((int16_t)(currentTime-rcTime) >0 ) {
    // rcTime = currentTime + 20000;  // 20ms
    rcTime = currentTime + 3000;      // 3ms

    computeRC();
  }
 
  while(1) {
    currentTime = micros();
    cycleTime = currentTime - previousTime;
    #if defined(LOOP_TIME)
      if (cycleTime >= LOOP_TIME) break;
    #else
      break;  
    #endif
  }
  previousTime = currentTime;

  serialCom();
  MSPEncode();
}
