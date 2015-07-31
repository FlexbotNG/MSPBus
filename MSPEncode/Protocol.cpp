#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "MultiWii.h"
#include "Serial.h"
#include "Protocol.h"
#include "RX.h"

/************************************** MultiWii Serial Protocol *******************************************************/
// Multiwii Serial Protocol 0 
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
//range id [50-99] won't be assigned and can therefore be used for any custom multiwii fork without further MSP id conflict

#define MSP_PRIVATE              1     //in+out message      to be used for a generic framework : MSP + function code (LIST/GET/SET) + data. no code yet

#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONF           120   //out message         Servo settings

#define MSP_NAV_STATUS           121   //out message         Returns navigation status
#define MSP_NAV_CONFIG           122   //out message         Returns navigation parameters

#define MSP_CELLS                130   //out message         FRSKY Battery Cell Voltages

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONF       212   //in message          Servo settings
#define MSP_SET_MOTOR            214   //in message          PropBalance function
#define MSP_SET_NAV_CONFIG       215   //in message          Sets nav config parameters - write to the eeprom  

#define MSP_ENCODE               220   // Skypup 2015.07.26

#define MSP_SET_ACC_TRIM         239   //in message          set acc angle trim values
#define MSP_ACC_TRIM             240   //out message         get acc angle trim values
#define MSP_BIND                 241   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

#ifdef DEBUGMSG
#define DEBUG_MSG_BUFFER_SIZE 128
static char debug_buf[DEBUG_MSG_BUFFER_SIZE];
static uint8_t head_debug;
static uint8_t tail_debug;
static uint8_t debugmsg_available();
static void debugmsg_serialize(uint8_t l);
#endif

static uint8_t CURRENTPORT=0;

#define INBUF_SIZE 64
static uint8_t inBuf[INBUF_SIZE][UART_NUMBER];
static uint8_t checksum[UART_NUMBER];
static uint8_t indRX[UART_NUMBER];
static uint8_t cmdMSP[UART_NUMBER];

void evaluateOtherData(uint8_t sr);
void evaluateCommand(uint8_t c);

static uint8_t read8()  {
  return inBuf[indRX[CURRENTPORT]++][CURRENTPORT]&0xff;
}
static uint16_t read16() {
  uint16_t t = read8();
  t+= (uint16_t)read8()<<8;
  return t;
}
static uint32_t read32() {
  uint32_t t = read16();
  t+= (uint32_t)read16()<<16;
  return t;
}

static void serialize8(uint8_t a) {
  SerialSerialize(CURRENTPORT,a);
  checksum[CURRENTPORT] ^= a;
}
static void serialize16(int16_t a) {
  serialize8((a   ) & 0xFF);
  serialize8((a>>8) & 0xFF);
}
static void serialize32(uint32_t a) {
  serialize8((a    ) & 0xFF);
  serialize8((a>> 8) & 0xFF);
  serialize8((a>>16) & 0xFF);
  serialize8((a>>24) & 0xFF);
}

static void headSerialResponse(uint8_t err, uint8_t s) {
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  checksum[CURRENTPORT] = 0; // start calculating a new checksum
  serialize8(s);
  serialize8(cmdMSP[CURRENTPORT]);
}

static void headSerialReply(uint8_t s) {
  headSerialResponse(0, s);
}

static void headSerialError() {
  headSerialResponse(1,0);
}

static void tailSerialReply() {
  serialize8(checksum[CURRENTPORT]);
  UartSendData(CURRENTPORT);
}

static void serializeNames(PGM_P s) {
  headSerialReply(strlen_P(s));
  for (PGM_P c = s; pgm_read_byte(c); c++)
    serialize8(pgm_read_byte(c));
  tailSerialReply();
}

static void __attribute__ ((noinline)) s_struct_w(uint8_t *cb,uint8_t siz) {
  while(siz--) *cb++ = read8();
}

static void s_struct_partial(uint8_t *cb,uint8_t siz) {
  while(siz--) serialize8(*cb++);
}

static void s_struct(uint8_t *cb,uint8_t siz) {
  headSerialReply(siz);
  s_struct_partial(cb,siz);
  tailSerialReply();
}

static void mspAck() {
  headSerialReply(0);
  tailSerialReply();
}

enum MSP_protocol_bytes {
  IDLE,
  HEADER_START,
  HEADER_M,
  HEADER_ARROW,
  HEADER_SIZE,
  HEADER_CMD
};

void serialCom() {
  uint8_t c,cc,port,state,bytesTXBuff;
  static uint8_t offset[UART_NUMBER];
  static uint8_t dataSize[UART_NUMBER];
  static uint8_t c_state[UART_NUMBER];
  uint32_t timeMax; // limit max time in this function in case of GPS

  timeMax = micros();
  for(port=0;port<UART_NUMBER;port++) {
    CURRENTPORT=port;
#define RX_COND
#if defined(SERIAL_RX) && (UART_NUMBER > 1)
#define RX_COND && (RX_SERIAL_PORT != port)
#endif
    cc = SerialAvailable(port);
    while (cc-- RX_COND) {
      bytesTXBuff = SerialUsedTXBuff(port); // indicates the number of occupied bytes in TX buffer
      if (bytesTXBuff > TX_BUFFER_SIZE - 50 ) return; // ensure there is enough free TX buffer to go further (50 bytes margin)
      c = SerialRead(port);
#ifdef SUPPRESS_ALL_SERIAL_MSP
      evaluateOtherData(c); // no MSP handling, so go directly
#else //SUPPRESS_ALL_SERIAL_MSP
      state = c_state[port];
      // regular data handling to detect and handle MSP and other data
      if (state == IDLE) {
        if (c=='$') state = HEADER_START;
        else evaluateOtherData(c); // evaluate all other incoming serial data
      } 
      else if (state == HEADER_START) {
        state = (c=='M') ? HEADER_M : IDLE;
      } 
      else if (state == HEADER_M) {
        state = (c=='<') ? HEADER_ARROW : IDLE;
      } 
      else if (state == HEADER_ARROW) {
        if (c > INBUF_SIZE) {  // now we are expecting the payload size
          state = IDLE;
          continue;
        }
        dataSize[port] = c;
        checksum[port] = c;
        offset[port] = 0;
        indRX[port] = 0;
        state = HEADER_SIZE;  // the command is to follow
      } 
      else if (state == HEADER_SIZE) {
        cmdMSP[port] = c;
        checksum[port] ^= c;
        state = HEADER_CMD;
      } 
      else if (state == HEADER_CMD) {
        if (offset[port] < dataSize[port]) {
          checksum[port] ^= c;
          inBuf[offset[port]++][port] = c;
        } 
        else {
          if (checksum[port] == c) // compare calculated and transferred checksum
            evaluateCommand(cmdMSP[port]); // we got a valid packet, evaluate it
          state = IDLE;
          cc = 0; // no more than one MSP per port and per cycle
        }
      }
      c_state[port] = state;

      // SERIAL: try to detect a new nav frame based on the current received buffer
#if defined(GPS_SERIAL)
      if (GPS_SERIAL == port) {
        static uint32_t GPS_last_frame_seen; //Last gps frame seen at this time, used to detect stalled gps communication
        if (GPS_newFrame(c)) {
          //We had a valid GPS data frame, so signal task scheduler to switch to compute
          if (GPS_update == 1) GPS_update = 0; 
          else GPS_update = 1; //Blink GPS update
          GPS_last_frame_seen = timeMax;
          GPS_Frame = 1;
        }

        // Check for stalled GPS, if no frames seen for 1.2sec then consider it LOST
        if ((timeMax - GPS_last_frame_seen) > 1200000) {
          //No update since 1200ms clear fix...
          f.GPS_FIX = 0;
          GPS_numSat = 0;
        }
      }
      if (micros()-timeMax>250) return;  // Limit the maximum execution time of serial decoding to avoid time spike
#endif
#endif // SUPPRESS_ALL_SERIAL_MSP
    } // while
  } // for
}

// Skypup 2015.07.26
void MSPEncode()
{
  CURRENTPORT = 0;

//  if (serialHeadTX[CURRENTPORT] != serialTailTX[CURRENTPORT]) 
//    return;

  // Head
  serialize8('$');
  serialize8('M');
  serialize8('>');
  checksum[CURRENTPORT] = 0; // start calculating a new checksum
  serialize8(RC_CHANS*2);
  serialize8(MSP_ENCODE);
  // Payload
  s_struct_partial((uint8_t*)&rcData,RC_CHANS*2);
  // Tail
  tailSerialReply();
}

void evaluateCommand(uint8_t c) {
}

// evaluate all other incoming serial data
void evaluateOtherData(uint8_t sr) {
}

void SerialWrite16(uint8_t port, int16_t val)
{
  CURRENTPORT=port;
  serialize16(val);
  UartSendData(port);
}

