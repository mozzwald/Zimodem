/*
   Copyright 2016-2019 Bo Zimmerman

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

	   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
//#define TCP_SND_BUF                     4 * TCP_MSS
#define ZIMODEM_VERSION "3.5"
const char compile_date[] = __DATE__ " " __TIME__;
#define DEFAULT_NO_DELAY true
#define null 0

#ifdef ARDUINO_ESP32_DEV
# define ZIMODEM_ESP32
#elif defined(ESP32)
# define ZIMODEM_ESP32
#elif defined(ARDUINO_ESP320)
# define ZIMODEM_ESP32
#elif defined(ARDUINO_NANO32)
# define ZIMODEM_ESP32
#elif defined(ARDUINO_LoLin32)
# define ZIMODEM_ESP32
#elif defined(ARDUINO_ESPea32)
# define ZIMODEM_ESP32
#elif defined(ARDUINO_QUANTUM)
# define ZIMODEM_ESP32
#else
# define ZIMODEM_ESP8266
#endif


#ifdef ZIMODEM_ESP32
# define PIN_FACTORY_RESET GPIO_NUM_0
# define DEFAULT_PIN_DCD GPIO_NUM_14
# define DEFAULT_PIN_CTS GPIO_NUM_13
# define DEFAULT_PIN_RTS GPIO_NUM_15 // unused
# define DEFAULT_PIN_RI GPIO_NUM_32
# define DEFAULT_PIN_DSR GPIO_NUM_12
# define DEFAULT_PIN_DTR GPIO_NUM_27
# define debugPrintf Serial.printf
# define INCLUDE_SD_SHELL true
# define DEFAULT_FCT FCT_DISABLED
# define SerialConfig uint32_t
# define UART_CONFIG_MASK 0x8000000
# define UART_NB_BIT_MASK      0B00001100 | UART_CONFIG_MASK
# define UART_NB_BIT_5         0B00000000 | UART_CONFIG_MASK
# define UART_NB_BIT_6         0B00000100 | UART_CONFIG_MASK
# define UART_NB_BIT_7         0B00001000 | UART_CONFIG_MASK
# define UART_NB_BIT_8         0B00001100 | UART_CONFIG_MASK
# define UART_PARITY_MASK      0B00000011
# define UART_PARITY_NONE      0B00000000
# define UART_NB_STOP_BIT_MASK 0B00110000
# define UART_NB_STOP_BIT_0    0B00000000
# define UART_NB_STOP_BIT_1    0B00010000
# define UART_NB_STOP_BIT_15   0B00100000
# define UART_NB_STOP_BIT_2    0B00110000
# define preEOLN serial.prints
# define echoEOLN serial.write
# define HARD_DCD_HIGH 1
//# define HARD_DCD_LOW 1
#else  // ESP-8266, e.g. ESP-01, ESP-12E, inverted for C64Net WiFi Modem
//# define DEFAULT_PIN_DSR 13  // We don't need this for Atari, move it to an unused pin so we can move UART to other pins.
# define DEFAULT_PIN_DSR 10
//# define DEFAULT_PIN_DTR 12 // We don't need this for Atari, move it to an unused pin so we can use it for CMD
# define DEFAULT_PIN_DTR 9
# define DEFAULT_PIN_RI 14
# define DEFAULT_PIN_RTS 4
# define DEFAULT_PIN_CTS 5 // is 0 for ESP-01, see getDefaultCtsPin() below.
# define DEFAULT_PIN_DCD 0
# define DEFAULT_PIN_MTR 16 // Atari Motor Ctrl
# define DEFAULT_PIN_CMD 12 // Atari Command
# define DEFAULT_FCT FCT_AMTRCTL // Atari Motor Ctrl
# define RS232_INVERTED 1
# define debugPrintf doNothing
# define preEOLN(...)
# define echoEOLN(...) serial.prints(EOLN)
#endif

/* START Atari COMMAND */
#include <FS.h>

// Uncomment for Debug on 2nd UART (GPIO 2)
#define DEBUG_S

enum {ID, COMMAND, AUX1, AUX2, CHECKSUM, ACK, NAK, PROCESS, WAIT} cmdState;
#define DELAY_T5          1500
#define READ_CMD_TIMEOUT  12
#define CMD_TIMEOUT       50
#define STATUS_SKIP       8
unsigned long cmdTimer = 0;
byte statusSkipCount = 0;
byte sector[128];
volatile bool cmdFlag=false;
File atr;

union
{
  struct
  {
    unsigned char devic;
    unsigned char comnd;
    unsigned char aux1;
    unsigned char aux2;
    unsigned char cksum;
  };
  byte cmdFrameData[5];
} cmdFrame;
/* END Atari COMMAND */

#ifdef RS232_INVERTED
# define DEFAULT_DCD_HIGH  HIGH
# define DEFAULT_DCD_LOW  LOW
# define DEFAULT_CTS_HIGH  HIGH
# define DEFAULT_CTS_LOW  LOW
# define DEFAULT_RTS_HIGH  HIGH
# define DEFAULT_RTS_LOW  LOW
# define DEFAULT_RI_HIGH  HIGH
# define DEFAULT_RI_LOW  LOW
# define DEFAULT_DSR_HIGH  HIGH
# define DEFAULT_DSR_LOW  LOW
# define DEFAULT_DTR_HIGH  HIGH
# define DEFAULT_DTR_LOW  LOW
#else
# define DEFAULT_DCD_HIGH  LOW
# define DEFAULT_DCD_LOW  HIGH
# define DEFAULT_CTS_HIGH  LOW
# define DEFAULT_CTS_LOW  HIGH
# define DEFAULT_RTS_HIGH  LOW
# define DEFAULT_RTS_LOW  HIGH
# define DEFAULT_RI_HIGH  LOW
# define DEFAULT_RI_LOW  HIGH
# define DEFAULT_DSR_HIGH  LOW
# define DEFAULT_DSR_LOW  HIGH
# define DEFAULT_DTR_HIGH  LOW
# define DEFAULT_DTR_LOW  HIGH
#endif

#define DEFAULT_BAUD_RATE 1200
#define DEFAULT_SERIAL_CONFIG SERIAL_8N1
#define MAX_PIN_NO 50
#define INTERNAL_FLOW_CONTROL_DIV 380


class ZMode
{
  public:
    virtual void serialIncoming();
    virtual void loop();
};

#include "pet2asc.h"
#include "rt_clock.h"
#include "filelog.h"
#include "serout.h"
#include "connSettings.h"
#include "wificlientnode.h"
#include "stringstream.h"
#include "phonebook.h"
#include "wifiservernode.h"
#include "zstream.h"
#include "proto_http.h"
#include "proto_ftp.h"
#include "zcommand.h"
#include "zconfigmode.h"

#ifdef INCLUDE_SD_SHELL
#include "proto_xmodem.h"
#include "proto_zmodem.h"
#include "zbrowser.h"
#endif

static WiFiClientNode *conns = null;
static WiFiServerNode *servs = null;
static PhoneBookEntry *phonebook = null;
static bool pinSupport[MAX_PIN_NO];
static bool browseEnabled = false;
static String termType = "Zimodem";

static ZMode *currMode = null;
static ZStream streamMode;
//static ZSlip slipMode; // not yet implemented
static ZCommand commandMode;
static ZConfig configMode;
#ifdef INCLUDE_SD_SHELL
static ZBrowser browseMode;
#endif
static RealTimeClock zclock(0);

enum BaudState
{
  BS_NORMAL,
  BS_SWITCH_TEMP_NEXT,
  BS_SWITCHED_TEMP,
  BS_SWITCH_NORMAL_NEXT
};

static bool wifiConnected =false;
static String wifiSSI;
static String wifiPW;
static String hostname;
static SerialConfig serialConfig = DEFAULT_SERIAL_CONFIG;
static int baudRate=DEFAULT_BAUD_RATE;
static int dequeSize=1+(DEFAULT_BAUD_RATE/INTERNAL_FLOW_CONTROL_DIV);
static BaudState baudState = BS_NORMAL; 
static unsigned long resetPushTimer=0;
static int tempBaud = -1; // -1 do nothing
static int dcdStatus = LOW;
static int pinDCD = DEFAULT_PIN_DCD;
static int pinCTS = DEFAULT_PIN_CTS;
static int pinRTS = DEFAULT_PIN_RTS;
static int pinDSR = DEFAULT_PIN_DSR;
static int pinDTR = DEFAULT_PIN_DTR;
static int pinRI = DEFAULT_PIN_RI;
static int pinMTR = DEFAULT_PIN_MTR; // Atari Motor Ctrl
static int pinCMD = DEFAULT_PIN_CMD; // Atari Command
static int dcdActive = DEFAULT_DCD_HIGH;
static int dcdInactive = DEFAULT_DCD_LOW;
static int ctsActive = DEFAULT_CTS_HIGH;
static int ctsInactive = DEFAULT_CTS_LOW;
static int rtsActive = DEFAULT_RTS_HIGH;
static int rtsInactive = DEFAULT_RTS_LOW;
static int riActive = DEFAULT_RI_HIGH;
static int riInactive = DEFAULT_RI_LOW;
static int dtrActive = DEFAULT_DTR_HIGH;
static int dtrInactive = DEFAULT_DTR_LOW;
static int dsrActive = DEFAULT_DSR_HIGH;
static int dsrInactive = DEFAULT_DSR_LOW;

/* START Atari COMMAND */
/**
   calculate 8-bit checksum.
*/
byte sio_checksum(byte* chunk, int length)
{
  int chkSum = 0;
  for (int i = 0; i < length; i++) {
    chkSum = ((chkSum + chunk[i]) >> 8) + ((chkSum + chunk[i]) & 0xff);
  }
  return (byte)chkSum;
}

/**
   ISR for falling COMMAND
*/
void ICACHE_RAM_ATTR sio_isr_cmd()
{
  cmdFlag=true;
}

/**
   Get ID
*/
void sio_get_id()
{
  cmdFrame.devic = Serial.read();
  if (cmdFrame.devic == 0x31)
    cmdState = COMMAND;
  else
  {
    cmdState = WAIT;
    cmdTimer = 0;
    sio_reset_serial();
  }

#ifdef DEBUG_S
  Serial1.print("CMD DEVC: ");
  Serial1.println(cmdFrame.devic, HEX);
#endif
}

/**
   Get Command
*/

void sio_get_command()
{
  cmdFrame.comnd = Serial.read();
  cmdState=AUX1;

//  if (cmdFrame.comnd == 'R' || cmdFrame.comnd == 'W' || cmdFrame.comnd == 'P' || cmdFrame.comnd == 'S' )
//    cmdState = AUX1;
//  else
//  {
//    cmdState = WAIT;
//    cmdTimer = 0;
//  }

#ifdef DEBUG_S
  Serial1.print("CMD CMND: ");
  Serial1.println(cmdFrame.comnd, HEX);
#endif
}

/**
   Get aux1
*/
void sio_get_aux1()
{
  cmdFrame.aux1 = Serial.read();
  cmdState = AUX2;

#ifdef DEBUG_S
  Serial1.print("CMD AUX1: ");
  Serial1.println(cmdFrame.aux1, HEX);
#endif
}

/**
   Get aux2
*/
void sio_get_aux2()
{
  cmdFrame.aux2 = Serial.read();
  cmdState = CHECKSUM;

#ifdef DEBUG_S
  Serial1.print("CMD AUX2: ");
  Serial1.println(cmdFrame.aux2, HEX);
#endif
}

/**
   Get Checksum, and compare
*/
void sio_get_checksum()
{
  byte ck;
  cmdFrame.cksum = Serial.read();
  ck = sio_checksum((byte *)&cmdFrame.cmdFrameData, 4);

#ifdef DEBUG_S
    Serial1.print("CMD CKSM: ");
    Serial1.print(cmdFrame.cksum, HEX);
#endif

    if (ck == cmdFrame.cksum)
    {
#ifdef DEBUG_S
      Serial1.println(", ACK");
#endif
      sio_ack();
    }
    else
    {
#ifdef DEBUG_S
      Serial1.println(", NAK");
#endif
      sio_nak();
    }
}

/**
   Process command
*/

void sio_process()
{
  switch (cmdFrame.comnd)
  {
    case 'R':
      sio_read();
      break;
    case 'S':
      sio_status();
      break;
    case '!':
      sio_format();
      break;
  }

  sio_reset_serial();
  cmdState = WAIT;
  cmdTimer = 0;
}

/**
   format (fake)
*/
void sio_format()
{
  byte ck;

  for (int i=0;i<128;i++)
    sector[i]=0;

  sector[0]=0xFF; // no bad sectors.
  sector[1]=0xFF;

  ck = sio_checksum((byte *)&sector, 128);

  delayMicroseconds(DELAY_T5); // t5 delay
  Serial.write('C'); // Completed command
  Serial.flush();

  // Write data frame
  Serial.write(sector,128);

  // Write data frame checksum
  Serial.write(ck);
  Serial.flush();
  delayMicroseconds(200);
#ifdef DEBUG_S
  Serial1.printf("We faked a format.\n");
#endif
}

/**
   Read
*/
void sio_read()
{
  byte ck;
  byte sector[128];
  int offset =(256 * cmdFrame.aux2)+cmdFrame.aux1;
  offset *= 128;
  offset -= 128;
  offset += 16; // skip 16 byte ATR Header
  atr.seek(offset, SeekSet);
  atr.read(sector, 128);

  ck = sio_checksum((byte *)&sector, 128);

  delayMicroseconds(1500); // t5 delay
  Serial.write('C'); // Completed command
  Serial.flush();

  // Write data frame
  Serial.write(sector,128);

  // Write data frame checksum
  Serial.write(ck);
  Serial.flush();
  delayMicroseconds(200);
#ifdef DEBUG_S
  Serial1.print("SIO READ OFFSET: ");
  Serial1.print(offset);
  Serial1.print(" - ");
  Serial1.println((offset + 128));
#endif
}

/**
   Status
*/
void sio_status()
{
  byte status[4] = {0x00, 0xFF, 0xFE, 0x00};
  byte ck;

  ck = sio_checksum((byte *)&status, 4);

  delayMicroseconds(DELAY_T5); // t5 delay
  Serial.write('C'); // Command always completes.
  Serial.flush();
  delayMicroseconds(200);
  //delay(1);

  // Write data frame
  for (int i = 0; i < 4; i++)
    Serial.write(status[i]);

  // Write checksum
  Serial.write(ck);
  Serial.flush();
  delayMicroseconds(200);
}

/**
   Send an acknowledgement
*/
void sio_ack()
{
  delayMicroseconds(500);
  Serial.write('A');
  Serial.flush();
  //cmdState = PROCESS;
  sio_process();
}

/**
   Send a non-acknowledgement
*/
void sio_nak()
{
  delayMicroseconds(500);
  Serial.write('N');
  Serial.flush();
  cmdState = WAIT;
  cmdTimer = 0;
}

void sio_incoming(){
  switch (cmdState)
  {
    case ID:
      sio_get_id();
      break;
    case COMMAND:
      sio_get_command();
      break;
    case AUX1:
      sio_get_aux1();
      break;
    case AUX2:
      sio_get_aux2();
      break;
    case CHECKSUM:
      sio_get_checksum();
      break;
    case ACK:
      sio_ack();
      break;
    case NAK:
      sio_nak();
      break;
    case PROCESS:
      sio_process();
      break;
    case WAIT:
      //Serial.read(); // Toss it for now
      cmdTimer = 0;
      break;
  }
}

void sio_reset_serial()
{
  HWSerial.begin(baudRate, serialConfig);
  HWSerial.swap();
#ifdef DEBUG_S
  Serial1.println("ZIModem Baud");
#endif
}

/* END Atari COMMAND */

static int getDefaultCtsPin()
{
#ifdef ZIMODEM_ESP32
  return DEFAULT_PIN_CTS;
#else
  if((ESP.getFlashChipRealSize()/1024)>=4096) // assume this is a striketerm/esp12e
    return DEFAULT_PIN_CTS;
  else
    return 0;
#endif 
}

static void doNothing(const char* format, ...) 
{
}

static void s_pinWrite(uint8_t pinNo, uint8_t value)
{
  if(pinSupport[pinNo])
  {
    digitalWrite(pinNo, value);
  }
}

static void setHostName(const char *hname)
{
#ifdef ZIMODEM_ESP32
      tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, hname);
#else
      WiFi.hostname(hname);
#endif
}

static bool connectWifi(const char* ssid, const char* password)
{
  while(WiFi.status() == WL_CONNECTED)
  {
    WiFi.disconnect();
    delay(100);
    yield();
  }
#ifndef ZIMODEM_ESP32
  if(hostname.length() > 0)
    setHostName(hostname.c_str());
#endif
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if(hostname.length() > 0)
    setHostName(hostname.c_str());
  bool amConnected = (WiFi.status() == WL_CONNECTED) && (strcmp(WiFi.localIP().toString().c_str(), "0.0.0.0")!=0);
  int WiFiCounter = 0;
  while ((!amConnected) && (WiFiCounter < 30))
  {
    WiFiCounter++;
    if(!amConnected)
      delay(500);
    amConnected = (WiFi.status() == WL_CONNECTED) && (strcmp(WiFi.localIP().toString().c_str(), "0.0.0.0")!=0);
  }
  wifiConnected = amConnected;
  if(!amConnected)
    WiFi.disconnect();
  return wifiConnected;
}

static void checkBaudChange()
{
  switch(baudState)
  {
    case BS_SWITCH_TEMP_NEXT:
      changeBaudRate(tempBaud);
      baudState = BS_SWITCHED_TEMP;
      break;
    case BS_SWITCH_NORMAL_NEXT:
      changeBaudRate(baudRate);
      baudState = BS_NORMAL;
      break;
    default:
      break;
  }
}

static void changeBaudRate(int baudRate)
{
  flushSerial(); // blocking, but very very necessary
  delay(500); // give the client half a sec to catch up
  debugPrintf("Baud change to %d.\n",baudRate);
  dequeSize=1+(baudRate/INTERNAL_FLOW_CONTROL_DIV);
  debugPrintf("Deque constant now: %d\n",dequeSize);
#ifdef ZIMODEM_ESP32
  HWSerial.changeBaudRate(baudRate);
#else
  HWSerial.begin(baudRate, serialConfig);  //Change baud rate
  HWSerial.swap(); //Swap UART Pins for Atari Motor Control (GPIO13-RX & GPIO15-TX)
#endif  
}

static void changeSerialConfig(SerialConfig conf)
{
  flushSerial(); // blocking, but very very necessary
  delay(500); // give the client half a sec to catch up
  debugPrintf("Config changing %d.\n",(int)conf);
  dequeSize=1+(baudRate/INTERNAL_FLOW_CONTROL_DIV);
  debugPrintf("Deque constant now: %d\n",dequeSize);
#ifdef ZIMODEM_ESP32
  HWSerial.changeConfig(conf);
#else
  HWSerial.begin(baudRate, conf);  //Change baud rate
  HWSerial.swap(); //Swap UART Pins for Atari Motor Control (GPIO13-RX & GPIO15-TX)
#endif  
  debugPrintf("Config changed.\n");
}

static int checkOpenConnections()
{
  int num=WiFiClientNode::getNumOpenWiFiConnections();
  if(num == 0)
  {
    if((dcdStatus == dcdActive)
    &&(dcdStatus != dcdInactive))
    {
      dcdStatus = dcdInactive;
      s_pinWrite(pinDCD,dcdStatus);
      if(baudState == BS_SWITCHED_TEMP)
        baudState = BS_SWITCH_NORMAL_NEXT;
      if(currMode == &commandMode)
        clearSerialOutBuffer();
    }
  }
  else
  {
    if((dcdStatus == dcdInactive)
    &&(dcdStatus != dcdActive))
    {
      dcdStatus = dcdActive;
      s_pinWrite(pinDCD,dcdStatus);
      if((tempBaud > 0) && (baudState == BS_NORMAL))
        baudState = BS_SWITCH_TEMP_NEXT;
    }
  }
  return num;
}

void setup() 
{
  for(int i=0;i<MAX_PIN_NO;i++)
    pinSupport[i]=false;
#ifdef ZIMODEM_ESP32
  Serial.begin(115200); //the debug port
  Serial.setDebugOutput(true);
  debugPrintf("Debug port open and ready.\n");
  for(int i=12;i<=23;i++)
    pinSupport[i]=true;
  for(int i=25;i<=27;i++)
    pinSupport[i]=true;
  for(int i=32;i<=33;i++)
    pinSupport[i]=true;
  pinSupport[36]=true;
  pinSupport[39]=true;
#else
  pinSupport[0]=true;
  pinSupport[2]=true;
  if((ESP.getFlashChipRealSize()/1024)>=4096) // assume this is a strykelink/esp12e
  {
    pinSupport[4]=true;
    pinSupport[5]=true;
    for(int i=9;i<=15;i++)
      pinSupport[i]=true;
    pinSupport[11]=false;
  }
#endif
  initSDShell();
  currMode = &commandMode;
  if(!SPIFFS.begin())
  {
    SPIFFS.format();
    SPIFFS.begin();
    debugPrintf("SPIFFS Formatted.");
  }
#ifdef DEBUG_S
  Serial1.begin(19200);
  Serial1.println();
  Serial1.println("FujiNet ZIModem Started");
#endif
  atr=SPIFFS.open("/autorun.atr","r");
  attachInterrupt(digitalPinToInterrupt(DEFAULT_PIN_CMD), sio_isr_cmd, FALLING);
  cmdState = WAIT; // Start in SIO Command WAIT State
  cmdFlag = false;
  HWSerial.begin(DEFAULT_BAUD_RATE, DEFAULT_SERIAL_CONFIG);  //Start Serial
  HWSerial.swap(); //Swap UART Pins for Atari Motor Control (GPIO13-RX & GPIO15-TX)
  commandMode.loadConfig();
  PhoneBookEntry::loadPhonebook();
  dcdStatus = dcdInactive;
  s_pinWrite(pinDCD,dcdStatus);
  flushSerial();
}

void checkFactoryReset()
{
#ifdef ZIMODEM_ESP32
    if(!digitalRead(PIN_FACTORY_RESET))
    {
      if(resetPushTimer != 1)
      {
        if(resetPushTimer==0)
        {
          resetPushTimer=millis();
          if(resetPushTimer==1)
            resetPushTimer++;
        }
        else
        if((millis() - resetPushTimer) > 5000)
        {
          SPIFFS.remove("/zconfig.txt");
          SPIFFS.remove("/zphonebook.txt");
          SPIFFS.remove("/zlisteners.txt");
          PhoneBookEntry::clearPhonebook();
          SPIFFS.end();
          SPIFFS.format();
          SPIFFS.begin();
          PhoneBookEntry::clearPhonebook();
          if(WiFi.status() == WL_CONNECTED)
            WiFi.disconnect();
          baudRate = DEFAULT_BAUD_RATE;
          commandMode.loadConfig();
          PhoneBookEntry::loadPhonebook();
          dcdStatus = dcdInactive;
          s_pinWrite(pinDCD,dcdStatus);
          wifiSSI="";
          wifiConnected=false;
          delay(500);
          zclock.reset();
          commandMode.reset();
          resetPushTimer=1;
        }
      }
    }
    else
    if(resetPushTimer != 0)
      resetPushTimer=0;
#endif
}

void loop() 
{
  checkFactoryReset();
  if (cmdFlag)
  {
    if (digitalRead(DEFAULT_PIN_CMD) == LOW)
    {
      cmdState = ID;
      cmdTimer = millis();
      cmdFlag=false;
      //Change baud rate for SIO and swap the pins
#ifdef DEBUG_S
  Serial1.println("SIO Baud");
#endif
      HWSerial.begin(19200);
      HWSerial.swap();
    }
  }

  if(HWSerial.available() > 0)
  {
    if (cmdState == WAIT)
    {
      currMode->serialIncoming();
    }
    else
    {
      sio_incoming();
    }
  }
  currMode->loop();
  zclock.tick();
  if (millis() - cmdTimer > CMD_TIMEOUT && cmdState != WAIT)
  {
    Serial1.print("SIO CMD TIMEOUT: ");
    Serial1.println(cmdState);
    sio_reset_serial();
    cmdState = WAIT;
    cmdTimer = 0;
  }
}
