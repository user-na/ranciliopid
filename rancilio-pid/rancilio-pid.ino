/********************************************************
   Version 2.0.0 wip (xx.xx.2020)
******************************************************/

/********************************************************
  INCLUDES
******************************************************/
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include "userConfig.h" // needs to be configured by the user
#include <U8g2lib.h>
#include "PID_v1.h" //for PID calculation
#include <OneWire.h>    //Library for one wire communication to dallas temptemp sensor
#include "TSIC.h"       //Library for TSIC temp sensor
#include <BlynkSimpleEsp8266.h>
#include "icon.h"   //user icons for display
#include "ESPCrashMonitor-master/ESPCrashMonitor.h"
#include "RemoteDebug.h"        //https://github.com/JoaoLopesF/RemoteDebug
// task scheduler definitions must be before including
#define _TASK_PRIORITY      // support for layered task prioritization
#define _TASK_WDT_IDS       // support for Task IDs and Control Points
#define _TASK_TIMECRITICAL  // time critical tracking option enabled
#include <TaskScheduler.h>

/********************************************************
  Instances
******************************************************/
RemoteDebug Debug;
//DISPLAY constructor, select the one needed for your display
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);   //e.g. 1.3"
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);    //e.g. 0.96"

/********************************************************
  DEFINES
******************************************************/
#define FIRMWARE_VERSION "2.0.0 wip"   //Firmware version

#define DEBUGMODE   // Debug mode is active if #define DEBUGMODE is set
#define SERIAL_BAUD 115200  // The BAUD rate (speed) of the serial port (console).

//#define BLYNK_PRINT Serial    // In detail debugging for blynk
//#define BLYNK_DEBUG

#ifndef DEBUGMODE
#define DEBUG_printf(x)
#define DEBUG_println(x)
#define DEBUG_print(x)
#define DEBUGSTART(x)
#else
#define DEBUG_DISABLE_AUTO_FUNC true  // Disable te auto function feature of RemoteDebug
#define WEBSOCKET_DISABLED true  // Disable Websocket
#define DEBUG_printf(x, ...) if (Debug.isActive(Debug.ANY)) Debug.printf(x, ##__VA_ARGS__)
#define DEBUG_println(x, ...) if (Debug.isActive(Debug.ANY)) Debug.println(x, ##__VA_ARGS__)
#define DEBUG_print(x, ...) if (Debug.isActive(Debug.ANY)) Debug.print(x, ##__VA_ARGS__)
#define DEBUGSTART(x) Serial.begin(x)
#endif

// Pin definitions
#define pinRelayVentil    12    // Output pin for 3-way-valve
#define pinRelayPumpe     13    // Output pin for pump
#define pinRelayHeater    14    // Output pin for heater
#define pinBrewSwitch     15    // Input pon for brew switch
#define OLED_RESET 16     // Output pin for dispaly reset pin
#define OLED_SCL 5        // Output pin for dispaly clock pin
#define OLED_SDA 4        // Output pin for dispaly data pin

// Display definitions
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels  

// one wire definition
#define ONE_WIRE_BUS 2  // Data wire is plugged into port 2 on the Arduino

/********************************************************
  definitions below must be changed in the userConfig.h file
******************************************************/
int Offlinemodus = OFFLINEMODUS;
const int OnlyPID = ONLYPID;
const int TempSensor = TEMPSENSOR;
const int Brewdetection = BREWDETECTION;
const int fallback = FALLBACK;
const int triggerType = TRIGGERTYPE;
const bool ota = OTA;
const int grafana = GRAFANA;
const unsigned long wifiConnectionDelay = WIFICINNECTIONDELAY;
const unsigned int maxWifiReconnects = MAXWIFIRECONNECTS;
const bool feedForwardControl = FEEDFORWARDCONTROL;
const float brewFlowrate = BREWFLOWRATE;
const float emptyBrewFlowrate = EMPTYBREWFLOWRATE;
const unsigned int brewBoarder_1 = BREWBOARDER_1;
const unsigned int brewBoarder_2 = BREWBOARDER_2;
const unsigned int heatingPower = HEATINGPOWER;
unsigned int brewType = 0;  // 0 = now brew detected; 1 = normal brew detected; 2 = empty brew detected;
int machineLogo = MACHINELOGO;
const unsigned long stabilisingTime = STABILISINGTIME;
const float emptyCorrectionFactor = EMPTYCORRECTIONFACTOR;
const float normalCorrectionFactor = NORMALCORRECTIONFACTOR;
const bool delayHeating = DELAYHEATING;

// Wifi
const char* hostname = HOSTNAME;
const char* auth = AUTH;
const char* ssid = D_SSID;
const char* pass = PASS;
unsigned long lastWifiConnectionAttempt = millis();
unsigned int wifiReconnects = 0; //actual number of reconnects

// OTA
const char* OTAhost = OTAHOST;
const char* OTApass = OTAPASS;

//Blynk
const char* blynkaddress  = BLYNKADDRESS;
const int blynkport = BLYNKPORT;
unsigned int blynkReCnctFlag;  // Blynk Reconnection Flag
unsigned int blynkReCnctCount = 0;  // Blynk Reconnection counter
unsigned long lastBlynkConnectionAttempt = millis();

//backflush values
const unsigned long fillTime = FILLTIME;
const unsigned long flushTime = FLUSHTIME;
int maxflushCycles = MAXFLUSHCYCLES;

/********************************************************
   Declarations
******************************************************/
int pidON = 1 ;                 // 1 = control loop in closed loop
int relayON, relayOFF;          // used for relay trigger type. Do not change!
bool kaltstart = true;       // true = Rancilio started for first time
bool emergencyStop = false;  // Notstop bei zu hoher Temperatur

int inX = 0, inY = 0, inOld = 0, inSum = 0; //used for filter()
int bars = 0; //used for getSignalStrength()
double Q = 0; // thermal energy variable
unsigned long heatingTime = 0;  //time where output=100%; used for FeedForwardControl
unsigned long totalFfwdTime = 0; //used for FeedForwardControl
unsigned long totalHeatingTime = 0; //used for FeedForwardControl
double preHeatingOutput = -1;    //start output; used for FeedForwardControl
bool thermalCalc = false;
bool preHeatingPidMode = 0; //1 = Automatic, 0 = Manual
bool brewDetected = 0;
bool setupDone = false;
int backflushON = 0;            // 1 = activate backflush
int flushCycles = 0;            // number of active flush cycles
int backflushState = 10;        // counter for state machine
bool Ffwd = 0;

/********************************************************
   moving average - brewdetection
*****************************************************/
const int numReadings = 15;             // number of values per Array
double readingstemp[numReadings];        // the readings from Temp
unsigned long readingstime[numReadings];        // the readings from time
double readingchangerate[numReadings];

int readIndex = 1;              // the index of the current reading
double total = 0;               // total sum of readingchangerate[]
double heatrateaverage = 0;     // the average over the numReadings
double changerate = 0;          // local change rate of temprature
double heatrateaveragemin = 0 ;
unsigned long  timeBrewdetection = 0 ;
int timerBrewdetection = 0 ;    // flag is set if brew was detected
int firstreading = 1 ;          // Ini of the field, also used for sensor check

/********************************************************
   PID - values for offline brewdetection
*****************************************************/
double aggbKp = AGGBKP;
double aggbTn = AGGBTN;
double aggbTv = AGGBTV;
#if (aggbTn == 0)
double aggbKi = 0;
#else
double aggbKi = aggbKp / aggbTn;
#endif
double aggbKd = aggbTv * aggbKp ;
double brewtimersoftware = 45;    // 20-5 for detection
double brewboarder = 150 ;        // border for the detection, be carefull: to low: risk of wrong brew detection and rising temperature
const int PonE = PONE;

/********************************************************
   Analog Input
******************************************************/
const int analogPin = 0; // AI0 will be used
int brewcounter = 10;
int brewswitch = LOW;
double brewtime = 25000;  //brewtime in ms
double totalbrewtime = 0; //total brewtime set in softare or blynk
double preinfusion = 2000;  //preinfusion time in ms
double preinfusionpause = 5000;   //preinfusion pause time in ms
unsigned long bezugsZeit = 0;   //total brewed time
unsigned long bezugsZeitAlt = 0;
unsigned long startZeit = 0;    //start time of brew
const unsigned long analogreadingtimeinterval = 100 ; // ms
unsigned long previousMillistempanalogreading ; // ms for analogreading

/********************************************************
   Sensor check
******************************************************/
bool sensorError = false;
int error = 0;
int maxErrorCounter = 10 ;  //depends on intervaltempmes* , define max seconds for invalid data

/********************************************************
   PID
******************************************************/
unsigned long previousMillistemp;  // initialisation at the end of init()
const unsigned long intervaltempmestsic = 800 ;
int pidMode = 1; //1 = Automatic, 0 = Manual

const unsigned int windowSize = 1000;
volatile unsigned int isrCounter = 0;  // counter for ISR
unsigned long windowStartTime;
double Input, Output;
double previousInput = 0;

double setPoint = SETPOINT;
double aggKp = AGGKP;
double aggTn = AGGTN;
double aggTv = AGGTV;
double startKp = STARTKP;
double startTn = STARTTN;
#if (startTn == 0)
double startKi = 0;
#else
double startKi = startKp / startTn;
#endif


#if (aggTn == 0)
double aggKi = 0;
#else
double aggKi = aggKp / aggTn;
#endif
double aggKd = aggTv * aggKp ;

PID bPID(&Input, &Output, &setPoint, aggKp, aggKi, aggKd, PonE, DIRECT);    //PID initialisation

/********************************************************
   Temp Sensors TSIC 306
******************************************************/
TSIC Sensor1(ONE_WIRE_BUS);   // only Signalpin, VCCpin unused by default
uint16_t temperature = 0;     // internal variable used to read temeprature in 2byte from i2c
float Temperatur_C = 0;       // internal variable that holds the converted temperature in °C

/********************************************************
   Pressure sensor
   messure and verify "offset" value, should be 10% of ADC bit reading @supply volate (3.3V)
   same goes for "fullScale", should be 90%
******************************************************/
int offset = 102;   // 10% of ADC input @3.3V supply
int fullScale = 922;  // 90% of ADC input @3.3V supply
int maxPressure = 200; // maximum pressure [psi] according to datasheet
float inputPressure = 0;    // variable to hold pressure value from sensor

/********************************************************
   BLYNK
******************************************************/
//Update Intervall zur App
unsigned long previousMillisBlynk;  // initialisation at the end of init(), to not count boot time
const unsigned long intervalBlynk = 1000;
int blynksendcounter = 1;

//Update für Display
unsigned long previousMillisDisplay;  // initialisation at the end of init(), to not count boot time
const unsigned long intervalDisplay = 500;

/********************************************************
  Task Scheduler
  Current test employs two priority layers:
  Base scheduler runs tasks t1, t2 and t3
  High priority scheduler runs tasks t4 and t5
*****************************************************/
Scheduler lpr, hpr;
// Callback methods prototypes
void refreshTemp();
void checkPressure();
void t3Callback();
void printScreen();
void sendToBlynk();
void printScreen();
void brew();
void readAnalogInput();
// Tasks
Task tCheckTemp(TASK_IMMEDIATE, TASK_FOREVER, &refreshTemp, &lpr, false); //adding task to the chain on creation
Task tCheckPressure(400, TASK_FOREVER, &checkPressure, &lpr, false); //adding task to the chain on creation
Task tCheckWeight(3000, TASK_FOREVER, &t3Callback, &lpr, false); //adding task to the chain on creation
Task tSendBlynk(intervalBlynk, TASK_FOREVER, &sendToBlynk, &lpr, false); //adding task to the chain on creation
Task tPrintScreen(intervalDisplay, TASK_FOREVER, &printScreen, &lpr, false); //adding task to the chain on creation

Task tBrew(500, TASK_FOREVER, &brew, &hpr, false);  //adding task to the chain on creation
Task tAnalogInput(analogreadingtimeinterval, TASK_FOREVER, &readAnalogInput, &hpr, false);  //adding task to the chain on creation


/********************************************************
  Pressure sensor
  Verify before installation: meassured analog input value (should be 3,300 V for 3,3 V supply) and respective ADC value (3,30 V = 1023)
*****************************************************/
void checkPressure() {    
  inputPressure = ((filter(analogRead(analogPin)) - offset) * maxPressure * 0.0689476) / (fullScale - offset);    // pressure conversion and unit conversion [psi] -> [bar]
  DEBUG_print("Pressure [bar]: ");
  DEBUG_println(inputPressure);

  //TODO: what if pressure is <0, error? error only if pressure low and pump running?
}

void t3Callback() {
  //do
}


/********************************************************
  Emergency stop inf temp is to high
*****************************************************/
void testEmergencyStop() {
  if (Input > 120 && emergencyStop == false) {
    emergencyStop = true;
  } else if (Input < 100 && emergencyStop == true) {
    emergencyStop = false;
  }
}

/*************************************************************************************************
** Funktion Filtern()  by GuntherB   2014        **
**************************************************************************************************
** Bildet einen Tiefpassfilter (RC-Glied) nach.             **
** Tau [ms]               **
** Periode = Aufruf alle Periode [ms]               **
**                          **
**                            **
**  Input: FiltVal der gefilterte Wert, NewVal der neue gelesene Wert; FF Filterfaktor, Periode **
**  Output: FiltVal                   **
**  genutzte Globale Variablen:   keine             **
**************************************************************************************************/
void filterPT1(double &FiltVal, float NewVal, unsigned long Periode, unsigned long Tau) {
  static  unsigned long lastRun = 0;
  unsigned long FF = Tau / Periode;
  if (millis() - lastRun < Periode) return;
  if (firstreading) {
    FiltVal = NewVal;   //Prevent ramping up from zero to real value at system startup
    return;
  }
  lastRun = millis();
  FiltVal = ((FiltVal * FF) + NewVal) / (FF + 1);
}


/********************************************************
  DISPLAY - EmergencyStop
*****************************************************/
void displayEmergencyStop(void) {
  u8g2.clearBuffer();
  u8g2.drawXBMP(0, 0, logo_width, logo_height, logo_bits_u8g2);   //draw temp icon
  u8g2.setCursor(32, 24);
  u8g2.print("Ist :  ");
  u8g2.print(Input, 1);
  u8g2.print(" ");
  u8g2.print((char)176);
  u8g2.print("C");
  u8g2.setCursor(32, 34);
  u8g2.print("Soll:  ");
  u8g2.print(setPoint, 1);
  u8g2.print(" ");
  u8g2.print((char)176);
  u8g2.print("C");

  //draw current temp in icon
  if (isrCounter < 500) {
    u8g2.drawLine(9, 48, 9, 5);
    u8g2.drawLine(10, 48, 10, 4);
    u8g2.drawLine(11, 48, 11, 3);
    u8g2.drawLine(12, 48, 12, 4);
    u8g2.drawLine(13, 48, 13, 5);
    u8g2.setCursor(32, 4);
    u8g2.print("HEATING STOPPED");
  }
  u8g2.sendBuffer();
}

/********************************************************
  Read analog input pin
*****************************************************/
void readAnalogInput() {
  //brewswitch = filter(analogRead(analogPin));
  filter(analogRead(analogPin));
}

/********************************************************
  check sensor value.
  If < 0 or difference between old and new >25, then increase error.
  If error is equal to maxErrorCounter, then set sensorError
*****************************************************/
bool checkSensor(float tempInput) {
  bool sensorOK = false;
  bool badCondition = ( tempInput < 0 || tempInput > 150 || fabs(tempInput - previousInput) > 5);
  if ( badCondition && !sensorError) {
    error++;
    sensorOK = false;
    DEBUG_print("WARN: temperature sensor reading: consec_errors = ");    DEBUG_print(error);    DEBUG_print(", temp_current = ");    DEBUG_println(tempInput);
  } else if (badCondition == false && sensorOK == false) {
    error = 0;
    sensorOK = true;
  }
  if (error >= maxErrorCounter && !sensorError) {
    sensorError = true ;
    DEBUG_print("ERROR: temperature sensor malfunction: emp_current = ");    DEBUG_println(tempInput);
  } else if (error == 0 && sensorError) {
    sensorError = false ;
  }
  return sensorOK;
}

/********************************************************
  Refresh temperature.
  Each time checkSensor() is called to verify the value.
  If the value is not valid, new data is not stored.
*****************************************************/
void refreshTemp() {

  previousInput = Input ;
  if (TempSensor == 2)
  {
    /*  variable "temperature" must be set to zero, before reading new data
          getTemperature only updates if data is valid, otherwise "temperature" will still hold old values
    */
    temperature = 0;
    static unsigned long offset = 0;
    unsigned long startDelay = tCheckTemp.getStartDelay();
    DEBUG_print("startdelay Temp:" );
    DEBUG_println(startDelay);
    unsigned long executionTime = micros();
    if (Sensor1.getTemperature(&temperature) ) {  // returns 1 if temperature was read
      executionTime = micros() - executionTime;
      //DEBUG_print("Execution time Temp:" );
      //DEBUG_println(executionTime / 1000);
      if (startDelay == 0 && offset == 0) {
        offset = (executionTime / 1000) - 3;
      } else if (startDelay == 0 && (executionTime / 1000) > 10) {
        offset = 0;
      }
      if (offset > intervaltempmestsic) {   // safety to prevent exception
        offset = 0;
      }
      //DEBUG_print("offset:" );
      //DEBUG_println(offset);
      tCheckTemp.delay(intervaltempmestsic + offset); // start task xy delayed
      Temperatur_C = Sensor1.calc_Celsius(&temperature);
      if (!checkSensor(Temperatur_C) && firstreading == 0) return;  //if sensor data is not valid, abort function; Sensor must be read at least one time at system startup
      filterPT1(Input, Temperatur_C, 400, 2000);
      //Input = Temperatur_C;
      if (Brewdetection != 0) {
        //movAvg();
      }
      if (firstreading != 0) {
        firstreading = 0;
      }
    } else {
      DEBUG_println("Temp reading failed!" );
    }
    DEBUG_println("------------");
  }
}

/********************************************************
    PreInfusion, Brew , if not Only PID
******************************************************/
void brew() {
  if (OnlyPID == 0) {    
    unsigned long currentMillistemp = millis();
    brewswitch = digitalRead(pinBrewSwitch);

    if (brewswitch == LOW && brewcounter > 10 && brewcounter < 43) {   //abort function for state machine from every state
      brewcounter = 42;
    }

    if (brewcounter > 10 && brewcounter < 42) {
      bezugsZeit = currentMillistemp - startZeit;
    }

    totalbrewtime = preinfusion + preinfusionpause + brewtime;    // running every cycle, in case changes are done during brew

    // state machine for brew
    switch (brewcounter) {
      case 10:    // waiting step for brew switch turning on
        if (brewswitch == HIGH) {
          startZeit = millis();
          brewcounter = 20;
          kaltstart = false;    // force reset kaltstart if shot is pulled
        }
        break;
      case 20:    //preinfusioon
        DEBUG_println("Preinfusion");
        digitalWrite(pinRelayVentil, relayON);
        digitalWrite(pinRelayPumpe, relayON);
        brewcounter = 21;
        break;
      case 21:    //waiting time preinfusion
        if (bezugsZeit > preinfusion) {
          brewcounter = 30;
        }
        break;
      case 30:    //preinfusion pause
        DEBUG_println("preinfusion pause");
        digitalWrite(pinRelayVentil, relayON);
        digitalWrite(pinRelayPumpe, relayOFF);
        brewcounter = 31;
        break;
      case 31:    //waiting time preinfusion pause
        if (bezugsZeit > preinfusion + preinfusionpause) {
          brewcounter = 40;
        }
        break;
      case 40:    //brew running
        DEBUG_println("Brew started");
        digitalWrite(pinRelayVentil, relayON);
        digitalWrite(pinRelayPumpe, relayON);
        brewcounter = 41;
        break;
      case 41:    //waiting time brew
        if (bezugsZeit > totalbrewtime) {
          brewcounter = 42;
        }
        break;
      case 42:    //brew finished
        DEBUG_println("Brew stopped");
        digitalWrite(pinRelayVentil, relayOFF);
        digitalWrite(pinRelayPumpe, relayOFF);
        brewcounter = 43;
        break;
      case 43:    // waiting for brewswitch off position
        if (brewswitch == LOW) {
          digitalWrite(pinRelayVentil, relayOFF);
          digitalWrite(pinRelayPumpe, relayOFF);
          currentMillistemp = 0;
          bezugsZeitAlt = bezugsZeit;
          bezugsZeit = 0;
          brewDetected = 0; //rearm brewdetection
          brewcounter = 10;
        }
        break;
    }
  }
}

/********************************************************
    send data to display
******************************************************/
void printScreen() {
  unsigned long startDelay = tPrintScreen.getStartDelay();
  DEBUG_print("startdelay Screen:" );
  DEBUG_println(startDelay);
  unsigned long executionTime = millis();
  if (!sensorError) {
    u8g2.clearBuffer();
    u8g2.drawXBMP(0, 0, logo_width, logo_height, logo_bits_u8g2);   //draw temp icon
    u8g2.setCursor(32, 14);
    u8g2.print("Ist :  ");
    u8g2.print(Input, 1);
    u8g2.print(" ");
    u8g2.print((char)176);
    u8g2.print("C");
    u8g2.setCursor(32, 24);
    u8g2.print("Soll:  ");
    u8g2.print(setPoint, 1);
    u8g2.print(" ");
    u8g2.print((char)176);
    u8g2.print("C");

    // Draw heat bar
    u8g2.drawLine(15, 58, 117, 58);
    u8g2.drawLine(15, 58, 15, 61);
    u8g2.drawLine(117, 58, 117, 61);

    u8g2.drawLine(16, 59, (Output / 10) + 16, 59);
    u8g2.drawLine(16, 60, (Output / 10) + 16, 60);
    u8g2.drawLine(15, 61, 117, 61);

    //draw current temp in icon
    if (abs(Input - setPoint) < 0.3) {
      if (isrCounter < 500) {
        u8g2.drawLine(9, 48, 9, 58 - (Input  / 2));
        u8g2.drawLine(10, 48, 10, 58 - (Input  / 2));
        u8g2.drawLine(11, 48, 11, 58 - (Input  / 2));
        u8g2.drawLine(12, 48, 12, 58 - (Input  / 2));
        u8g2.drawLine(13, 48, 13, 58 - (Input  / 2));
      }
    } else if (Input > 106) {
      u8g2.drawLine(9, 48, 9, 5);
      u8g2.drawLine(10, 48, 10, 4);
      u8g2.drawLine(11, 48, 11, 3);
      u8g2.drawLine(12, 48, 12, 4);
      u8g2.drawLine(13, 48, 13, 5);
    } else {
      u8g2.drawLine(9, 48, 9, 58 - (Input  / 2));
      u8g2.drawLine(10, 48, 10, 58 - (Input  / 2));
      u8g2.drawLine(11, 48, 11, 58 - (Input  / 2));
      u8g2.drawLine(12, 48, 12, 58 - (Input  / 2));
      u8g2.drawLine(13, 48, 13, 58 - (Input  / 2));
    }

    //draw setPoint line
    u8g2.drawLine(18, 58 - (setPoint / 2), 23, 58 - (setPoint / 2));

    // PID Werte ueber heatbar
    u8g2.setCursor(40, 48);

    u8g2.print(bPID.GetKp(), 0); // P
    u8g2.print("|");
    if (bPID.GetKi() != 0) {
      u8g2.print(bPID.GetKp() / bPID.GetKi(), 0);;
    } // I
    else
    {
      u8g2.print("0");
    }
    u8g2.print("|");
    u8g2.print(bPID.GetKd() / bPID.GetKp(), 0); // D
    u8g2.setCursor(98, 48);
    if (pidMode == 1) {
      if (Output < 99) {
        u8g2.print(Output / 10, 2);
      } else if (Output < 999) {
        u8g2.print(Output / 10, 1);
      } else {
        u8g2.print(Output / 10, 0);
      }
      u8g2.print("%");
    } else {
      u8g2.drawBox(97, 48, 28, 9);   //Draw Box
      u8g2.setDrawColor(0);
      if (Output < 99) {
        u8g2.print(Output / 10, 2);
      } else if (Output < 999) {
        u8g2.print(Output / 10, 1);
      } else {
        u8g2.print(Output / 10, 0);
      }
      u8g2.print("%");
      u8g2.setDrawColor(1);
    }


    // Brew
    u8g2.setCursor(32, 34);
    u8g2.print("Brew:  ");
    u8g2.print(bezugsZeit / 1000, 1);
    u8g2.print("/");
    if (ONLYPID == 1) {
      u8g2.print(brewtimersoftware, 0);             // deaktivieren wenn Preinfusion ( // voransetzen )
    }
    else
    {
      u8g2.print(totalbrewtime / 1000, 0);            // aktivieren wenn Preinfusion
    }
    u8g2.print("(");
    u8g2.print(bezugsZeitAlt / 1000, 1);
    u8g2.print(")");
    //draw box
    u8g2.drawFrame(0, 0, 128, 64);

    // Für Statusinfos
    u8g2.drawFrame(32, 0, 84, 12);
    if (Offlinemodus == 0) {
      getSignalStrength();
      if (WiFi.status() == WL_CONNECTED) {
        u8g2.drawXBMP(40, 2, 8, 8, antenna_OK_u8g2);
        for (int b = 0; b <= bars; b++) {
          u8g2.drawVLine(45 + (b * 2), 10 - (b * 2), b * 2);
        }
      } else {
        u8g2.drawXBMP(40, 2, 8, 8, antenna_NOK_u8g2);
        u8g2.setCursor(88, 2);
        u8g2.print("RC: ");
        u8g2.print(wifiReconnects);
      }
      if (Blynk.connected()) {
        u8g2.drawXBMP(60, 2, 11, 8, blynk_OK_u8g2);
      } else {
        u8g2.drawXBMP(60, 2, 8, 8, blynk_NOK_u8g2);
      }
    } else {
      u8g2.setCursor(40, 2);
      u8g2.print("Offlinemodus");
    }
    u8g2.sendBuffer();
  }
  //DEBUG_print("execution time screen: ");
  //DEBUG_println(millis() - executionTime);
  DEBUG_println("-------------");
}

/********************************************************
  send data to Blynk server
*****************************************************/

void sendToBlynk() {
  if (Offlinemodus == 1) return;
  unsigned long startDelay = tSendBlynk.getStartDelay();
  DEBUG_print("startdelay Blynk:" );
  DEBUG_println(startDelay);
  unsigned long executionTime = millis();
  if (Blynk.connected()) {
    if (blynksendcounter == 1) {
      Blynk.virtualWrite(V2, Input);
    }
    if (blynksendcounter == 2) {
      Blynk.virtualWrite(V23, Output);
    }
    if (blynksendcounter == 3) {
      Blynk.virtualWrite(V3, setPoint);
    }
    if (blynksendcounter == 4) {
      Blynk.virtualWrite(V35, heatrateaverage);
    }
    if (blynksendcounter == 5) {
      Blynk.virtualWrite(V36, heatrateaveragemin);
    }
    if (grafana == 1 && blynksendcounter >= 6) {
      Blynk.virtualWrite(V60, Input, Output, bPID.GetKp(), bPID.GetKi(), bPID.GetKd(), setPoint );
      blynksendcounter = 0;
    } else if (grafana == 0 && blynksendcounter >= 5) {
      blynksendcounter = 0;
    }
    blynksendcounter++;
  }
  //DEBUG_print("execution time blynk: ");
  //DEBUG_println(millis() - executionTime);
  DEBUG_println("-------------");
}

/********************************************************
  after ~28 cycles the input is set to 99,66% if the real input value
  sum of inX and inY multiplier must be 1
  increase inX multiplier to make the filter faster
*****************************************************/
int filter(int input) {
  inX = input * 0.3;
  inY = inOld * 0.7;
  inSum = inX + inY;
  inOld = inSum;

  return inSum;
}

/********************************************************
  Get Wifi signal strength and set bars for display
*****************************************************/
void getSignalStrength() {
  if (Offlinemodus == 1) return;

  long rssi;
  if (WiFi.status() == WL_CONNECTED) {
    rssi = WiFi.RSSI();
  } else {
    rssi = -100;
  }

  if (rssi >= -50) {
    bars = 4;
  } else if (rssi < -50 & rssi >= -65) {
    bars = 3;
  } else if (rssi < -65 & rssi >= -75) {
    bars = 2;
  } else if (rssi < -75 & rssi >= -80) {
    bars = 1;
  } else {
    bars = 0;
  }
}

/********************************************************
    Timer 1 - ISR for PID calculation and heat realay output
******************************************************/
void ICACHE_RAM_ATTR onTimer1ISR() {
  timer1_write(6250); // set interrupt time to 20ms

  if (Output <= isrCounter) {
    digitalWrite(pinRelayHeater, LOW);
  } else {
    digitalWrite(pinRelayHeater, HIGH);
  }

  isrCounter += 20; // += 20 because one tick = 20ms
  //set PID output as relais commands
  if (isrCounter > windowSize) {
    isrCounter = 0;
  }

  //run PID calculation
  bPID.Compute();
}

/********************************************************
  Initialize RS232 serial interface
******************************************************/
void initSerial() {
  DEBUGSTART(SERIAL_BAUD); //Initialize serial
  DEBUG_print(F("Firmware version: "));
  DEBUG_println(FIRMWARE_VERSION);
  DEBUG_println(F(" ,booting ..."));
}

/********************************************************
  Init Pins
******************************************************/
void initPins() {
  //Outputs
  DEBUG_print(F("INIT: Initializing Output pins... "));
  pinMode(pinRelayVentil, OUTPUT);
  pinMode(pinRelayPumpe, OUTPUT);
  pinMode(pinRelayHeater, OUTPUT);
  digitalWrite(pinRelayVentil, relayOFF);
  digitalWrite(pinRelayPumpe, relayOFF);
  digitalWrite(pinRelayHeater, LOW);

  //Inputs
  pinMode(pinBrewSwitch, INPUT);
  DEBUG_println(F("done"));
}

/********************************************************
  Define trigger type
******************************************************/
void defineTriggerType() {
  DEBUG_print(F("INIT: Initializing trigger Type (1 = high, 0 = low) ..."));
  if (triggerType)
  {
    relayON = HIGH;
    relayOFF = LOW;
  } else {
    relayON = LOW;
    relayOFF = HIGH;
  }
  DEBUG_print(F("done, type: "));
  DEBUG_println(triggerType);
}

/********************************************************
  Initialize Wifi network
******************************************************/
void initWiFi() {
  if ( Offlinemodus == 1) {
    return;
  }
  unsigned long started = millis();
  DEBUG_print(F("INIT: Initializing WiFi (timeout 20s) ... "));
  WiFi.hostname(hostname);
  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
    would try to act as both a client and an access-point and could cause
    network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);   // needed, otherwise exceptions are triggered \o.O/
  WiFi.setSleepMode(WIFI_NONE_SLEEP); // needed to prevent other bugs
  WiFi.begin(ssid, pass);
  DEBUG_println(F("done"));
  DEBUG_print(F("INIT: Connecting WiFi to: "));
  DEBUG_print(ssid);
  DEBUG_println("...");
  // wait up to 20 seconds for connection:
  while ((WiFi.status() != WL_CONNECTED) && (millis() - started < 20000))
  {
    yield();    //Prevent Watchdog trigger
  }

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect(true);    // disconnect Wifi, erase Wifi credentials
    DEBUG_println(F("ERROR: Failed to connect to WiFi!"));
  } else {
    DEBUG_println(F("done"));
    DEBUG_print(F("INFO: local IP: "));
    DEBUG_println(WiFi.localIP());
    DEBUG_print(F("INFO: MAC address: "));
    DEBUG_println(WiFi.macAddress());
  }
}

void initBlynk() {
  //if (WiFi.status() == WL_CONNECTED) {}
  //DEBUG_println(F("INIT: Initializing Blynk (timeout 20s) ... "));
  //Blynk.config(auth, blynkaddress, blynkport) ;
  //Blynk.connect(20000);   //try blynk connection
}

void initFallback() {
  //if fallback, read all values from blynk and write to eeprom (eeprom creat own function)
}

/*****************************************************
  Initialize OTA update listener if enabled
*****************************************************/
void initOTA() {
#if OTA == true
  if (Offlinemodus == 1) {
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.setHostname(OTAhost);  //  Device name for OTA
    ArduinoOTA.setPassword(OTApass);  //  Password for OTA
    ArduinoOTA.onStart([]() {
      DEBUG_println(F("INFO: Starting OTA update"));
      timer1_disable();   // Disable Timer1 interrupts if OTA is starting to prevent OTA failure
      digitalWrite(pinRelayHeater, LOW); //Stop heating
    });
    ArduinoOTA.onEnd([]() {
      DEBUG_println(F("INFO: OTA updater stopped."));
      timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE); // Enable Timer1 interrupts
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      DEBUG_printf("INFO: OTA Update Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      DEBUG_printf("ERROR: OTA update error [%u]: ", error);
      switch (error) {
        case OTA_AUTH_ERROR:
          Serial.println(F("Auth failed."));
          break;
        case OTA_BEGIN_ERROR:
          Serial.println(F("Begin failed."));
          break;
        case OTA_CONNECT_ERROR:
          Serial.println(F("Connect failed."));
          break;
        case OTA_RECEIVE_ERROR:
          Serial.println(F("Receive failed."));
          break;
        case OTA_END_ERROR:
          Serial.println(F("End failed."));
          break;
      }
      timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE); // Enable Timer1 interrupts
    });
    ArduinoOTA.begin();
    DEBUG_print(F("OTA Ready, "));
    DEBUG_print(F("IP address: "));
    DEBUG_println(WiFi.localIP());
  } else {
    DEBUG_println(F("OTA Failed, no WIFI"));
  }
#endif
}

/********************************************************
  Initialize Display
******************************************************/
void initU8G2() {
  DEBUG_print(F("INIT: Initializing display ... "));
  u8g2.setFont(u8g2_font_IPAandRUSLCD_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
  u8g2.begin();
  DEBUG_println(F("done"));
}

/********************************************************
   Initialize PID
******************************************************/
void initPID() {
  DEBUG_print(F("INIT: Initializing PID ... "));
  bPID.SetSampleTime(windowSize); // sets the period, in Milliseconds, at which the calculation is performed
  bPID.SetOutputLimits(0, windowSize);  //tell the PID to range between 0 and the full window size
  bPID.SetMode(AUTOMATIC);  //turn the PID on
  DEBUG_println(F("done"));
}

/********************************************************
   Initialize temp sensor
******************************************************/
void initTempSensor() {
  DEBUG_print(F("INIT: Initializing Temp Sensor ... "));
  if (TempSensor == 2) {
    temperature = 0;
    Sensor1.getTemperature(&temperature);
    Input = Sensor1.calc_Celsius(&temperature);
  }
  DEBUG_println(F("done"));
}

/********************************************************
  Timer1 ISR - initialization
  TIM_DIV1 = 0,   //80MHz (80 ticks/us - 104857.588 us max)
  TIM_DIV16 = 1,  //5MHz (5 ticks/us - 1677721.4 us max)
  TIM_DIV256 = 3  //312.5Khz (1 tick = 3.2us - 26843542.4 us max)
******************************************************/
void initTimer1ISR() {
  DEBUG_print(F("INIT: Initializing Timer 1 ISR ... "));
  timer1_isr_init();
  timer1_attachInterrupt(onTimer1ISR);
  timer1_enable(TIM_DIV256, TIM_EDGE, TIM_SINGLE);
  timer1_write(6250); // set interrupt time to 20ms
  DEBUG_println(F("done"));
}


/********************************************************
   Initialize remote debuging (e.g. Wifi)
******************************************************/
void initRemoteDebug() {
  DEBUG_print(F("INIT: Initializing remote debugging ... "));
  Debug.begin(hostname, Debug.DEBUG); // Initialize the server (telnet or web socket) of RemoteDebug
  //Debug.setPassword("r3m0t0."); // Password for WiFi client connection (telnet or webapp)  ?
  Debug.setResetCmdEnabled(true); // Enable the reset command
  //Debug.showProfiler(true); // Profiler (Good to measure times, to optimize codes)
  Debug.showColors(true); // Colors
  Debug.setSerialEnabled(true); // if you wants serial echo - only recommended if ESP is plugged in USB
  DEBUG_println(F("done"));
}

/********************************************************
   Initialize crash monitor
******************************************************/
void initCrashMonitor() {
  /*
    DEBUG_print(F("INIT: Initializing crash monitor... "));
    ESPCrashMonitor.disableWatchdog();
    DEBUG_println(F("DONE"));
    ESPCrashMonitor.dump(Serial);
    delay(100);
  */
}

/********************************************************
   Initialize task scheduler
******************************************************/
void initTaskScheduler() {
  DEBUG_print(F("INFO: Starting task scheduler..."));
  tBrew.setId(40);
  tAnalogInput.setId(50);

  lpr.setHighPriorityScheduler(&hpr);

  tCheckTemp.delay();
  tSendBlynk.delay(40);
  tPrintScreen.delay(60);

  //lpr.enableAll(true); // this will recursively enable the higher priority tasks as well
  tCheckTemp.enable();
  tCheckPressure.enable();
  //tCheckWeight.enable();
  tSendBlynk.enable();
  tPrintScreen.enable();

  tBrew.enable();
  //tAnalogInput.enable();

  DEBUG_println(F("done"));
}

void setup() {
  initSerial();
  //initCrashMonitor();
  initRemoteDebug();
  defineTriggerType();
  initPins();
  initU8G2();
  initTempSensor();
  initWiFi();
  initBlynk();
  initOTA();
  initPID();
  initTaskScheduler();

  //Initialisation MUST be at the very end of the init(), otherwise the time comparision in loop() will have a big offset
  unsigned long currentTime = millis();
  previousMillistemp = currentTime;
  windowStartTime = currentTime;
  previousMillisDisplay = currentTime;
  previousMillisBlynk = currentTime;

  initTimer1ISR();

  setupDone = true;
  DEBUG_println(F("INIT: Boot sequence complete."));
  //ESPCrashMonitor.enableWatchdog(ESPCrashMonitorClass::ETimeout::Timeout_2s);
}

void loop() {
  //Only do Wifi stuff, if Wifi is connected
  if (WiFi.status() == WL_CONNECTED && Offlinemodus == 0) {
    ArduinoOTA.handle();
    if (Blynk.connected()) {  // If connected run as normal
      Blynk.run();
      blynkReCnctCount = 0; //reset blynk reconnects if connected
    } else  {
      //checkBlynk();
    }
    wifiReconnects = 0;   //reset wifi reconnects if connected
  } else {
    //checkWifi();
  }
  Debug.handle(); // RemoteDebug handle
  lpr.execute();    // run task scheduler
  //ESPCrashMonitor.iAmAlive();  // Signal that we are alive first to get the full timeout period.


  //check if PID should run or not. If not, set to manuel and force output to zero
  if (pidON == 0 && pidMode == 1) {
    pidMode = 0;
    bPID.SetMode(pidMode);
    Output = 0 ;
  } else if (pidON == 1 && pidMode == 0 && !sensorError && !emergencyStop && backflushState == 10 && brewcounter == 10) {
    pidMode = 1;
    bPID.SetMode(pidMode);
  }


}
