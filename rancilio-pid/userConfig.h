/********************************************************
  Version 1.4 (06.04.2020)
  Config must be configured by the user
******************************************************/

#ifndef _userConfig_H
#define _userConfig_H  

/********************************************************
   Vorab-Konfig
******************************************************/
#define OFFLINEMODUS 0       // 0=Blynk und WLAN wird benötigt 1=OfflineModus (ACHTUNG EINSTELLUNGEN NUR DIREKT IM CODE MÖGLICH)
#define ONLYPID 0            // 1 = Nur PID , 0 = Vollausbau (PID + Preinfussion)
#define TEMPSENSOR 2         // 1 = DS19B20; 2 = TSIC306
#define BREWDETECTION 1      // 0 = off ,1 = Software, 2 = Hardware
#define FALLBACK 1           // 1: fallback auf eeprom Werte, wenn blynk nicht geht 0: deaktiviert
#define TRIGGERTYPE HIGH     // LOW = low trigger, HIGH = high trigger relay
#define OTA true             // true=activate update via OTA
#define PONE 1               // 1 = P_ON_E (normal), 0 = P_ON_M (spezieller PID Modus, ACHTUNG andere Formel zur Berechnung)
#define GRAFANA 1           // 1=Markus grafana Visualisierung. Zugang notwendig, 0=default, aus
#define WIFICINNECTIONDELAY 10000 // delay between reconnects in ms
#define MAXWIFIRECONNECTS 5  // maximum number of reconnects; use -1 to set to maximum ("deactivated")
#define MACHINELOGO 1        // 1 = Rancilio, 2 = Gaggia
#define DELAYHEATING 1       // 1 = delay heating until shot has been pulled, 0 = disable delay

// FeedForwardControl
#define FEEDFORWARDCONTROL 0 // 1 = activate Ffwd control; based on gradient of temperature profile and normal brew or empty brew detection
#define BREWFLOWRATE 1.10       // flowrate (ml/s) if normal shot is pulled
#define EMPTYBREWFLOWRATE 9.666  // flowrate (ml/s) if empty shot is pulled; tested 290g/30s @ cold machine
#define BREWBOARDER_1 150    // boarder to detect normal shot
#define BREWBOARDER_2 300    // boarder to detect empty shot
#define HEATINGPOWER 1000    // in Watt
#define STABILISINGTIME 60000     // waiting time (ms) to give the boiler time for temp stabilisation after brew detection
#define EMPTYCORRECTIONFACTOR  1.25     //factor to correct Q calculation
#define NORMALCORRECTIONFACTOR  1.35     //factor to correct Q calculation

//backflush values
#define FILLTIME 3000       // time in ms the pump is running
#define FLUSHTIME 6000      // time in ms the 3-way valve is open -> backflush
#define MAXFLUSHCYCLES 5      // number of cycles the backflush should run; 0 = disabled

// Wifi
#define AUTH "blynkauthcode"
#define D_SSID "wlanname"
#define PASS "wlanpass"
#define HOSTNAME "rancilio"

// OTA
#define OTAHOST "Rancilio"   // Name to be shown in ARUDINO IDE Port
#define OTAPASS "otapass"    // Password for OTA updates

#define BLYNKADDRESS "blynk.remoteapp.de"         // IP-Address of used blynk server
#define BLYNKPORT 8080
// define BLYNKADDRESS "raspberrypi.local"

//Werte für Waage Offline
#define WEIGHTSETPOINT 30

//PID - Werte für Brüherkennung Offline
#define AGGBKP 50    // Kp
#define AGGBTN 0   // Tn 
#define AGGBTV 20    // Tv

//PID - Werte für Regelung Offline
#define SETPOINT 95  // Temperatur Sollwert
#define AGGKP 69     // Kp
#define AGGTN 399    // Tn
#define AGGTV 0      // Tv
#define STARTKP 100   // Start Kp während Kaltstart
#define STARTTN 0  // Start Tn während Kaltstart
#define STARTTEMP 80 // Temperaturschwelle für deaktivieren des Start Kp

#endif // _userConfig_H
