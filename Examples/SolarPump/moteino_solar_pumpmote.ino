// Solar PumpMote sketch for solar powered irrigation pump control
// By Andrew Bauer <zonexpertconsulting@outlook.com>
//
// Portions of this sketch based on work by Felix Rusu of LowerPowerLab.com
// http://lowpowerlab.com/
//
// **********************************************************************************
// Copyright Felix Rusu of LowPowerLab.com, 2016
// RFM69 library and sample code by Felix Rusu - lowpowerlab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
//
//
#include <SPI.h>      //comes with Arduino IDE (www.arduino.cc)
#include <SPIFlash.h> //get it here: https://www.github.com/lowpowerlab/spiflash
#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>//get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_OTA.h> //get it here: https://github.com/lowpowerlab/rfm69
#include <EEPROM.h>

//
//****************************************************************************************************************
//**** IMPORTANT RADIO SETTINGS - YOU MUST CHANGE/CONFIGURE TO MATCH YOUR HARDWARE TRANSCEIVER CONFIGURATION! ****
//****************************************************************************************************************
#define GATEWAYID   1
#define NODEID      28
#define NETWORKID   53
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY       RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY      "do8OPePZjx0ztutR" //has to be same 16 characters/bytes on all nodes, not more not less!
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!

//
//*****************************************************************************************************************************
#define ENABLE_ATC      //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI        -75
#define ACK_TIME      100  // max # of ms to wait for an ack

//
//*****************************************************************************************************************************
#define START_PUMP    A0 // Output to activate the pump
#define BATT_MONITOR  A1  // Directly wired to battery +V
#define SOLAR_MONITOR A2  // Ambient light sensor
#define TRBL_MONITOR  A3  // Monitors the load terminals of the charge controller
                          // The charge controller will turn this off in the event of trouble
#define PUMP_MONITOR  A4  // Inicates 12v is being applied to the pump motor                          

#define PUMP_SCHEDULE 3 // Default pump cycle is every 3 hours
#define PUMP_DURATION 15    // Default pump runtime is 15 minutes
#define LOW_BATT      11.2       // Volts below which we will not try to start the pump
#define CHECKINDELAY     60000   // how often to report in
#define MIN_LIGHT_THRESHOLD 50   // the minimum analog read from the light senser, above which we call daytime

//
//*****************************************************************************************************************************
#define LED                  9   //pin connected to onboard LED
#define LED_PULSE_PERIOD  5000   //5s seems good value for pulsing/blinking (not too fast/slow)
#define SERIAL_BAUD     115200
#define SERIAL_EN                //comment out if you don't want any serial output

#define BATT_FORMULA(reading) reading / 51.57 // Value was experimentally determined to be the scale factor that produced the most accurate results

#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); delay(1);}
  #define DEBUGln(input) {Serial.println(input); delay(1);}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

#define FLASH_SS      8 // and FLASH SS on D8 on regular Moteinos (D23 on MoteinoMEGA)
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

// Global program variables
boolean inTrouble=false;
boolean DayTime=false;
boolean PumpStatus=false;
boolean LowBat=false;
boolean startup=false;
boolean chkBat=false;
boolean schedule=false;
boolean Power=true;
unsigned long pumpSchedule;
unsigned long pumpRunTime;
unsigned long pumpDuration=0;
unsigned long lastStartTime=0;
unsigned long lastBatCheck=0;
unsigned long lastCheckinTime=0;
float batteryVolts = 0;
float lightLevel = 0;
char sendBuff[54];
char statusText[20];
char BATstr[10]; // longest battery voltage reading message = 9chars
byte sendLen;
byte pumpSchedule_hrs;
byte pumpRunTime_min;

// Run once when the arduino starts up
void setup(void) {

#ifdef SERIAL_EN
  Serial.begin(SERIAL_BAUD);
#endif

// Configure our I/O points
  pinMode(LED, OUTPUT);
  pinMode(START_PUMP, OUTPUT);
  pinMode(BATT_MONITOR, INPUT);
  pinMode(SOLAR_MONITOR, INPUT);
  pinMode(TRBL_MONITOR, INPUT_PULLUP);
  pinMode(PUMP_MONITOR, INPUT_PULLUP);

  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
  radio.encrypt(ENCRYPTKEY);

#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
#endif

  sprintf(sendBuff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(sendBuff);
  
#ifdef ENABLE_ATC
  DEBUGln("RFM69_ATC Enabled (Auto Transmission Control)\n");
#endif

if (flash.initialize()) flash.sleep(); //if Moteino has FLASH-MEM, make sure it sleeps

readEEprom();  // Get the pump schedule values from our eeprom

// Get the initial status of our input points
  reportStatus(startup=true, chkBat=false, schedule=false); 
}

// Our main loop
void loop() {

  if ( millis() - lastStartTime < 0 ) // Reset our timer when the millis clock overflows back to 0
    lastStartTime=0;

  if ( millis() - pumpDuration < 0 ) // Reset our timer when the millis clock overflows back to 0
    pumpDuration=0;

  // When the pump is running, we compare against the pumpDuration timer
  // When the pump is stopped, we compare against the lastStartTime timer
  if ( PumpStatus ) {
    if ( millis() - pumpDuration >  pumpRunTime ) {  // Turn off the pump if it has run long enough
      digitalWrite(START_PUMP, LOW);
      reportStatus(startup=false, chkBat=false, schedule=false);
      lastStartTime=millis();
      }
  } else {
    if ( Power && OKtoStart() && millis() - lastStartTime >  pumpSchedule ) { // Turn on the pump if it is time
      digitalWrite(START_PUMP, HIGH);
      reportStatus(startup=false, chkBat=false, schedule=false);
      pumpDuration=millis();
    }
  }

  // Our periodic checkin with the gateway
  if (millis()-lastCheckinTime > CHECKINDELAY) {
    reportStatus(startup=false, chkBat=true, schedule=false);
  }

// Check for commands from the gateway
  if (radio.receiveDone()) {
    DEBUG('[');DEBUG(radio.SENDERID);DEBUG("] ");
    for (byte i = 0; i < radio.DATALEN; i++)
      DEBUG((char)radio.DATA[i]);

    if (radio.DATALEN==3)
    {
      //check for an OPEN/CLOSE/STATUS request
      if (radio.DATA[0]=='R' && radio.DATA[1]=='U' && radio.DATA[2]=='N') {
        DEBUGln("Start pump request received from gateway...");
        if ( !inTrouble && !LowBat ) {
          digitalWrite(START_PUMP, HIGH);
          pumpDuration=millis();
        }
        reportStatus(startup=false, chkBat=true, schedule=false);
      }
      if (radio.DATA[0]=='S' && radio.DATA[1]=='T' && radio.DATA[2]=='P') {
        DEBUGln("Stop pump request received from gateway...");
        digitalWrite(START_PUMP, LOW);
        reportStatus(startup=false, chkBat=true, schedule=false);
        lastStartTime=millis();
      }
      if (radio.DATA[0]=='S' && radio.DATA[1]=='T' && radio.DATA[2]=='S') {
        DEBUGln("Refresh status request received from gateway...");
        reportStatus(startup=false, chkBat=true, schedule=false);
      }
      if (radio.DATA[0]=='U' && radio.DATA[1]=='P' && radio.DATA[2]=='1') {
        DEBUGln("Schedule increase request received from gateway...");
        updateSchedule(1);
        reportStatus(startup=false, chkBat=false, schedule=true);
      }
      if (radio.DATA[0]=='D' && radio.DATA[1]=='N' && radio.DATA[2]=='1') {
        DEBUGln("Schedule decrease request received from gateway...");
        updateSchedule(-1);
        reportStatus(startup=false, chkBat=false, schedule=true);
      }
      if (radio.DATA[0]=='U' && radio.DATA[1]=='P' && radio.DATA[2]=='5') {
        DEBUGln("Pump duration increase request received from gateway...");
        updateDuration(5);
        reportStatus(startup=false, chkBat=false, schedule=true);
      }
      if (radio.DATA[0]=='D' && radio.DATA[1]=='N' && radio.DATA[2]=='5') {
        DEBUGln("Pump duration decrease request received from gateway...");
        updateDuration(-5);
        reportStatus(startup=false, chkBat=false, schedule=true);
      }
      if (radio.DATA[0]=='P' && radio.DATA[1]=='W' && radio.DATA[2]=='R') {
        DEBUGln("Toggle power request received from gateway...");
        TogglePower();
        reportStatus(startup=false, chkBat=false, schedule=false);
      }      
    }
  }
}

// Passing startup=True to this function causes it to return full telemtetry to the gateway
// Passing startup=False to this functions causes it to return just the word "START" to the gateway
// Passing chkBat=True will check the battery voltage
// Passing chkBat=False will skip the battery check
void reportStatus(boolean startup, boolean chkBat, boolean schedule) {
  readInputs(chkBat);
  
  if (startup) { // report to the gateway we just started up
    sprintf(sendBuff, "START SCHED:%u DUR:%u",
                      pumpSchedule_hrs,
                      pumpRunTime_min
            );    
  } else if (schedule) {
    sprintf(sendBuff, "SCHED:%u DUR:%u",
                      pumpSchedule_hrs,
                      pumpRunTime_min
            );    
  } else {
    sprintf(sendBuff, "BAT:%s PUMP:%s STATUS:%s MODE:%s",
                      BATstr,
                      Power==LOW ? "OFF" : PumpStatus==LOW ? "STOPPED" : PumpStatus==HIGH ? "RUNNING" : "UNKNOWN",
                      inTrouble==LOW ? "NORMAL" : inTrouble==HIGH ? "TROUBLE" : "UNKNOWN",
                      DayTime==LOW ? "NIGHT" : DayTime==HIGH ? "DAY" : "UNKNOWN"
            );
  }
    
  sendLen = strlen(sendBuff);
  DEBUG("Solar Pump Status: "); DEBUGln(sendBuff);
  if (radio.sendWithRetry(GATEWAYID, sendBuff, sendLen, 5, ACK_TIME) ) {
    DEBUGln("ACK: OK");
  } else {
    DEBUGln("ACK: NOT OK");
  }
  
  DEBUGln("Sent checkin data");
  lastCheckinTime = millis();

  DEBUG("Value of OKtoStart: "); DEBUGln(OKtoStart());
  DEBUG("Value of millis: "); DEBUGln(millis());
  DEBUG("Value of lastStartTime: "); DEBUGln(lastStartTime);
  DEBUG("Value of pumpSchedule: "); DEBUGln(pumpSchedule);
  DEBUG("Value of pumpDuration: "); DEBUGln(pumpDuration);
  DEBUG("Value of pumpRunTime: "); DEBUGln(pumpRunTime);
}

// Read the battery voltage then update BATstr
void checkBattery() {
  unsigned int readings=0;
  int strlength=0;
  for (byte i=0; i<10; i++) //take 10 samples, and average
    readings+=analogRead(BATT_MONITOR);
  batteryVolts = BATT_FORMULA(readings / 10.0);
  LowBat = batteryVolts < LOW_BATT;
  if ( batteryVolts < 10 ) {
    strlength = 3;
  } else {
    strlength = 4;
  }
  dtostrf(batteryVolts, strlength, 1, BATstr); //update the BATStr, which gets sent every reportstatus cycle
}

// Read the Adafruit ambient light sensor
float readLightSensor() {
  unsigned int readings=0;
  for (byte i=0; i<10; i++) //take 10 samples, and average
    readings+=analogRead(SOLAR_MONITOR);

  lightLevel = readings / 10.0;

  return lightLevel;
}

boolean OKtoStart() {
  return !inTrouble && DayTime && !LowBat;
}

// Read the current status of our input points
// Passing True to this function causes it to check the battery voltage
// Passing False to this functions causes it to skip battery voltage check
void readInputs(boolean chkBat) {
  if ( chkBat )
    checkBattery();
  inTrouble=digitalRead(TRBL_MONITOR);
  DayTime=(readLightSensor() > MIN_LIGHT_THRESHOLD);
  PumpStatus=!digitalRead(PUMP_MONITOR);

  if ( inTrouble )  // For safety, force the pump output off if we detect trouble
    digitalWrite(START_PUMP, LOW);
}

// Get our pump schedule values from the eeprom
void readEEprom() {
  pumpSchedule_hrs = EEPROM.read(0); // max 255 hours
  pumpRunTime_min = EEPROM.read(1);  // max 255 minutes

  DEBUG("Read value of pumpSchedule_hrs from EEPROM: "); DEBUGln(pumpSchedule_hrs);
  DEBUG("Read value of pumpRunTime_min from EEPROM: "); DEBUGln(pumpRunTime_min);
  
  if ( pumpSchedule_hrs > 0 && pumpSchedule_hrs <= 255 ) {
    pumpSchedule =  pumpSchedule_hrs * 3600000;
  } else {
    pumpSchedule_hrs = PUMP_SCHEDULE;
    pumpSchedule = pumpSchedule_hrs * 3600000;
  }
  
  if ( pumpRunTime_min > 0 && pumpRunTime_min <= 255 ) {
    pumpRunTime = pumpRunTime_min * 60000;
  } else {
    pumpRunTime_min = PUMP_DURATION;
    pumpRunTime = pumpRunTime_min * 60000;
  }

  DEBUG("Pump schedule in milliseconds: "); DEBUGln(pumpSchedule);
  DEBUG("Pump duration in milliseconds:: "); DEBUGln(pumpRunTime);

}

// Store our new pump schedule to eeprom
void updateSchedule(int delta) {
  
  if ( pumpSchedule_hrs + delta <= 0 ) {
    pumpSchedule_hrs = 255;
  } else if ( pumpSchedule_hrs + delta > 255 ) {
    pumpSchedule_hrs = 1;  
  } else {
    pumpSchedule_hrs = pumpSchedule_hrs + delta;
  }

  EEPROM.write(0,pumpSchedule_hrs);
  pumpSchedule = pumpSchedule_hrs * 3600000;
}

// Store our new pump duration to eeprom
void updateDuration(int delta) {

  if ( pumpRunTime_min + delta <= 0 ) {
    pumpRunTime_min = 255;
  } else if ( pumpRunTime_min + delta > 255 ) {
    pumpRunTime_min = 5;  
  } else {
    pumpRunTime_min = pumpRunTime_min + delta;
  }
  
  EEPROM.write(1,pumpRunTime_min);
  pumpRunTime = pumpRunTime_min * 60000;
}

// Called when we receive a PWR command from the gateway
void TogglePower() {
  Power = !Power;

  if ( !Power )
    digitalWrite(START_PUMP, LOW); 
}

