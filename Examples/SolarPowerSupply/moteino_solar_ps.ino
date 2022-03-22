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
#include <SPI.h>      //comes with Arduino IDE (www.arduino.cc)
#include <SPIFlash.h> //get it here: https://www.github.com/lowpowerlab/spiflash
#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>//get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_OTA.h> //get it here: https://github.com/lowpowerlab/rfm69
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
                      //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
//#include <Wire.h>     //comes with Arduino

//****************************************************************************************************************
//**** IMPORTANT RADIO SETTINGS - YOU MUST CHANGE/CONFIGURE TO MATCH YOUR HARDWARE TRANSCEIVER CONFIGURATION! ****
//****************************************************************************************************************
#define NODEID        11    //unique for each node on same network
#define NETWORKID     5  //the same on all nodes that talk to each other
#define GATEWAYID     1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
#define ENCRYPTKEY    "somesecrettexttt" //exactly the same 16 characters/bytes on all nodes!
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -70

#define SEND_LOOPS    15 //send data this many sleep loops (15 loops of 8sec cycles = 120sec ~ 2 minutes)
#define ACK_TIME      50  // max # of ms to wait for an ack
#define POWER_OUT     A0 // Output to activate +5V power
#define BATT_MONITOR  A1  // Directly wired to battery +V
#define BATT_FORMULA(reading) reading / 51.39 // Value was experimentally determined to be the scale factor that produced the most accurate results
#define BATT_READ_LOOPS  SEND_LOOPS*10  // read and report battery voltage every this many sleep cycles (ex 30cycles * 8sec sleep = 240sec/4min). For 450 cycles you would get ~1 hour intervals between readings

//#define LED           9 // Moteinos have LEDs on D9
//#define SERIAL_EN             //comment this out when deploying to an installed Mote to save a few KB of sketch size
#define SERIAL_BAUD    115200
#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); delay(1);}
  #define DEBUGln(input) {Serial.println(input); delay(1);}
  #define DEBUGFlush() { Serial.flush(); }
#else
  #define DEBUG(input);
  #define DEBUGln(input);
  #define DEBUGFlush();
#endif

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

#define FLASH_SS      8 // and FLASH SS on D8 on regular Moteinos (D23 on MoteinoMEGA)
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

// Define variables
boolean PWRStatus=false;
float batteryVolts = 3;
char BATstr[10]; // longest battery voltage reading message = 9chars
char sendBuff[50];
byte sendLen;
byte sendLoops=0;
byte battReadLoops=0;

void setup() { // Setup routine called once during startup
#ifdef SERIAL_EN  
  Serial.begin(SERIAL_BAUD);
#endif

  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
  radio.encrypt(ENCRYPTKEY);

//Auto Transmission Control - dials down transmit power to save battery (-100 is the noise floor, -90 is still pretty good)
//For indoor nodes that are pretty static and at pretty stable temperatures (like a MotionMote) -90dBm is quite safe
//For more variable nodes that can expect to move or experience larger temp drifts a lower margin like -70 to -80 would probably be better
//Always test your ATC mote in the edge cases in your own environment to ensure ATC will perform as you expect
#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
#endif

  sprintf(sendBuff, "\nSolar Power Supply - Transmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(sendBuff);
  
#ifdef ENABLE_ATC
  DEBUGln("RFM69_ATC Enabled (Auto Transmission Control)\n");
#endif

  if (flash.initialize()) flash.sleep(); //if Moteino has FLASH-MEM, make sure it sleeps

  pinMode(POWER_OUT, OUTPUT);
  digitalWrite(POWER_OUT, LOW);
  pinMode(BATT_MONITOR, INPUT);
  #ifdef LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  #endif
  reportStatus(true);
}

void loop() { //Main loop
  if (battReadLoops--<=0) { //only read battery every BATT_READ_LOOPS cycles
    readBattery();
    battReadLoops = BATT_READ_LOOPS-1;
  }

  if (sendLoops--<=0) { //send readings every SEND_LOOPS
    sendLoops = SEND_LOOPS-1;
    reportStatus(false);
  }

  gotoSleep(SLEEP_8S);
  DEBUGln("WAKEUP"); DEBUG(" battReadLoops: "); DEBUGln(battReadLoops); DEBUG(" SendLoops: "); DEBUGln(sendLoops);
}

void PWRcontrol(boolean PWRstate) { // Toggle the state of power output
  #ifdef LED
  digitalWrite(LED, PWRstate);
  #endif
  digitalWrite(POWER_OUT, PWRstate);
  PWRStatus=PWRstate;
}

void readBattery() { // Read the battery voltage then update BATstr
  unsigned int readings=0;
  for (byte i=0; i<10; i++) //take 10 samples, and average
    readings+=analogRead(BATT_MONITOR);
  batteryVolts = BATT_FORMULA(readings / 10.0);
  dtostrf(batteryVolts, 3,2, BATstr); //update the BATStr, which gets sent every reportstatus cycle
}

void reportStatus(boolean startup) { // xmit the status of the moteino
  if (startup) { // report to the gateway we just started up
    sprintf(sendBuff, "START");
    startup = false;
  } else {
    readBattery();
    sprintf(sendBuff, "POWER:%s BAT:%s",
                      PWRStatus==LOW ? "OFF" : PWRStatus==HIGH ? "ON" : "UNKNOWN",
                      BATstr
           );    
  }

  sendLen = strlen(sendBuff);
  DEBUG("Moteino Status: "); DEBUGln(sendBuff);

  if (radio.sendWithRetry(GATEWAYID, sendBuff, sendLen, 3, ACK_TIME) ) {
    DEBUGln("ACK: OK");
    if (radio.DATALEN) processCommand(); // the gateway sent us commmand(s) with the ACK packet.
  } else {
    DEBUGln("ACK: NOT OK");
  }
  DEBUGln("Sent checkin data");
}

void gotoSleep(period_t period) {
  DEBUGFlush();
  radio.sleep();
  LowPower.powerDown(period, ADC_OFF, BOD_OFF); 
}

void processCommand() {
  byte ackLen = 0;
  char ackData[radio.DATALEN];
    
  DEBUG("Radio Data received from gateway: "); DEBUG("["); DEBUG(radio.SENDERID);DEBUG("] ");
  for (byte i = 0; i < radio.DATALEN; i++) DEBUG((char)radio.DATA[i]); DEBUGln();

  // Process data received from the gateway ACK packet
  if (strstr(radio.DATA, "POWER:ON")) {
    PWRcontrol(HIGH);
    sprintf(ackData, "POWER:ON");
  } else if (strstr(radio.DATA, "POWER:OFF")) {
    PWRcontrol(LOW);
    sprintf(ackData, "POWER:OFF");
  } else { // If we don't know what we received just echo it back as-is
    sprintf(ackData, radio.DATA);
  }
      
  ackLen = strlen(ackData);
  radio.sendACK(ackData, ackLen);
}
