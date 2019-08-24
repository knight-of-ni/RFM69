// **********************************************************************************************************
// PWM fan controller sketch that works with Moteinos equipped with RFM69W/RFM69HW.
// It uses Timer2, modulated at 25KHz, to control fan speed based on temperature.
//
// **********************************************************************************
// pwm25kHzBegin and pwmDuty function calls from Arduino forum:
// https://forum.arduino.cc/index.php?topic=415167.msg2859274#msg2859274
//
// **********************************************************************************
// Moteino Sketch Framework Copyright Felix Rusu 2016, http://www.LowPowerLab.com/contact
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
#include <SPI.h>           //included in Arduino IDE (www.arduino.cc)
#include <SPIFlash.h>      //get it here: https://github.com/lowpowerlab/spiflash
#include <RFM69.h>         //get it here: https://github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://github.com/lowpowerlab/rfm69
#include <RFM69_OTA.h>     //get it here: https://github.com/lowpowerlab/rfm69
#include <Wire.h>          //included in Arduino IDE (www.arduino.cc)
#include <EEPROM.h>
#include <DHT.h>

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************
#define GATEWAYID   1
#define NODEID      29
#define NETWORKID   53 
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY       RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY      "do8OPePZjx0ztutR" //has to be same 16 characters/bytes on all nodes, not more not less!
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
//*********************************************************************************************
//#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -75
#define ACK_TIME      100  // max # of ms to wait for an ack
//*********************************************************************************************
#define FANPIN 3        // What pin the fan pwm output is connected to
#define DHTPIN 4        // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
#define CHECKINDELAY  60000 // send WeatherShield data every so often (ms)

#define BATT_MONITOR  A7  // Directly wired to battery +V
#define BATT_FORMULA(reading) reading / 50.1 // Value was experimentally determined to be the scale factor that produced the most accurate results

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

#define BLINK_EN                 //uncomment to blink LED on every send
#define SERIAL_EN                //comment out if you don't want any serial output

#ifdef SERIAL_EN
  #define SERIAL_BAUD   115200
  #define DEBUG(input)   {Serial.print(input);}
  #define DEBUGln(input) {Serial.println(input);}
  #define SERIALFLUSH() {Serial.flush();}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
  #define SERIALFLUSH();
#endif
//*****************************************************************************************************************************

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

SPIFlash flash(FLASH_SS, 0xEF30); //WINDBOND 4MBIT flash chip on CS pin D8 (default for Moteino)
DHT dht(DHTPIN, DHTTYPE);

// Global variables
boolean startup=false;
float F;
float H;
float batteryVolts = 12;
char Fstr[10];
char Hstr[10];
char BATstr[10]; // longest battery voltage reading message = 9chars
char sendBuff[54];
byte duty;
byte lotemp;
byte hitemp;
unsigned long lastCheckTime;

char input=0;
byte sendLen;

void setup(void)
{
#ifdef SERIAL_EN
  Serial.begin(SERIAL_BAUD);
#endif
  pinMode(LED, OUTPUT);
  
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

  if (flash.initialize()) flash.sleep();
  
  SERIALFLUSH();

  pinMode(BATT_MONITOR, INPUT);
  dht.begin();
  pinMode(FANPIN, OUTPUT);
  pwm25kHzBegin();
  readEEprom();
  if ( getTemp() )
    calcPWM();

  sprintf(sendBuff, "FanController - transmitting at: %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(sendBuff);

  Blink(LED, 100);Blink(LED, 100);Blink(LED, 100);

  // Send our initial status
  reportStatus(startup=true);
}

void loop() {
  if ( millis() - lastCheckTime < 0 ) // Reset our timer when the millis clock overflows back to 0
    lastCheckTime=0;

  // Run the control loop periodically
  if ( millis() - lastCheckTime > CHECKINDELAY ) { 
    //read DHT Sensor
    if ( getTemp() ) {
      // Update our PWM output
      calcPWM();
      
      // Report back to the gateway
      reportStatus(startup=false);
    }
  lastCheckTime = millis();  
  }

// Check for commands from the gateway
  if (radio.receiveDone()) {
    DEBUG('[');DEBUG(radio.SENDERID);DEBUG("] ");
    for (byte i = 0; i < radio.DATALEN; i++)
      DEBUG((char)radio.DATA[i]);

    if (radio.DATALEN==3)
    {
      if (radio.DATA[0]=='S' && radio.DATA[1]=='T' && radio.DATA[2]=='S') {
        DEBUGln("Refresh status request received from gateway...");
        reportStatus(startup=false);
      }
      if (radio.DATA[0]=='U' && radio.DATA[1]=='P' && radio.DATA[2]=='L') {
        DEBUGln("Lotemp increase request received from gateway...");
        updateLotemp(1);
        calcPWM();
        reportStatus(startup=false);
      }
      if (radio.DATA[0]=='D' && radio.DATA[1]=='N' && radio.DATA[2]=='L') {
        DEBUGln("Lotemp decrease request received from gateway...");
        updateLotemp(-1);
        calcPWM();
        reportStatus(startup=false);
      }
      if (radio.DATA[0]=='U' && radio.DATA[1]=='P' && radio.DATA[2]=='H') {
        DEBUGln("Hitemp increase request received from gateway...");
        updateHitemp(1);
        calcPWM();
        reportStatus(startup=false);
      }
      if (radio.DATA[0]=='D' && radio.DATA[1]=='N' && radio.DATA[2]=='H') {
        DEBUGln("Hitemp decrease request received from gateway...");
        updateHitemp(-1);
        calcPWM();
        reportStatus(startup=false);
      }
    }
  }
}

void Blink(byte PIN, byte DELAY_MS) {
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS/2);
  digitalWrite(PIN,LOW);
  delay(DELAY_MS/2);  
}

// Obtained from https://forum.arduino.cc/index.php?topic=415167.msg2859274#msg2859274
void pwm25kHzBegin() {
  TCCR2A = 0;                               // TC2 Control Register A
  TCCR2B = 0;                               // TC2 Control Register B
  TIMSK2 = 0;                               // TC2 Interrupt Mask Register
  TIFR2 = 0;                                // TC2 Interrupt Flag Register
  TCCR2A |= (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);  // OC2B cleared/set on match when up/down counting, fast PWM
  TCCR2B |= (1 << WGM22) | (1 << CS21);     // prescaler 8
  OCR2A = 79;                               // TOP overflow value (Hz)
  OCR2B = 0;
}

void pwmDuty(byte ocrb) {
  OCR2B = ocrb;                             // PWM Width (duty)
}

void dutyCycle(byte range) {
  byte ocrb = map(range, 1.25, 100, 0, 79);
  pwmDuty(ocrb);
}

boolean getTemp() { // get the tempurature and humidty from the DHT sensor
  boolean stat = false;

  for (byte attempts = 0; attempts < 3; attempts++) {
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    H = dht.readHumidity();
    // Read temperature as Celsius
    F = dht.readTemperature(true);

    if (isnan(H) || isnan(F)) {
      DEBUGln("Failed to read from DHT sensor!");
      delay(500);
    } else {
      dtostrf(F, 3,2, Fstr);
      dtostrf(H, 3,2, Hstr);
      DEBUG("DHT Sensor reported a temperature of "); DEBUGln(Fstr);
      stat = true;
      break;
    }
  }
return stat;    
}

// Calculate the PWM duty
void calcPWM() {     
  if ( F < lotemp ) { // Turn off the fans if we are below our min value
    duty=0;
  } else if ( F > hitemp ) { // Turn on the fans 100% if we are above our max value
    duty=100;
  } else {
    duty= map(F, lotemp, hitemp, 30, 100); // map a duty cycle from 30-100 that corresponds to the lotemp-hitemp range.
  }
  DEBUG("Setting PWM Duty Cycle to "); DEBUGln(duty);
  dutyCycle(duty);  
}

// Get our low and high temp setpoints from the eeprom
void readEEprom() {
  lotemp = EEPROM.read(0); // 0-255 degrees F
  hitemp = EEPROM.read(1); // 0-255 degrees F

  if ( hitemp < lotemp ) // shouldn't happen
    hitemp = lotemp;

  // Pick some sensible defaults if eeprom has not yet been written to
  if (( hitemp == 0 && lotemp == 0 ) || ( hitemp == 255 && lotemp == 255 )) {
    lotemp = 80;
    hitemp = 95;
  }
}

// Write a new low temp setpoint to the eeprom
void updateLotemp(int delta) {
  
  if ( lotemp + delta < 0 ) {
    lotemp = 255;
  } else if ( lotemp + delta > 255 ) {
    lotemp = 0;
  } else if ( lotemp + delta > hitemp ) {
    lotemp = hitemp;
  } else {
    lotemp = lotemp + delta;
  }
  
  EEPROM.write(0,lotemp);
}

// Write a new hi temp setpoint to the eeprom
void updateHitemp(int delta) {
  
  if ( hitemp + delta < 0 ) {
    hitemp = 255;
  } else if ( hitemp + delta > 255 ) {
    hitemp = 0;
  } else if ( hitemp + delta < lotemp ) {
    hitemp = lotemp;
  } else {
    hitemp = hitemp + delta;
  }
  
  EEPROM.write(1,hitemp);
}

// Send data to the gateway
void reportStatus(boolean startup) {

  if (startup) {
    sprintf(sendBuff, "START HIT:%u LOT:%u DTY:%u F:%s H:%s", hitemp, lotemp, duty, Fstr, Hstr);
  } else {
    checkBattery();
    sprintf(sendBuff, "HIT:%u LOT:%u DTY:%u F:%s H:%s BAT:%sv", hitemp, lotemp, duty, Fstr, Hstr, BATstr);
  }
  sendLen = strlen(sendBuff);

  if (radio.sendWithRetry(GATEWAYID, sendBuff, sendLen, 5, ACK_TIME) ) {
    DEBUGln("ACK: OK");
  } else {
    DEBUGln("ACK: NOT OK");
  }
      
  #ifdef BLINK_EN
    Blink(LED, 5);
  #endif
}

void checkBattery() { // Read the battery voltage then update BATstr
  unsigned int readings=0;
  int strlength=0;
  for (byte i=0; i<10; i++) //take 10 samples, and average
    readings+=analogRead(BATT_MONITOR);
  batteryVolts = BATT_FORMULA(readings / 10.0);
  if ( batteryVolts < 10 ) {
    strlength = 3;
  } else {
    strlength = 4;
  }
  dtostrf(batteryVolts, strlength, 1, BATstr); //update the BATStr, which gets sent every reportstatus cycle
}


