/**
  WxUnoGPRS - Weather Station for Arduino UNO, GPRS connected

  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of Weather Station.

  WxUno is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by the Free
  Software Foundation, either version 3 of the License, or (at your option) any
  later version.

  WxUno is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  WxUno.  If not, see <http://www.gnu.org/licenses/>.


  WiFi connected weather station, reading the athmospheric sensor BME280 and
  the illuminance sensor TSL2561 and publishing the measured data along with
  various local telemetry.
*/

// The DEBUG and DEVEL flag
//#define DEBUG
//#define DEVEL

// The sensors are connected to I2C
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// GPRS
#include <M590Client.h>
#include <SoftwareSerial.h>

// NTP
#include <TimeLib.h>

// Device name
char *NODENAME = "WxUnoGPRS";
char *VERSION = "1.1";
bool  PROBE = true;    // True if the station is being probed

// GPRS credentials
char apn[]  = "internet.simfony.net";
char user[] = "";
char pass[] = "";

// APRS parameters
const char *aprsServer = "cwop5.aprs.net";
const int   aprsPort = 14580;
const char *aprsCallSign = "FW0728";
const char *aprsPassCode = "-1";
const char *aprsLocation = "2455.29N/04527.08E_";
const char *aprsPath = ">APRS,TCPIP*:";
const int   altMeters = 83; // 282
const long  altFeet = (long)(altMeters * 3.28084);
float altCorr = pow((float)(1.0 - 2.25577e-5 * altMeters), (float)(-5.25578));
// Reports and measurements
const int   aprsRprtHour = 10;  // Number of APRS reports per hour
const int   aprsMsrmMax = 3;    // Number of measurements per report (keep even)
int         aprsMsrmCount = 0;  // Measurements counter
int         aprsTlmSeq = 0;     // Telemetry sequence mumber
// Telemetry bits
char        aprsTlmBits = B00000000;

// The APRS connection client
SoftwareSerial SerialAT(2, 3); // RX, TX
M590Drv GPRS_Modem;
M590Client APRS_Client(&GPRS_Modem);
IPAddress ip;
String pkt = "";

// Statistics (median filter for the last 3 values)
int rmTemp[4];
int rmPres[4];
int rmVcc[4];
int rmA0[4];
int rmA1[4];

// Sensors
const unsigned long snsDelay = 3600000UL / (aprsRprtHour * aprsMsrmMax);
unsigned long snsNextTime = 0UL;  // The next time to read the sensors
Adafruit_BMP280 atmo;             // The athmospheric sensor
bool atmo_ok = false;             // The athmospheric sensor flag

/*
  Simple median filter
  2014-03-25: started by David Cary
*/
int mdnOut(int *buf) {
  if (buf[0] < 3) return buf[3];
  else {
    int the_max = max(max(buf[1], buf[2]), buf[3]);
    int the_min = min(min(buf[1], buf[2]), buf[3]);
    // unnecessarily clever code
    int the_median = the_max ^ the_min ^ buf[1] ^ buf[2] ^ buf[3];
    return the_median;
  }
}

void mdnIn(int *buf, int x) {
  if (buf[0] < 3) buf[0]++;
  buf[1] = buf[2];
  buf[2] = buf[3];
  buf[3] = x;
}

void aprsSend(const char *pkt) {
#ifdef DEBUG
  Serial.print(pkt);
#endif
  APRS_Client.print(pkt);
}

void aprsSend(const __FlashStringHelper *pkt) {
#ifdef DEBUG
  Serial.print(pkt);
#endif
  APRS_Client.print(pkt);
}

void aprsSend(String &pkt) {
#ifdef DEBUG
  Serial.print(pkt.c_str());
#endif
  APRS_Client.write((uint8_t *)pkt.c_str(), strlen(pkt.c_str()));
}

void aprsSendCRLF() {
#ifdef DEBUG
  Serial.print(F("\r\n"));
#endif
  APRS_Client.print(F("\r\n"));
}

void aprsSendHeader(const char *sep) {
  aprsSend(aprsCallSign);
  aprsSend(aprsPath);
  aprsSend(sep);
}

/**
  Return time in APRS format: DDHHMMz
*/
char *aprsTime() {
  time_t moment = now();
  char buf[8];
  sprintf_P(buf, PSTR("%02d%02d%02dz"), day(moment), hour(moment), minute(moment));
  return buf;
}

/**
  Send APRS authentication data
  user FW0690 pass -1 vers WxUno 0.2"
*/
void aprsAuthenticate() {
  pkt  = F("user ");
  pkt += aprsCallSign;
  pkt += F(" pass ");
  pkt += aprsPassCode;
  pkt += F(" vers ");
  pkt += NODENAME;
  pkt += F(" ");
  pkt += VERSION;
  pkt += F("\r\n");
  aprsSend(pkt);
  //aprsSend(F("user "));
  //aprsSend(aprsCallSign);
  //aprsSend(F(" pass "));
  //aprsSend(aprsPassCode);
  //aprsSend(F(" vers "));
  //aprsSend(NODENAME);
  //aprsSend(F(" "));
  //aprsSend(VERSION);
  //aprsSendCRLF();
}

/**
  Send APRS weather data, then try to get the forecast
  FW0690>APRS,TCPIP*:@152457h4427.67N/02608.03E_.../...g...t044h86b10201L001WxUno

  @param temp temperature
  @param hmdt humidity
  @param pres athmospheric pressure
  @param lux illuminance
*/
void aprsSendWeather(int temp, int hmdt, int pres, int lux) {
  pkt  = aprsCallSign;
  pkt += aprsPath;
  pkt += F("@");
  // Compose the APRS packet
  pkt += aprsTime();
  pkt += aprsLocation;
  // Wind
  pkt += F(".../...g...");
  // Temperature
  if (temp >= -460) { // 0K in F
    char buf[5];
    sprintf_P(buf, PSTR("t%03d"), temp);
    pkt += buf;
  }
  else {
    pkt += F("t...");
  }
  // Humidity
  if (hmdt >= 0) {
    if (hmdt == 100) {
      pkt += F("h00");
    }
    else {
      char buf[5];
      sprintf_P(buf, PSTR("h%02d"), hmdt);
      pkt += buf;
    }
  }
  // Athmospheric pressure
  if (pres >= 0) {
    char buf[7];
    sprintf_P(buf, PSTR("b%05d"), pres);
    pkt += buf;
  }
  // Illuminance, if valid
  if (lux >= 0) {
    char buf[5];
    sprintf_P(buf, PSTR("L%02d"), (int)(lux * 0.0079));
    pkt += buf;
  }
  // Comment (device name)
  pkt += NODENAME;
  pkt += F("\r\n");
  aprsSend(pkt);
}
void aprsSendWeather_OLD(int temp, int hmdt, int pres, int lux) {
  aprsSendHeader("@");
  // Compose the APRS packet
  aprsSend(aprsTime());
  aprsSend(aprsLocation);
  // Wind
  aprsSend(F(".../...g..."));
  // Temperature
  if (temp >= -460) { // 0K in F
    char buf[5];
    sprintf_P(buf, PSTR("t%03d"), temp);
    aprsSend(buf);
  }
  else {
    aprsSend(F("t..."));
  }
  // Humidity
  if (hmdt >= 0) {
    if (hmdt == 100) {
      aprsSend(F("h00"));
    }
    else {
      char buf[5];
      sprintf_P(buf, PSTR("h%02d"), hmdt);
      aprsSend(buf);
    }
  }
  // Athmospheric pressure
  if (pres >= 0) {
    char buf[7];
    sprintf_P(buf, PSTR("b%05d"), pres);
    aprsSend(buf);
  }
  // Illuminance, if valid
  if (lux >= 0) {
    char buf[5];
    sprintf_P(buf, PSTR("L%03d"), (int)(lux * 0.0079));
    aprsSend(buf);
  }
  // Comment (device name)
  aprsSend(NODENAME);
  aprsSendCRLF();
}

/**
  Send APRS telemetry and, periodically, send the telemetry setup
  FW0690>APRS,TCPIP*:T#517,173,062,213,002,000,00000000

  @param vcc voltage
  @param rssi wifi level
  @param heap free memory
  @param luxVis raw visible illuminance
  @param luxIrd raw infrared illuminance
  @bits digital inputs
*/
void aprsSendTelemetry(int a0, int a1, int rssi, int vcc, int temp, byte bits) {
  // Increment the telemetry sequence number, reset it if exceeds 999
  if (++aprsTlmSeq > 999) aprsTlmSeq = 0;
  // Send the telemetry setup on power up (first minutes) or if the sequence number is 0
  if ((aprsTlmSeq == 0) or (millis() < snsDelay + snsDelay)) aprsSendTelemetrySetup();
  // Compose the APRS packet
  pkt  = aprsCallSign;
  pkt += aprsPath;
  pkt += F("T");
  char buf[40];
  sprintf_P(buf, PSTR("#%03d,%03d,%03d,%03d,%03d,%03d,"), aprsTlmSeq, a0, a1, rssi, vcc, temp);
  pkt += buf;
  char bbuf[10];
  itoa(bits, bbuf, 2);
  pkt += bbuf;
  pkt += F("\r\n");
  aprsSend(pkt);
}
void aprsSendTelemetry_OLD(int a0, int a1, int rssi, int vcc, int temp, byte bits) {
  // Increment the telemetry sequence number, reset it if exceeds 999
  if (++aprsTlmSeq > 999) aprsTlmSeq = 0;
  // Send the telemetry setup on power up (first minutes) or if the sequence number is 0
  if ((aprsTlmSeq == 0) or (millis() < snsDelay + snsDelay)) aprsSendTelemetrySetup();
  // Compose the APRS packet
  aprsSendHeader("T");
  char buf[40];
  sprintf_P(buf, PSTR("#%03d,%03d,%03d,%03d,%03d,%03d,"), aprsTlmSeq, a0, a1, rssi, vcc, temp);
  aprsSend(buf);
  char bbuf[10];
  itoa(bits, bbuf, 2);
  aprsSend(bbuf);
  aprsSendCRLF();
}

/**
  Send APRS telemetry setup
*/
void aprsSendTelemetrySetup() {
  char padCallSign[10];
  sprintf_P(padCallSign, PSTR("%-9s"), aprsCallSign);
  // Parameter names
  pkt  = aprsCallSign;
  pkt += aprsPath;
  pkt += F(":");
  pkt += padCallSign;
  pkt += F(":PARM.Light,Thrm,RSSI,Vcc,Tmp,PROBE,ATMO,LUX,SAT,BAT,TM,B7,B8");
  pkt += F("\r\n");
  aprsSend(pkt);
  // Equations
  pkt  = aprsCallSign;
  pkt += aprsPath;
  pkt += F(":");
  pkt += padCallSign;
  pkt += F(":EQNS.0,20,0,0,20,0,0,-1,0,0,0.004,4.5,0,1,-100");
  pkt += F("\r\n");
  aprsSend(pkt);
  // Units
  pkt  = aprsCallSign;
  pkt += aprsPath;
  pkt += F(":");
  pkt += padCallSign;
  pkt += F(":UNIT.lux,mV,dBm,V,C,prb,on,on,sat,low,err,N/A,N/A");
  pkt += F("\r\n");
  aprsSend(pkt);
  // Bit sense and project name
  pkt  = aprsCallSign;
  pkt += aprsPath;
  pkt += F(":");
  pkt += padCallSign;
  pkt += F(":BITS.10011111, ");
  pkt += NODENAME;
  pkt += F("/");
  pkt += VERSION;
  pkt += F("\r\n");
  aprsSend(pkt);
}

void aprsSendTelemetrySetup_OLD() {
  char padCallSign[10];
  sprintf_P(padCallSign, PSTR("%-9s"), aprsCallSign);
  // Parameter names
  aprsSendHeader(":");
  aprsSend(padCallSign);
  aprsSend(F(":PARM.Light,Thrm,RSSI,Vcc,Tmp,PROBE,ATMO,LUX,SAT,BAT,TM,B7,B8"));
  aprsSendCRLF();
  // Equations
  aprsSendHeader(":");
  aprsSend(padCallSign);
  aprsSend(F(":EQNS.0,20,0,0,20,0,0,-1,0,0,0.004,4.5,0,1,-100"));
  aprsSendCRLF();
  // Units
  aprsSendHeader(":");
  aprsSend(padCallSign);
  aprsSend(F(":UNIT.lux,mV,dBm,V,C,prb,off,off,sat,low,err,N/A,N/A"));
  aprsSendCRLF();
  // Bit sense and project name
  aprsSendHeader(":");
  aprsSend(padCallSign);
  aprsSend(F(":BITS.10011111, "));
  aprsSend(NODENAME);
  aprsSend(F("/"));
  aprsSend(VERSION);
  aprsSendCRLF();
}

/**
  Send APRS status
  FW0690>APRS,TCPIP*:>13:06 Fine weather

  @param message the status message to send
*/
void aprsSendStatus(const char *message) {
  // Send only if the message is not empty
  if (message[0] != '\0') {
    // Send the APRS packet
    aprsSendHeader(">");
    aprsSend(message);
    aprsSendCRLF();
  }
}

/**
  Send APRS position and altitude
  FW0690>APRS,TCPIP*:!DDMM.hhN/DDDMM.hhW$comments

  @param comment the comment to append
*/
void aprsSendPosition(const char *comment) {
  // Compose the APRS packet
  aprsSendHeader("!");
  aprsSend(aprsLocation);
  aprsSend(F("/000/000/A="));
  char buf[7];
  sprintf_P(buf, PSTR("%06d"), altFeet);
  aprsSend(buf);
  aprsSend(comment);
  aprsSendCRLF();
}

int readMCUTemp() {
  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC

  delay(10);                        // Wait for voltages to become stable.
  ADCSRA |= _BV(ADSC);              // Start the ADC
  while (bit_is_set(ADCSRA, ADSC)); // Detect end-of-conversion

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  long wADC = ADCW;

  // The returned temperature is in hundreds degrees Celsius; calibrated
  return (int)(84.87 * wADC - 25840);
}

/*
  Read the power supply voltage, by measuring the internal 1V1 reference
*/
int readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(10);                        // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);              // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // Detect end-of-conversion

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  long wADC = ADCW;
  // Return Vcc in mV; 1125300 = 1.1 * 1023 * 1000
  // Calibration: 1.074
  return (int)(1098702UL / wADC);
}

time_t getUNIXTime() {
  union {
    uint32_t t = 0UL;
    uint8_t  b[4];
  } uxtm;

  // Try to establish the PPP link
  // FIXME blocker
  while (!GPRS_Modem.ppp_connect(apn)) delay(1000);

  int i = 3;
  if (APRS_Client.connect("utcnist.colorado.edu", 37)) {
    unsigned int timeout = millis() + 10000UL;   // 10 seconds timeout
    while (millis() <= timeout and i >= 0) {
      if (APRS_Client.available()) uxtm.b[i--] = APRS_Client.read();
    }
    APRS_Client.stop();
  }
  if (i < 0) {
    uint32_t tm = uxtm.t - 2208988800UL;
    Serial.print(F("Time sync: "));
    Serial.println(tm);
    return tm;
  }
  else {
    Serial.println(F("Time sync error"));
    return 0;
  }
}

void setup() {
  // Init the serial com
  Serial.println();
  Serial.begin(9600);
  Serial.println(NODENAME);
  Serial.println(__DATE__);

  pkt.reserve(120);

  // Set GSM module baud rate
  SerialAT.begin(9600);
  GPRS_Modem.begin(&SerialAT, SIM_PRESENT);
  char str[20];
  GPRS_Modem.get_imei(str, sizeof(str));
  Serial.println(str);

  // Start time sync
  setSyncProvider(getUNIXTime);
  setSyncInterval(60 * 60);

  // BMP280
  if (atmo.begin(0x76)) {
    atmo_ok = true;
    Serial.println(F("BMP280 sensor detected."));
  }
  else {
    atmo_ok = false;
    Serial.println(F("BMP280 sensor missing."));
  }

  // Initialize the random number generator and set the APRS telemetry start sequence
  randomSeed(readMCUTemp() + now() + GPRS_Modem.get_rssi() + readVcc() + millis());
  aprsTlmSeq = random(1000);

  // Start the sensor timer
  snsNextTime = millis();
}

void loop() {
  // Read the sensors and publish telemetry
  if (millis() >= snsNextTime) {
    // Keep the time
    now();
    // Count to check if we need to send the APRS data
    if (++aprsMsrmCount > aprsMsrmMax) aprsMsrmCount = 1;
    // Set the telemetry bit 7 if the station is being probed
    if (PROBE) aprsTlmBits = B10000000;

    // Set the telemetry bit 2 if time is not accurate
    if (timeStatus() != timeSet) aprsTlmBits |= B00000100;

    // Read BMP280
    float temp, pres;
    if (atmo_ok) {
      // Set the bit 5 to show the sensor is present (reverse)
      aprsTlmBits |= B01000000;
      // Get the weather parameters
      temp = atmo.readTemperature();
      pres = atmo.readPressure();
      // Median Filter
      mdnIn(rmTemp, (int)(temp * 9 / 5 + 32));      // Store directly integer Fahrenheit
      mdnIn(rmPres, (int)(pres * altCorr / 10.0));  // Store directly sea level in dPa
    }

    // Read Vcc first (mV)
    int vcc = readVcc();
    if (vcc < 5000) {
      // Set the bit 3 to show the battery is low
      aprsTlmBits |= B00001000;
    }

    // Various telemetry
    int a0 = analogRead(A0);
    int a1 = analogRead(A1);

    // Median Filter
    mdnIn(rmA0, ((unsigned long)vcc * (unsigned long)a0) / 20480);
    mdnIn(rmA1, ((unsigned long)vcc * (unsigned long)a1) / 20480);
    mdnIn(rmVcc, vcc);

    // 500 / R(kO); R = R0(1023/x-1)
    int lux = 51150L / a0 - 50;

    // APRS (after the first 3600/(aprsMsrmMax*aprsRprtHour) seconds,
    //       then every 60/aprsRprtHour minutes)
    if (aprsMsrmCount == 1) {
      // Try to establish the PPP link
      // FIXME blocker
      while (!GPRS_Modem.ppp_connect(apn)) delay(1000);
      // Get RSSI
      int rssi = GPRS_Modem.get_rssi();
      // Connect to APRS server
      if (APRS_Client.connect(aprsServer, aprsPort)) {
        aprsAuthenticate();
        //aprsSendPosition(" WxUnoProbe");
        if (atmo_ok) aprsSendWeather(mdnOut(rmTemp), -1, mdnOut(rmPres), lux);
        //aprsSendWeather(rmTemp.out(), -1, -1, -1);
        aprsSendTelemetry(mdnOut(rmA0), mdnOut(rmA1), -rssi, (mdnOut(rmVcc) - 4500) / 4, readMCUTemp() / 100 + 100, aprsTlmBits);
        //aprsSendStatus("Fine weather");
        //aprsSendTelemetrySetup();
        APRS_Client.stop();
      }
      else Serial.println(F("Connection failed"));
    }

    // Repeat sensor reading
    snsNextTime += snsDelay;
  }
}
