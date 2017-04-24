/**
  WxUnoGPRS - Weather Station for Arduino UNO, GPRS connected

  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of Weather Station.

  WxUnoGPRS is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by the Free
  Software Foundation, either version 3 of the License, or (at your option) any
  later version.

  WxUnoGPRS is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  WxUnoGPRS.  If not, see <http://www.gnu.org/licenses/>.


  GPRS connected weather station, reading the temperature and athmospheric
  pressure sensor BMP280, as well as internal temperature, supply voltage,
  local illuminance, publishing the measured data to CWOP APRS.
*/

// The DEBUG and DEVEL flag
//#define DEBUG
#define DEVEL

// Watchdog, sleep
#include <avr/wdt.h>
#include <avr/sleep.h>

// EEPROM and CRC32
#include <EEPROM.h>
#include <CRC32.h>

// The sensors are connected to I2C
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// GPRS
#include <M590Client.h>
#include <SoftwareSerial.h>

// Device name
const char NODENAME[] PROGMEM = "WxUnoGPRS";
const char VERSION[]  PROGMEM = "1.7";
bool  PROBE = true;    // True if the station is being probed

// GPRS credentials
const char *apn  = "internet.simfony.net";
const char *user = "";
const char *pass = "";

// APRS parameters
const char *aprsServer = "cwop5.aprs.net";
const int   aprsPort = 14580;
#ifdef DEVEL
const int   altMeters = 83;   // Bucharest
#else
const int   altMeters = 282;  // Targoviste
#endif
const long  altFeet = (long)(altMeters * 3.28084);
float altCorr = pow((float)(1.0 - 2.25577e-5 * altMeters), (float)(-5.25578));

const char aprsCallSign[] PROGMEM = "FW0727";
const char aprsPassCode[] PROGMEM = "-1";
const char aprsPath[]     PROGMEM = ">APRS,TCPIP*:";
const char aprsLocation[] PROGMEM = "4455.29N/02527.08E_";
const char aprsTlmPARM[]  PROGMEM = ":PARM.Light,Thrm,RSSI,Vcc,Tmp,PROBE,ATMO,LUX,SAT,BAT,TM,B7,B8";
const char aprsTlmEQNS[]  PROGMEM = ":EQNS.0,20,0,0,20,0,0,-1,0,0,0.004,4.5,0,1,-100";
const char aprsTlmUNIT[]  PROGMEM = ":UNIT.mV,mV,dBm,V,C,prb,on,on,sat,low,err,N/A,N/A";
const char aprsTlmBITS[]  PROGMEM = ":BITS.10011111, ";
const char eol[]          PROGMEM = "\r\n";

// Time
const char   *timeServer = "utcnist.colorado.edu";
const int     timePort = 37;
unsigned long timeNextSync = 0UL;
unsigned long timeDelta = 0UL;
bool          timeOk = false;
// EEPROM address for storing last known time
int           eeTime = 0;

// Reports and measurements
const int   aprsRprtHour  = 10; // Number of APRS reports per hour
const int   aprsMsrmMax   = 3;  // Number of measurements per report (keep even)
int         aprsMsrmCount = 0;  // Measurements counter
int         aprsTlmSeq    = 0;  // Telemetry sequence mumber

// Telemetry bits
char        aprsTlmBits   = B00000000;

// The APRS connection client
SoftwareSerial SerialAT(2, 3); // RX, TX
M590Drv GPRS_Modem;
M590Client GPRS_Client(&GPRS_Modem);
IPAddress ip;
// The APRS packet buffer
char aprsPkt[120] = "";

// Statistics (round median filter for the last 3 values)
int mTemp[4];
int mPres[4];
int mRSSI[4];
int mRad[4];
int mVcc[4];
int mA0[4];
int mA1[4];

// Sensors
const unsigned long snsDelay    = 3600000UL / (aprsRprtHour * aprsMsrmMax);
unsigned long       snsNextTime = 0UL;  // The next time to read the sensors
Adafruit_BMP280 atmo;                   // The athmospheric sensor
bool atmo_ok = false;                   // The athmospheric sensor flag

/**
  Simple median filter: get the median
  2014-03-25: started by David Cary

  @param *buf the round median buffer
  @return the median
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

/**
  Simple median filter: add value to buffer

  @param *buf the round median buffer
  @param x the value to add
*/
void mdnIn(int *buf, int x) {
  if (buf[0] < 3) buf[0]++;
  buf[1] = buf[2];
  buf[2] = buf[3];
  buf[3] = x;
}

/**
  Write the time to EEPROM, along with CRC32: 8 bytes
*/
void timeEEWrite(unsigned long tm) {
  // Create a CRC32 checksum calculator.
  CRC32 crc;
  crc.update(&tm, sizeof(tm));
  unsigned long ck = crc.finalize();
  // Write the data
  EEPROM.put(eeTime, tm);
  EEPROM.put(eeTime + sizeof(tm), ck);
}

/**
  Read the time from EEPROM, along with CRC32 and verify
*/
unsigned long timeEERead() {
  unsigned long tm, tc;
  EEPROM.get(eeTime, tm);
  EEPROM.get(eeTime + sizeof(tm), tc);
  // Create a CRC32 checksum calculator.
  CRC32 crc;
  crc.update(&tm, sizeof(tm));
  unsigned long ck = crc.finalize();
  // Verify
  if (tc == ck) return tm;
  else          return 0UL;
}

/**
  Get current time as UNIX time

  @return current UNIX time
*/
unsigned long timeUNIX(bool sync = true) {
  // Check if we need to sync
  if (millis() >= timeNextSync and sync) {
    // Try to get the time from Internet
    unsigned long tm = timeSync();
    if (tm == 0) {
      // Time sync has failed, sync again over one minute
      timeNextSync += 1UL * 60 * 1000;
      timeOk = false;
      // Try to get old time from eeprom, if time delta is zero
      if (timeDelta == 0) {
        // Compute an approximate time delta, if time is valid
        tm = timeEERead();
        if (tm != 0) {
          timeDelta = tm - (millis() / 1000);
          Serial.print(F("Time sync error, using EEPROM: 0x"));
          Serial.println(tm, 16);
        }
        else Serial.println(F("Time sync error, invalid EEPROM"));
      }
    }
    else {
      // Compute the new time delta
      timeDelta = tm - (millis() / 1000);
      // Time sync has succeeded, sync again in 8 hours
      timeNextSync += 8UL * 60 * 60 * 1000;
      timeOk = true;
      // Store this known time
      timeEEWrite(tm);
      Serial.print(F("Network UNIX Time: 0x"));
      Serial.println(tm, 16);
    }
  }

  // Get current time based on uptime and time delta,
  // or just uptime for no time sync ever
  return (millis() / 1000) + timeDelta;
}

/*
  Connect to a time server using the RFC 868 time protocol

  @return UNIX time from server
*/
unsigned long timeSync() {
  union {
    uint32_t t = 0UL;
    uint8_t  b[4];
  } uxtm;

  // Try to establish the PPP link
  int bytes = -1;
  if (GPRS_Modem.pppConnect(apn)) {
    if (GPRS_Client.connect(timeServer, timePort)) {
      bytes = GPRS_Client.read(uxtm.b, sizeof(uxtm.t));
      GPRS_Client.stop();
    }
  }

  // Convert 1900 epoch to 1970 Unix time, if valid time
  if (bytes == sizeof(uxtm.t))
    return uxtm.t - 2208988800UL;
  else
    return 0UL;
}

/**
  Send an APRS packet and, eventuall, print it to serial line

  @param *pkt the packet to send
*/
void aprsSend(const char *pkt) {
  GPRS_Client.write((uint8_t *)pkt, strlen(pkt));
#ifdef DEBUG
  Serial.print(pkt);
#endif
}

/**
  Return time in APRS format: DDHHMMz

  @param *buf the buffer to return the time to
  @param len the buffer length
*/
char aprsTime(char *buf, size_t len) {
  // Get the time, but do not open a connection
  unsigned long utm = timeUNIX(false);
  // Compute hour, minute and second
  int hh = (utm % 86400L) / 3600;
  int mm = (utm % 3600) / 60;
  int ss =  utm % 60;
  // Return the formatted time
  snprintf_P(buf, len, PSTR("%02d%02d%02dh"), hh, mm, ss);
}

/**
  Send APRS authentication data
  user FW0690 pass -1 vers WxUnoGPRS 0.2"
*/
void aprsAuthenticate() {
  strcpy_P(aprsPkt, PSTR("user "));
  strcat_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, PSTR(" pass "));
  strcat_P(aprsPkt, aprsPassCode);
  strcat_P(aprsPkt, PSTR(" vers "));
  strcat_P(aprsPkt, NODENAME);
  strcat_P(aprsPkt, PSTR(" "));
  strcat_P(aprsPkt, VERSION);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**
  Send APRS weather data, then try to get the forecast
  FW0690>APRS,TCPIP*:@152457h4427.67N/02608.03E_.../...g...t044h86b10201L001WxUnoGPRS

  @param temp temperature
  @param hmdt humidity
  @param pres athmospheric pressure
  @param lux illuminance
*/
void aprsSendWeather(int temp, int hmdt, int pres, int lux) {
  char buf[8];
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR("@"));
  aprsTime(buf, sizeof(buf));
  strncat(aprsPkt, buf, sizeof(buf));
  strcat_P(aprsPkt, aprsLocation);
  // Wind (unavailable)
  strcat_P(aprsPkt, PSTR(".../...g..."));
  // Temperature
  if (temp >= -460) { // 0K in F
    sprintf_P(buf, PSTR("t%03d"), temp);
    strncat(aprsPkt, buf, sizeof(buf));
  }
  else {
    strcat_P(aprsPkt, PSTR("t..."));
  }
  // Humidity
  if (hmdt >= 0) {
    if (hmdt == 100) {
      strcat_P(aprsPkt, PSTR("h00"));
    }
    else {
      sprintf_P(buf, PSTR("h%02d"), hmdt);
      strncat(aprsPkt, buf, sizeof(buf));
    }
  }
  // Athmospheric pressure
  if (pres >= 0) {
    sprintf_P(buf, PSTR("b%05d"), pres);
    strncat(aprsPkt, buf, sizeof(buf));
  }
  // Illuminance, if valid
  if (lux >= 0 and lux <= 999) {
    sprintf_P(buf, PSTR("L%03d"), lux);
    strncat(aprsPkt, buf, sizeof(buf));
  }
  // Comment (device name)
  strcat_P(aprsPkt, NODENAME);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**
  Send APRS telemetry and, periodically, send the telemetry setup
  FW0690>APRS,TCPIP*:T#517,173,062,213,002,000,00000000

  @param a0 read analog A0
  @param a1 read analog A1
  @param rssi GSM RSSI level
  @param vcc voltage
  @param temp internal temperature
  @param bits digital inputs
*/
void aprsSendTelemetry(int a0, int a1, int rssi, int vcc, int temp, byte bits) {
  // Increment the telemetry sequence number, reset it if exceeds 999
  if (++aprsTlmSeq > 999) aprsTlmSeq = 0;
  // Send the telemetry setup on power up (first minutes) or if the sequence number is 0
  if ((aprsTlmSeq == 0) or (millis() < snsDelay + snsDelay)) aprsSendTelemetrySetup();
  // Compose the APRS packet
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR("T"));
  char buf[40];
  snprintf_P(buf, sizeof(buf), PSTR("#%03d,%03d,%03d,%03d,%03d,%03d,"), aprsTlmSeq, a0, a1, rssi, vcc, temp);
  strncat(aprsPkt, buf, sizeof(buf));
  itoa(bits, buf, 2);
  strncat(aprsPkt, buf, sizeof(buf));
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**
  Send APRS telemetry setup
*/
void aprsSendTelemetrySetup() {
  char padCallSign[10];
  strcpy_P(padCallSign, aprsCallSign);  // Workaround
  sprintf_P(padCallSign, PSTR("%-9s"), padCallSign);
  // Parameter names
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR(":"));
  strncat(aprsPkt, padCallSign, sizeof(padCallSign));
  strcat_P(aprsPkt, aprsTlmPARM);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
  // Equations
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR(":"));
  strncat(aprsPkt, padCallSign, sizeof(padCallSign));
  strcat_P(aprsPkt, aprsTlmEQNS);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
  // Units
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR(":"));
  strncat(aprsPkt, padCallSign, sizeof(padCallSign));
  strcat_P(aprsPkt, aprsTlmUNIT);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
  // Bit sense and project name
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR(":"));
  strncat(aprsPkt, padCallSign, sizeof(padCallSign));
  strcat_P(aprsPkt, aprsTlmBITS);
  strcat_P(aprsPkt, NODENAME);
  strcat_P(aprsPkt, PSTR("/"));
  strcat_P(aprsPkt, VERSION);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**
  Send APRS status
  FW0690>APRS,TCPIP*:>Fine weather

  @param message the status message to send
*/
void aprsSendStatus(const char *message) {
  // Send only if the message is not empty
  if (message[0] != '\0') {
    // Send the APRS packet
    strcpy_P(aprsPkt, aprsCallSign);
    strcat_P(aprsPkt, aprsPath);
    strcat_P(aprsPkt, PSTR(">"));
    strcat(aprsPkt, message);
    strcat_P(aprsPkt, eol);
    aprsSend(aprsPkt);
  }
}

/**
  Send APRS position and altitude
  FW0690>APRS,TCPIP*:!DDMM.hhN/DDDMM.hhW$comments

  @param comment the comment to append
*/
void aprsSendPosition(const char *comment) {
  // Compose the APRS packet
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR("!"));
  strcat_P(aprsPkt, aprsLocation);
  strcat_P(aprsPkt, PSTR("/000/000/A="));
  char buf[7];
  sprintf_P(buf, PSTR("%06d"), altFeet);
  strncat(aprsPkt, buf, sizeof(buf));
  strcat(aprsPkt, comment);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**
  Read the internal MCU temperature
  The internal temperature has to be used with the internal reference of 1.1V.
  Channel 8 can not be selected with the analogRead function yet.

  @return temperature in hundreds of degrees Celsius, *calibrated for my device*
*/
int readMCUTemp() {
  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2);  // prescaler of 128
  ADCSRA |= _BV(ADEN);  // enable the ADC

  // Wait for Vref to settle
  delay(10);
  // Take an ADC reading in sleep mode
  noInterrupts();
  set_sleep_mode (SLEEP_MODE_ADC);
  sleep_enable();
  
  // Start conversion
  ADCSRA |= _BV(ADSC);
  interrupts();
  sleep_cpu();     
  sleep_disable();

  // Awake again, reading should be done, but better make sure
  // maybe the timer interrupt fired 
  while (bit_is_set(ADCSRA, ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  long wADC = ADCW;

  // The returned temperature is in hundreds degrees Celsius; calibrated
  return (int)(84.87 * wADC - 25840);
}

/*
  Read the power supply voltage, by measuring the internal 1V1 reference

  @return voltage in millivolts, *calibrated for my device*
*/
int readVcc() {
  // Set the reference to Vcc and the measurement to the internal 1.1V reference
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  ADCSRA |= _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2);  // prescaler of 128
  ADCSRA |= _BV(ADEN);  // enable the ADC

  // Wait for Vref to settle
  delay(10);
  // Take an ADC reading in sleep mode
  noInterrupts();
  set_sleep_mode (SLEEP_MODE_ADC);
  sleep_enable();
  
  // Start conversion
  ADCSRA |= _BV(ADSC);
  interrupts();
  sleep_cpu();     
  sleep_disable();

  // Awake again, reading should be done, but better make sure
  // maybe the timer interrupt fired 
  while (bit_is_set(ADCSRA, ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  long wADC = ADCW;
  
  // Return Vcc in mV; 1125300 = 1.1 * 1024 * 1000
  // 1.1V calibration: 1.074
  return (int)(1099776UL / wADC);
}

/*
  Software reset
  (c) Mircea Diaconescu http://web-engineering.info/node/29
*/
void softReset(uint8_t prescaller) {
  // Start watchdog with the provided prescaller
  wdt_enable(prescaller);
  // Wait for the prescaller time to expire
  // without sending the reset signal by using
  // the wdt_reset() method
  while (true) {}
}

/**
  Print a character array from program memory
*/
void print_P(const char *str) {
  uint8_t val;
  do {
    val = pgm_read_byte(str);
    if (val) {
      Serial.write(val);
      str++;
    }
  } while (val);
}

/**
  Main Arduino setup function
*/
void setup() {
  // Init the serial com
  Serial.println();
  Serial.begin(9600);
  print_P(NODENAME);
  Serial.print(F(" "));
  print_P(VERSION);
  Serial.print(F(" "));
  Serial.println(__DATE__);

  // Set GSM module baud rate
  SerialAT.begin(9600);
  // Initialize the modem, restart if failed
  if (GPRS_Modem.begin(&SerialAT, SIM_PRESENT)) {
    // Start time sync
    timeUNIX();
  }
  else {
    // Wait a minute
    delay(60000);
    softReset(WDTO_4S);
  }

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
  randomSeed(readMCUTemp() + timeUNIX(false) + GPRS_Modem.getRSSI() + readVcc() + millis());
  aprsTlmSeq = random(1000);

  // Start the sensor timer
  snsNextTime = millis();
}

/**
  Main Arduino loop
*/
void loop() {
  // Read the sensors
  if (millis() >= snsNextTime) {
    // Count to check if we need to send the APRS data
    if (++aprsMsrmCount > aprsMsrmMax) aprsMsrmCount = 1;
    // Set the telemetry bit 7 if the station is being probed
    if (PROBE) aprsTlmBits = B10000000;

    // Check the time and set the telemetry bit 2 if time is not accurate
    unsigned long tm = timeUNIX();
    if (!timeOk) aprsTlmBits |= B00000100;

    // Read BMP280
    float temp, pres;
    if (atmo_ok) {
      // Set the bit 5 to show the sensor is present (reverse)
      aprsTlmBits |= B01000000;
      // Get the weather parameters
      temp = atmo.readTemperature();
      pres = atmo.readPressure();
      // Median Filter
      mdnIn(mTemp, (int)(temp * 9 / 5 + 32));      // Store directly integer Fahrenheit
      mdnIn(mPres, (int)(pres * altCorr / 10.0));  // Store directly sea level in dPa
    }

    // Read Vcc (mV) and add to the round median filter
    int vcc = readVcc();
    mdnIn(mVcc, vcc);
    if (vcc < 4750 or vcc > 5250) {
      // Set the bit 3 to show the USB voltage is wrong (5V +/- 5%)
      aprsTlmBits |= B00001000;
    }

    // Get RSSI (will get FALSE if the modem is not working)
    int rssi = GPRS_Modem.getRSSI();
    if (rssi) mdnIn(mRSSI, -rssi);

    // Various analog telemetry
    int a0 = analogRead(A0);
    int a1 = analogRead(A1);

    // Add to round median filter, mV ( a / 1024 * 5000)
    mdnIn(mA0, (5000 * (unsigned long)a0) / 1024);
    mdnIn(mA1, (5000 * (unsigned long)a1) / 1024);

    // Upper part
    // 500 / R(kO); R = R0(1024 / x - 1)
    // Lower part
    // Vout=RawADC0*0.0048828125;
    // lux=(2500/Vout-500)/10;
    //int lux = 51150L / a0 - 50;

    // Illuminance value in lux
    long lux = 50L * (1024L - a0) / a0;
    // Calculate the solar radiation in mW/m^2
    // FIXME this is in cW/m^2
    int solRad = (int)(lux * 0.79);
    // Set the bit 5 to show the sensor is present (reverse) and there is any light
    if (solRad > 0) aprsTlmBits |= B00100000;
    // Set the bit 4 to show the sensor is saturated
    if (solRad > 999) aprsTlmBits |= B00010000;
    // Add to round median filter
    mdnIn(mRad, solRad);

    // APRS (after the first 3600/(aprsMsrmMax*aprsRprtHour) seconds,
    //       then every 60/aprsRprtHour minutes)
    if (aprsMsrmCount == 1) {
      // TODO Wake up the modem if sleeping
      // Try to establish the PPP link, restart if failed
      // TODO store to flash last measurements and time
      if (!GPRS_Modem.pppConnect(apn)) {
        Serial.println(F("PPP link failed, restarting..."));
        // If time is good, store it
        if (timeOk) timeEEWrite(tm);
        // Reset
        softReset(WDTO_4S);
      }
      else {
        // Connect to APRS server
        if (GPRS_Client.connect(aprsServer, aprsPort)) {
          aprsAuthenticate();
          //aprsSendPosition(" WxUnoProbe");
          if (atmo_ok) aprsSendWeather(mdnOut(mTemp), -1, mdnOut(mPres), mdnOut(mRad));
          aprsSendTelemetry(mdnOut(mA0) / 20,
                            mdnOut(mA1) / 20,
                            mdnOut(mRSSI),
                            (mdnOut(mVcc) - 4500) / 4,
                            readMCUTemp() / 100 + 100,
                            aprsTlmBits);
          //aprsSendStatus("Fine weather");
          GPRS_Client.stop();
        }
        else Serial.println(F("Connection failed"));
        // TODO Send the modem to sleep
      }
    }

    // Repeat sensor reading
    snsNextTime += snsDelay;
  }
}
