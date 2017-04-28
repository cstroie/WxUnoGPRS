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

// Device name and software version
const char NODENAME[] PROGMEM = "WxUnoGPRS";
const char VERSION[]  PROGMEM = "2.2";
bool       PROBE              = true;       // True if the station is being probed

// GPRS credentials
const char apn[]  PROGMEM = "internet.simfony.net";  // GPRS access point
const char user[] PROGMEM = "";                      // GPRS user name
const char pass[] PROGMEM = "";                      // GPRS password

// APRS parameters
const char  aprsServer[] PROGMEM  = "cwop5.aprs.net";   // CWOP APRS-IS server address to connect to
const int   aprsPort              = 14580;              // CWOP APRS-IS port
#ifdef DEVEL
const int   altMeters             = 83;                 // Altitude in Bucharest
#else
const int   altMeters             = 282;                // Altitude in Targoviste
#endif
const long  altFeet = (long)(altMeters * 3.28084);                                    // Altitude in feet
const float altCorr = pow((float)(1.0 - 2.25577e-5 * altMeters), (float)(-5.25578));  // Altitude correction for QNH

const char aprsCallSign[] PROGMEM = "FW0727";
const char aprsPassCode[] PROGMEM = "-1";
const char aprsPath[]     PROGMEM = ">APRS,TCPIP*:";
const char aprsLocation[] PROGMEM = "4455.29N/02527.08E_";
const char aprsTlmPARM[]  PROGMEM = ":PARM.Light,Soil,RSSI,Vcc,Tmp,PROBE,ATMO,LUX,SAT,BAT,TM,B7,B8";
const char aprsTlmEQNS[]  PROGMEM = ":EQNS.0,20,0,0,20,0,0,-1,0,0,0.004,4.5,0,1,-100";
const char aprsTlmUNIT[]  PROGMEM = ":UNIT.mV,mV,dBm,V,C,prb,on,on,sat,low,err,N/A,N/A";
const char aprsTlmBITS[]  PROGMEM = ":BITS.10011111, ";
const char eol[]          PROGMEM = "\r\n";

char       aprsPkt[100]           = "";     // The APRS packet buffer, largest packet is 82 for v2.1

// Time synchronization and keeping
const char    timeServer[] PROGMEM  = "utcnist.colorado.edu";  // Time server address to connect to (RFC868)
const int     timePort              = 37;                      // Time server port
unsigned long timeNextSync          = 0UL;                     // Next time to syncronize
unsigned long timeDelta             = 0UL;                     // Difference between real time and internal clock
bool          timeOk                = false;                   // Flag to know the time is accurate
const int     eeTime                = 0;                       // EEPROM address for storing last known good time

// Reports and measurements
const int aprsRprtHour   = 10; // Number of APRS reports per hour
const int aprsMsrmMax    = 3;  // Number of measurements per report (keep even)
int       aprsMsrmCount  = 0;  // Measurements counter
int       aprsTlmSeq     = 0;  // Telemetry sequence mumber

// Telemetry bits
char      aprsTlmBits    = B00000000;

// M590 control pins
const int pinM590Ring    = 2;  // Incoming call/sms signal
const int pinM590Rx      = 3;  // MCU RX
const int pinM590Tx      = 4;  // MCU TX
const int pinM590Sleep   = 5;  // Modem low power control
const int pinM590Power   = 6;  // Modem On/Off control

// Modemm and connection client
SoftwareSerial  SerialAT(pinM590Rx, pinM590Tx); // Software RS232 link to modem
M590Drv         GPRS_Modem;                     // Modem driver
M590Client      GPRS_Client(&GPRS_Modem);       // GPRS client
unsigned long   linkLastTime = 0UL;             // Last time the modem and tcp connected

// When ADC completed, take an interrupt
EMPTY_INTERRUPT(ADC_vect);

// Statistics (round median filter for the last 3 values)
enum      rMedIdx {MD_TEMP, MD_PRES, MD_RSSI, MD_SRAD, MD_VCC, MD_A0, MD_A1, MD_ALL};
int       rMed[MD_ALL][4];
const int eeRMed = 16; // EEPROM address for storing the round median array

// Sensors
const unsigned long snsDelay    = 3600000UL / (aprsRprtHour * aprsMsrmMax); // Delay between sensor readings
unsigned long       snsNextTime = 0UL;                                      // Next time to read the sensors
Adafruit_BMP280     atmo;                                                   // The athmospheric sensor
bool                atmo_ok = false;                                        // The athmospheric sensor presence flag

// Function prototypes
void modemSleep(bool enable, bool initial = false);

/**
  Simple median filter: get the median
  2014-03-25: started by David Cary

  @param idx the index in round median array
  @return the median
*/
int rMedOut(int idx) {
  // Return the last value if the buffer is not full yet
  if (rMed[idx][0] < 3) return rMed[idx][3];
  else {
    // Get the maximum and the minimum
    int the_max = max(max(rMed[idx][1], rMed[idx][2]), rMed[idx][3]);
    int the_min = min(min(rMed[idx][1], rMed[idx][2]), rMed[idx][3]);
    // Clever code: XOR the max and min, remaining the middle
    return the_max ^ the_min ^ rMed[idx][1] ^ rMed[idx][2] ^ rMed[idx][3];
  }
}

/**
  Simple median filter: add value to array

  @param idx the index in round median array
  @param x the value to add
*/
void rMedIn(int idx, int x) {
  // At index 0 there is the number of values stored
  if (rMed[idx][0] < 3) rMed[idx][0]++;
  // Shift one position
  rMed[idx][1] = rMed[idx][2];
  rMed[idx][2] = rMed[idx][3];
  rMed[idx][3] = x;
}

/**
  Write the round median array to EEPROM, along with CRC32: 60 bytes = sizeof(int) * 4 * MD_ALL + 4 bytes
*/
void rMedEEWrite() {
  // Compute CRC32 checksum
  CRC32 crc32;
  crc32.update(&rMed, sizeof(rMed));
  unsigned long crc = crc32.finalize();
  // Write the data
  EEPROM.put(eeRMed, rMed);
  EEPROM.put(eeRMed + sizeof(rMed), crc);
}

/**
  Read the round median array from EEPROM, along with CRC32 and verify
*/
void rMedEERead() {
  unsigned long ck;
  // Read the data
  EEPROM.get(eeRMed, rMed);
  EEPROM.get(eeRMed + sizeof(rMed), ck);
  // Compute CRC32 checksum
  CRC32 crc32;
  crc32.update(&rMed, sizeof(rMed));
  unsigned long crc = crc32.finalize();
  // Verify and zero it if check fails
  if (crc != ck) memset(&rMed, 0, sizeof(rMed));
}

/**
  Write the time to EEPROM, along with CRC32: 8 bytes

  @param tm the time value to store
*/
void timeEEWrite(unsigned long tm) {
  // Compute CRC32 checksum
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
  // Read the data
  EEPROM.get(eeTime, tm);
  EEPROM.get(eeTime + sizeof(tm), tc);
  // Compute CRC32 checksum
  CRC32 crc;
  crc.update(&tm, sizeof(tm));
  unsigned long ck = crc.finalize();
  // Verify
  if (tc == ck) return tm;
  else          return 0UL;
}

/**
  Get current time as UNIX time (1970 epoch)

  @param sync flag to show whether network sync is to be performed
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

/**
  Connect to a time server using the RFC 868 time protocol

  @return UNIX time from server
*/
unsigned long timeSync() {
  union {
    uint32_t t = 0UL;
    uint8_t  b[4];
  } uxtm;

  // Wake up the modem (if power saving already enabled)
  modemSleep(false);

  // Try to establish the PPP link
  int bytes = sizeof(uxtm.b);
  if (GPRS_Modem.pppConnect_P(apn)) {
    if (GPRS_Client.connect_P(timeServer, timePort)) {
      // Read network time during 5 seconds
      unsigned long timeout = millis() + 5000UL;
      while (millis() <= timeout and bytes != 0) {
        char b = GPRS_Client.read();
        if (b != -1) uxtm.b[--bytes] = uint8_t(b);
      }
      GPRS_Client.stop();
      // Keep the millis the connection worked
      linkLastTime = millis();
    }
  }

  // Send the modem to sleep (if power saving already enabled)
  modemSleep(true);

  // Convert 1900 epoch to 1970 Unix time, if read data is valid
  if (!bytes) return (unsigned long)uxtm.t - 2208988800UL;
  else        return 0UL;
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
  Return time in zulu APRS format: HHMMSSh

  @param *buf the buffer to return the time to
  @param len the buffer length
*/
char aprsTime(char *buf, size_t len) {
  // Get the time, but do not open a connection to server
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
void aprsSendPosition(const char *comment = NULL) {
  // Compose the APRS packet
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR("!"));
  strcat_P(aprsPkt, aprsLocation);
  strcat_P(aprsPkt, PSTR("/000/000/A="));
  char buf[7];
  sprintf_P(buf, PSTR("%06d"), altFeet);
  strncat(aprsPkt, buf, sizeof(buf));
  if (comment != NULL) strcat(aprsPkt, comment);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**
  Read the analog pin after a delay, while sleeping, using interrupt

  @param pin the analog pin
  @return raw analog read value
*/
int readAnalog(uint8_t pin) {
  // Allow for channel or pin numbers
  if (pin >= 14) pin -= 14;

  // Set the analog reference to DEFAULT, select the channel (low 4 bits).
  // This also sets ADLAR (left-adjust result) to 0 (the default).
  ADMUX = _BV(REFS0) | (pin & 0x07);
  ADCSRA |= _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2);  // prescaler of 128
  ADCSRA |= _BV(ADEN);  // enable the ADC
  ADCSRA |= _BV(ADIE);  // enable intterupt

  // Wait for voltage to settle
  delay(10);
  // Take an ADC reading in sleep mode
  noInterrupts();
  // Start conversion
  ADCSRA |= _BV(ADSC);
  set_sleep_mode(SLEEP_MODE_ADC);
  interrupts();

  // Awake again, reading should be done, but better make sure
  // maybe the timer interrupt fired
  while (bit_is_set(ADCSRA, ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  long wADC = ADCW;

  // The returned reading
  return (int)(wADC);
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
  ADCSRA |= _BV(ADIE);  // enable intterupt

  // Wait for voltage to settle
  delay(10);
  // Take an ADC reading in sleep mode
  noInterrupts();
  // Start conversion
  ADCSRA |= _BV(ADSC);
  set_sleep_mode(SLEEP_MODE_ADC);
  interrupts();

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
  ADCSRA |= _BV(ADIE);  // enable intterupt

  // Wait for voltage to settle
  delay(10);
  // Take an ADC reading in sleep mode
  noInterrupts();
  // Start conversion
  ADCSRA |= _BV(ADSC);
  set_sleep_mode(SLEEP_MODE_ADC);
  interrupts();

  // Awake again, reading should be done, but better make sure
  // maybe the timer interrupt fired
  while (bit_is_set(ADCSRA, ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  long wADC = ADCW;

  // Return Vcc in mV; 1125300 = 1.1 * 1024 * 1000
  // 1.1V calibration: 1.074
  return (int)(1099776UL / wADC);
}

/**
  Software reset the MCU
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
  Power on/off the M590 modem (6s)

  @param initial configure the MCU control pin, initially
*/
void modemOnOff(bool initial = false) {
  if (initial) {
    // Make sure the pin stays HIGH before using it
    digitalWrite(pinM590Power, HIGH);
    pinMode(pinM590Power, OUTPUT);
  }
  // Enable it for 1s, then disable and wait 5s more
  digitalWrite(pinM590Power, LOW);
  delay(1000);
  digitalWrite(pinM590Power, HIGH);
  delay(5000);
}

/**
  Power on/off the M590 modem

  @param enable enable or disable the low power mode
  @param initial configure the MCU control pin, initially
*/
void modemSleep(bool enable, bool initial = false) {
  if (initial) {
    // Make sure the pin stays HIGH before using it
    digitalWrite(pinM590Sleep, HIGH);
    pinMode(pinM590Sleep, OUTPUT);
    // Configure the modem to use power saving mode
    GPRS_Modem.pwrSave();
  }
  if (enable) {
#ifdef DEBUG
    Serial.print(F("Putting the modem to sleep... "));
#endif
    digitalWrite(pinM590Sleep, LOW);
    delay(2000);
#ifdef DEBUG
    Serial.println(F("done"));
#endif
  }
  else {
#ifdef DEBUG
    Serial.print(F("Waking up the modem... "));
#endif
    digitalWrite(pinM590Sleep, HIGH);
    delay(100);
#ifdef DEBUG
    Serial.println(F("done"));
#endif
  }
}

/**
  Check if the link failed for too long (3600 / aprsRprtHour) and reset
*/
void linkFailed() {
  if (millis() >= linkLastTime + 3600000UL / aprsRprtHour) {
    Serial.println(F("PPP link failed for the last reports, resetting all"));
    // If time is good, store it
    if (timeOk) timeEEWrite(timeUNIX(false));
    // Store the round median array
    rMedEEWrite();
    // Try to power off the modem (need 5s)
    GPRS_Modem.powerDown();
    // Reset the MCU (in 8s)
    softReset(WDTO_8S);
  }
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
  Serial.begin(9600);
  Serial.println();
  print_P(NODENAME);
  Serial.print(F(" "));
  print_P(VERSION);
  Serial.print(F(" "));
  Serial.println(__DATE__);

  // Try to restore the round median array
  rMedEERead();

  // Set GSM module baud rate
  SerialAT.begin(9600);
  // Initialize the modem, restart if failed (total time to restart: 30s)
  if (GPRS_Modem.begin(&SerialAT, SIM_PRESENT)) {
    // Start time sync
    timeUNIX();
  }
  else {
#ifdef DEBUG
    Serial.println(F("Trying to power on the modem, then reset the MCU"));
#endif
    // Try to enble the modem
    modemOnOff(true);
    // Wait a little and reset the MCU
    delay(15000);
    softReset(WDTO_4S);
  }

  // BMP280
  atmo_ok = atmo.begin(0x76);
  if (atmo_ok) Serial.println(F("BMP280 sensor detected"));
  else         Serial.println(F("BMP280 sensor missing"));

  // Initialize the random number generator and set the APRS telemetry start sequence
  randomSeed(readMCUTemp() + timeUNIX(false) + GPRS_Modem.getRSSI() + readVcc() + millis());
  aprsTlmSeq = random(1000);

  // Start the sensor timer
  snsNextTime = millis();

  // Send the modem to sleep
  modemSleep(true, true);
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
      // Add to the round median filter
      rMedIn(MD_TEMP, (int)(temp * 9 / 5 + 32));      // Store directly integer Fahrenheit
      rMedIn(MD_PRES, (int)(pres * altCorr / 10.0));  // Store directly sea level in dPa
    }

    // Read Vcc (mV) and add to the round median filter
    int vcc = readVcc();
    rMedIn(MD_VCC, vcc);
    if (vcc < 4750 or vcc > 5250) {
      // Set the bit 3 to show the USB voltage is wrong (5V +/- 5%)
      aprsTlmBits |= B00001000;
    }

    // Various analog telemetry
    int a0 = readAnalog(A0);
    int a1 = readAnalog(A1);

    // Add to round median filter, mV (a / 1024 * Vcc)
    rMedIn(MD_A0, (vcc * (unsigned long)a0) / 1024);
    rMedIn(MD_A1, (vcc * (unsigned long)a1) / 1024);

    // Upper part
    // 500 / R(kO); R = R0(1024 / x - 1)
    // Lower part
    // Vout=RawADC0*0.0048828125;
    // lux=(2500/Vout-500)/10;
    //int lux = 51150L / a0 - 50;

    // Illuminance value in lux
    long lux = 50L * (1024L - a0) / a0;
    // Calculate the solar radiation in W/m^2
    // FIXME this is in cW/m^2
    int solRad = (int)(lux * 0.79);
    // Set the bit 5 to show the sensor is present (reverse) and there is any light
    if (solRad > 0) aprsTlmBits |= B00100000;
    // Set the bit 4 to show the sensor is saturated
    if (solRad > 999) aprsTlmBits |= B00010000;
    // Add to round median filter
    rMedIn(MD_SRAD, solRad);

    // APRS (after the first 3600/(aprsMsrmMax*aprsRprtHour) seconds,
    //       then every 60/aprsRprtHour minutes)
    if (aprsMsrmCount == 1) {
      // Wake up the modem, if sleeping
      modemSleep(false);
      // Get RSSI (will get FALSE (0) if the modem is not working)
      int rssi = GPRS_Modem.getRSSI();
      if (rssi) rMedIn(MD_RSSI, -rssi);
      // Try to establish the PPP link, restart if it failed the last two reports
      // TODO store to flash last measurements
      if (GPRS_Modem.pppConnect_P(apn)) {
        // Connect to APRS server
        if (GPRS_Client.connect_P(aprsServer, aprsPort)) {
          // Authentication
          aprsAuthenticate();
          // Send the position, altitude and comment in firsts minutes after boot
          if (millis() < snsDelay + snsDelay) aprsSendPosition();
          // Send weather data if the athmospheric sensor is present
          if (atmo_ok) aprsSendWeather(rMedOut(MD_TEMP), -1, rMedOut(MD_PRES), rMedOut(MD_SRAD));
          // Send the telemetry
          aprsSendTelemetry(rMedOut(MD_A0) / 20,
                            rMedOut(MD_A1) / 20,
                            rMedOut(MD_RSSI),
                            (rMedOut(MD_VCC) - 4500) / 4,
                            readMCUTemp() / 100 + 100,
                            aprsTlmBits);
          //aprsSendStatus("Fine weather");
          // Close the connection
          GPRS_Client.stop();
          // Keep the millis the connection worked
          linkLastTime = millis();
        }
        else linkFailed();
      }
      else linkFailed();
      // Send the modem to sleep
      modemSleep(true);
    }

    // Repeat sensor reading
    snsNextTime += snsDelay;
  }
}
