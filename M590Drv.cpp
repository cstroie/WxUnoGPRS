/* This is a library for the M590E GSM Modem 
Written by Brian Ejike*/

#include <Arduino.h>
#include <avr/pgmspace.h>

#include "M590Drv.h"
#include "debug.h"

#define NUM_TAGS    14

#if defined(__SAM3X8E__)
#define vsnprintf_P  vsnprintf
#endif

//Stream * M590Drv::gsm;

//Ring_Buffer M590Drv::ringBuf(36);

const char * MODEM_TAGS[] = {
  "\r\nOK\r\n",
  "OK\r\n",
  "\r\nERROR\n",
  "CREG: 0,0",
  "CREG: 0,1",
  "CREG: 0,2",
  "CREG: 0,3",
  "CREG: 0,4",
  "CREG: 0,5",
  ",CON",
  ",DIS",
  "+CFUN: 0",
  "+CFUN: 1",
  "+CGATT: 1"
};

typedef enum {
  TAG_OK,
  TAG_TCP_OK,
  TAG_ERROR,
  TAG_REG_INACTIVE,
  TAG_REG_SUCCESS,
  TAG_REG_SEARCH,
  TAG_REG_ERROR,
  TAG_REG_UNKNOWN,
  TAG_REG_ROAMING,
  TAG_LINK_CONNECTED,
  TAG_LINK_DISCONNECTED,
  TAG_FUN_SLEEP,
  TAG_FUN_WORK,
  TAG_GPRS_OK
} ModemTags;

M590Drv::M590Drv() {
}

uint8_t M590Drv::begin(Stream * ss, char sim_state) {
  gsm = ss;
  gsm->setTimeout(1000);
  SIM_PRESENCE = sim_state;

  bool initOK = false;

  for(int i=0; i<5; i++) {
    if (send_cmd(F("AT")) == TAG_OK) {
      initOK = true;
      break;
    }
    delay(1000);
  }

  if (!initOK) {
    LOGERROR(F("M590 not found"));
    return false;
  }
  send_cmd(F("ATE0"));

  // LED Signal
  //send_cmd(F("AT+SIGNAL=3"));

  if (!SIM_PRESENCE) {
    LOGINFO(F("SIM absent"));
    LOGINFO(F("Initialization complete"));
    return true;
  }

  LOGINFO(F("Waiting for SIM registration..."));
  if (send_cmd(F("AT+CREG=0")) != TAG_OK) {
    LOGERROR(F("SIM registration error"));
    return false;
  }

  int retry;
  bool regOk = false;
  for (retry = 0; retry < 60; retry++) {
    int regStatus = send_cmd(F("AT+CREG?"));
    if      (regStatus == TAG_REG_INACTIVE) {
      LOGINFO(F("Inactive"));
    }
    else if (regStatus == TAG_REG_SUCCESS) {
      LOGINFO(F("SIM registered on the local network"));
      regOk = true;
    }
    else if (regStatus == TAG_REG_SEARCH) {
      LOGINFO(F("Searching for base stations"));
    }
    else if (regStatus == TAG_REG_ERROR) {
      LOGINFO(F("Rejected registration"));
    }
    else if (regStatus == TAG_REG_UNKNOWN) {
      LOGINFO(F("Unknown code"));
    }
    else if (regStatus == TAG_REG_ROAMING) {
      LOGINFO(F("SIM registered, roaming"));
      regOk = true;
    }
    if (regOk) break;
    delay(1000);
  }
  if (!regOk) {
    LOGERROR(F("SIM registration failed"));
    return false;
  }

  // Display some info
  char buf[30];
  LOGINFO2(F("RSSI: "), getRSSI(), F("dBm"));
  getIMEI(buf, sizeof(buf));
  LOGINFO1(F("IMEI: "), buf);
  getCOPS(buf, sizeof(buf));
  LOGINFO1(F("Oper: "), buf);


  // Close all stalled connections
  for (uint8_t link = 0; link < MAX_LINK; link++) tcpClose(link);

  // SMS
  send_cmd(F("AT+CMGF=1"));
  send_cmd(F("AT+CSCS=\"GSM\""));
  // Initial values
  _ppp_link = false;
  _buf_pos = 0;
  _curr_link = -1;
  LOGINFO(F("Initialization complete"));
  return true;
}

// RSSI
int16_t M590Drv::getRSSI() {
  if (!checkSerial()) return false;
  char buf[6];
  send_cmd_get(F("AT+CSQ"), F("CSQ:"), F(","), buf, sizeof(buf));
  int rssi = -113 + atoi(buf) + atoi(buf);
  return rssi;
}

// version
void M590Drv::getGMR(char *buf, int len) {
  if (!checkSerial()) return;
  send_cmd_get(F("AT+GMR"), F("GMR:"), F("\r\n\r\n"), buf, len);
}

// imei
void M590Drv::getIMEI(char *str, int len){
  if (!checkSerial()) return;
  send_cmd_get(F("AT+CGSN"), F("\r\n"), F("\r\n\r\n"), str, len);
}

// real-time clock
void M590Drv::getCCLK(char *str, int len) {
  if (!checkSerial()) return;
  send_cmd_get(F("AT+CCLK?"), F("CCLK: \""), F("\""), str, len);
}

void M590Drv::setCCLK(const char *str) {
  if (!checkSerial()) return;
  if (send_cmd(F("AT+CCLK=\"%s\""), 500, str) != TAG_OK) {
    LOGDEBUG(F("Could not set RTC"));
    return false;
  }
}

// Enable power saving
void M590Drv::pwrSave() {
  if (send_cmd(F("AT+ENPWRSAVE=1")) != TAG_OK) {
    LOGDEBUG(F("Could not enable power saving mode"));
  }
}


// network operator
void M590Drv::getCOPS(char *str, int len) {
  if (!checkSerial()) return;
  send_cmd_get(F("AT+COPS?"), F("COPS: 0,0,\""), F("\""), str, len);
}

// function: sleep/work
void M590Drv::setFUN(int fun, int rst) {
  if (send_cmd(F("AT+CFUN=%d,%d"), 2000, fun, rst) != TAG_OK) {
    LOGDEBUG(F("Could not set modem function/reset"));
  }
}

void M590Drv::funSleep() {
  if (send_cmd(F("AT+CFUN=0")) != TAG_OK) {
    LOGDEBUG(F("Could not send the modem to sleep"));
  }
}

void M590Drv::funWork() {
  if (send_cmd(F("AT+CFUN=1")) != TAG_OK) {
    LOGDEBUG(F("Could not wake up the modem"));
  }
}

void M590Drv::restart() {
  if (send_cmd(F("AT+CFUN=1,1")) != TAG_OK) {
    LOGDEBUG(F("Could not restart the modem"));
  }
}

int M590Drv::getFUN() {
  if (!checkSerial()) return false;
  int funStatus = send_cmd(F("AT+CFUN?"));
  if      (funStatus == TAG_FUN_SLEEP) {
    LOGINFO(F("Mode: sleeping"));
    return 0;
  }
  else if (funStatus == TAG_FUN_WORK) {
    LOGINFO(F("Mode: working"));
    return 1;
  }
  else {
    return -1;
  }
}

uint8_t M590Drv::checkSerial() {
  if (send_cmd(F("AT")) == TAG_OK) return true;
  LOGDEBUG(F("M590 not found"));
  return false;
}

uint8_t M590Drv::checkGPRS() {
  if (send_cmd(F("AT+CGATT?")) == TAG_GPRS_OK) return true;
  LOGDEBUG(F("GPRS not attached"));
  return false;
}

uint8_t M590Drv::pppConnect(const char *apn, const char *uname, const char *pwd) {
  if (!checkSerial()) return false;

  // Internal stack
  if (send_cmd(F("AT+XISP=0")) != TAG_OK) {
    LOGDEBUG(F("Could not enable internal stack"));
    return false;
  }
  // APN
  if (send_cmd(F("AT+CGDCONT=1,\"IP\",\"%s\""), 500, apn) != TAG_OK) {
    LOGDEBUG(F("Could not set APN"));
    return false;
  }
  // Authentication
  if (uname != NULL && pwd != NULL)
    send_cmd(F("AT+XGAUTH=1,1,\"%s\",\"%s\""), 500, uname, pwd);
  // Check the GPRS
  if (!checkGPRS()) return false;
  // Activate PPP link and check IP
  if (getIP(_ip_addr)) {
    LOGINFO1(F("PPP link is up, IP"), _ip_addr);
    return true;
  }
  else {
    LOGERROR(F("PPP link failed"));
    return false;
  }
}

uint8_t M590Drv::pppConnect_P(const char *apn, const char *uname, const char *pwd) {
  char apnBuf[strlen_P((char*)apn) + 1];
  strncpy_P(apnBuf, (char*)apn, sizeof(apnBuf));
  char unameBuf[strlen_P((char*)uname) + 1];
  strncpy_P(unameBuf, (char*)uname, sizeof(unameBuf));
  char pwdBuf[strlen_P((char*)pwd) + 1];
  strncpy_P(pwdBuf, (char*)pwd, sizeof(pwdBuf));
  return pppConnect(apnBuf, unameBuf, pwdBuf);
}

uint8_t M590Drv::getIP(IPAddress &ip) {
  if (!checkSerial()) return false;

  // Activate the PPP connection
  send_cmd(F("AT+XIIC=1"), 500);

  char retry;
  char temp[20];
  do {
    if (send_cmd_get(F("AT+XIIC?"), F("IC:    "), F("\r\n"), temp, sizeof(temp))) {
      if (temp[0] == '1') {
        _ip_addr.fromString(temp + 3);
        ip = _ip_addr;
        _ppp_link = true;
        return _ppp_link;
      }
    }
    delay(500);
    retry++;
  }
  while (retry < 10);

  _ppp_link = false;
  return _ppp_link;
}


uint8_t M590Drv::urlResolve(const char *url, IPAddress &ip) {
  if (!checkSerial()) return false;
  
  if (_ppp_link) {
    char temp[20];
    if (send_cmd_get(F("AT+DNS=\"%s\""), F("+DNS:"), F("\r\n"), temp, sizeof(temp), 4000, url)) {
      if (strstr(temp, "Error") == NULL) {
        ip.fromString(temp);
        return true;
      }
      else {
        LOGERROR(F("URL could not be resolved"));
      }
    }
    else {
      LOGERROR(F("Unexpected DNS response"));
    }
  }
  else {
    LOGERROR(F("No PPP link"));
  }
  return false;
}

bool M590Drv::linkStatus(uint8_t link) {
  if (link >= MAX_LINK) return false;
  if (_ppp_link) {
    if (send_cmd(F("AT+IPSTATUS=%d"), 500, link) == TAG_LINK_CONNECTED) {
      LOGDEBUG1(F("Link connected:"), link);
      return true;
    }
  }
  LOGDEBUG1(F("Link not connected:"), link);
  return false;
}

uint8_t M590Drv::tcpConnect(IPAddress &host, uint16_t port, uint8_t link) {
  if (link >= MAX_LINK) {
    LOGERROR1(F("Link not supported"), link);
    return false;
  }
  // Close the link, if open
  tcpClose(link);
  // Connect
  if (_ppp_link) {
    char hostbuf[20];
    sprintf(hostbuf, "%d.%d.%d.%d", host[0], host[1], host[2], host[3]);
    char temp[40];
    if (send_cmd_get(F("AT+TCPSETUP=%d,%s,%d"), F("UP:"), F("\r\n"), temp, sizeof(temp), 4000, link, hostbuf, port)) {
      if (strstr(temp, ",OK")) {
        LOGINFO1(F("Connected to "), host);
        return true;
      }
    }
  }
  else {
    LOGERROR(F("No PPP link"));
  }
  return false;
}

uint8_t M590Drv::tcpWrite(const uint8_t *data, uint16_t len, uint8_t link) {
  if (!linkStatus(link)) {
    LOGERROR(F("Link is not connected"));
    return false;
  }

  char params[10];
  sprintf_P(params, PSTR("%d,%u"), link, len);

  if (send_cmd_find(F("AT+TCPSEND=%s"), ">", 1000, params)) {
        gsm->write(data, len);
        gsm->write('\r');

        char temp[10];
        if (locate_tag("SEND:", "\r\n", temp, sizeof(temp), 3000, 0)) {
          if (strstr(temp, params)) {
            LOGINFO1(F("Data sent, bytes "), len);
            return true;
          }
          else {
            LOGERROR(F("Error sending data. No PPP link."));
            return false;
          }
        }
        else {
          LOGERROR(F("Error sending data. Unexpected response"));
          return false;
        }
  }
  else {
    LOGERROR(F("Error sending data. Expected '>'"));
    return false;
  }
}

bool M590Drv::tcpWrite(const __FlashStringHelper *data, uint16_t len, uint8_t link, bool appendCrLf) {
  if (!linkStatus(link)) {
    LOGERROR(F("Link is not connected"));
    return false;
  }

  char params[10];
  uint16_t len2 = len + appendCrLf + appendCrLf;
  sprintf_P(params, PSTR("%d,%u"), link, len2);

  if (send_cmd_find(F("AT+TCPSEND=%s"), ">", 1000, params)) {
    PGM_P p = reinterpret_cast<PGM_P>(data);
    for (int i=0; i<len; i++) {
      unsigned char c = pgm_read_byte(p++);
      gsm->write(c);
    }
    if (appendCrLf) {
      gsm->write('\r');
      gsm->write('\n');
    }

    char temp[10];
    if (locate_tag("SEND:", "\r\n", temp, sizeof(temp), 3000, 0)) {
      if (strstr(temp, params)){
        LOGINFO1(F("Data sent, bytes "), len2);
        return true;
      }
      else {
        LOGERROR(F("Error sending data. No PPP link."));
        return false;
      }
    }
    else {
      LOGERROR(F("Error sending data. Unexpected response"));
      return false;
    }
  }
  else {
    LOGERROR(F("Error sending data. Expected '>'"));
    return false;
  }
}

uint16_t M590Drv::avlData(uint8_t link) {
  //LOGDEBUG(_buf_pos);

  // If there is data in the buffer
  if (_buf_pos > 0 && _curr_link == link) return _buf_pos;

  int bytes = gsm->available();
  if (bytes) {
    LOGDEBUG1(F("Bytes in the serial buffer: "), bytes);
    if (gsm->find((char *)"CV:")) {
      // format is : +TCPRECV:<link>,<length>,<data>
      _curr_link = gsm->parseInt(); // <link>
      gsm->read();                  // ,
      _buf_pos = gsm->parseInt();   // <len>
      gsm->read();                  // ,

      LOGDEBUG();
      LOGDEBUG2(F("Data packet"), _curr_link, _buf_pos);

      if(_curr_link == link) return _buf_pos;
    }
  }
  /*
    else {
        LOGDEBUG("Nothing available!");
        return 0;
    }*/
  return 0;
}

bool M590Drv::readData(uint8_t *data, bool peek, uint8_t link, bool *conn_close) {
  if (link != _curr_link) return false;

  // see Serial.timedRead
  unsigned long _startMillis = millis();
  do {
    if (gsm->available()) {
      if (peek) {
        *data = gsm->peek();
      }
      else {
        *data = gsm->read();
        _buf_pos--;
      }
      //Serial.print((char)*data, 16);
      return true;
    }
  } while(millis() - _startMillis < 1000);

  // timed out, reset the buffer
  LOGERROR1(F("TIMEOUT:"), _buf_pos);

  _buf_pos = 0;
  _curr_link = -1;
  *data = 0;

  return false;
}

/**
 * Receive the data into a buffer.
 * It reads up to bufSize bytes.
 * @return  received data size for success else -1.
 */
int16_t M590Drv::readDataBuf(uint8_t *buf, uint16_t buf_size, uint8_t link) {
  if (link != _curr_link) return false;

  if(_buf_pos < buf_size) buf_size = _buf_pos;

  for(int i=0; i<buf_size; i++) {
    int c = timedRead();
    //LOGDEBUG(c);
    //Serial.println(c, 16);
    if (c == -1) return -1;

    buf[i] = (uint8_t)c;
    _buf_pos--;
  }

  return buf_size;
}

uint8_t M590Drv::tcpClose(uint8_t link) {
  if (!linkStatus(link)) return true;

  if (send_cmd(F("AT+TCPCLOSE=%d"), 500, link) == TAG_TCP_OK) {
    LOGINFO(F("TCP link closed"));
    return true;
  }
  else {
    LOGERROR1(F("Failed to close TCP link"), link);
    return false;
  }
}

uint8_t M590Drv::powerDown() {
  if (!checkSerial()) return false;

  if (send_cmd(F("AT+CPWROFF"), 500) == TAG_OK) {
    LOGINFO(F("Power off in progress. Completes in 5 secs."));
    return true;
  }
  else {
    LOGERROR(F("Error powering down!"));
    return false;
  }
}

void M590Drv::interact() {
  Serial.println(F("Entering transparent mode..."));
  while (1) {
    while (Serial.available() > 0)
      gsm->write(Serial.read());
    while (gsm->available() > 0)
      Serial.write(gsm->read());
  }
}

////////////////////////////////////////////////////////////////////////////
// Utility functions
////////////////////////////////////////////////////////////////////////////



/*
* Sends the AT command and stops if any of the TAGS is found.
* Extract the string enclosed in the passed tags and returns it in the outStr buffer.
* Returns true if the string is extracted, false if tags are not found of timed out.
*/
bool M590Drv::send_cmd_get(const __FlashStringHelper *cmd, const __FlashStringHelper *startTag, const __FlashStringHelper *endTag, char *outStr, int outStrLen, int init_timeout, ...) {
  char cmdBuf[CMD_BUFFER_SIZE];

  // Arguments
  va_list args;
  va_start (args, init_timeout);
  vsnprintf_P (cmdBuf, CMD_BUFFER_SIZE, (char*)cmd, args);
  va_end (args);

  char _startTag[strlen_P((char*)startTag)+1];
  strcpy_P(_startTag,  (char*)startTag);

  char _endTag[strlen_P((char*)endTag)+1];
  strcpy_P(_endTag,  (char*)endTag);
  LOGDEBUG1(F(">>"), cmd);

  // send AT command to modem
  gsm->println(cmdBuf);
  int idx = locate_tag(_startTag, _endTag, outStr, outStrLen, init_timeout);

  return idx;
}

bool M590Drv::locate_tag(const char *startTag, const char *endTag, char *outStr, int outStrLen, int init_timeout, int final_timeout) {
  int idx;
  bool ret = false;
  outStr[0] = 0;

  emptyBuffer(); // read result until the startTag is found
  idx = read_until(init_timeout, startTag, false);

  if (idx == NUM_TAGS) {
    //Serial.print(">> Found first tag: "); Serial.println(startTag);
    // clean the buffer to get a clean string
    ringBuf.init();
    // start tag found, search the endTag
    idx = read_until(500, endTag, false);
    if (idx == NUM_TAGS) {
      //Serial.print(">> Found 2nd tag: "); Serial.println(endTag);
      // end tag found
      // copy result to output buffer avoiding overflow
      ringBuf.getStrN(outStr, strlen(endTag), outStrLen-1);
      // read the remaining part of the response
      // FIXME NOOOO
      read_until(final_timeout, "\r\n", false, true);
      ret = true;
    }
    else {
      LOGWARN1(F("End tag not found"), startTag);
    }
  }
  else if (idx>=0 and idx<NUM_TAGS) {
    // the command has returned but no start tag is found
    LOGDEBUG1(F("No start tag found:"), idx);
  }
  else {
    // the command has returned but no tag is found
    LOGWARN1(F("No tag found:"), startTag);
  }

  LOGDEBUG1(F("<<"), outStr);
  LOGDEBUG();

  return ret;
}

int M590Drv::read_until(int timeout, const char* tag, bool findTags, bool emptySerBuf) {
  ringBuf.reset();

  char c;
  unsigned long lastRead = millis();
  int ret = -1;

  while (millis() - lastRead < timeout) {
    if(gsm->available()) {
      c = (char)gsm->read();
      LOGDEBUG0(c);
      ringBuf.push(c);

      if (tag != NULL) {
        if (ringBuf.endsWith(tag)) {
          ret = NUM_TAGS;
          return ret;
          //LOGDEBUG1("xxx");
        }
      }
      if (findTags) {
        for (int i=0; i<NUM_TAGS; i++) {
          if (ringBuf.endsWith(MODEM_TAGS[i])) {
            ret = i;
            return ret;
          }
        }
      }
      lastRead = millis();
    }
  }

  if (!emptySerBuf) {
    LOGWARN(F(">>> TIMEOUT >>>"));
  }

  return ret;
}

/*-
int M590Drv::send_cmd(const __FlashStringHelper* cmd, int timeout){
  LOGDEBUG(F("----------------------------------------------"));
  LOGDEBUG1(F(">>"), cmd);

  gsm->println(cmd);

  int idx = read_until(timeout);

  LOGDEBUG1(F("---------------------------------------------- >"), idx);
  LOGDEBUG();

    return idx; 
}
*/

/*
 * Sends the AT command and returns the id of the TAG.
 * The additional arguments are formatted into the command using sprintf.
 * Return -1 if no tag is found.
 */
int M590Drv::send_cmd(const __FlashStringHelper* cmd, int timeout, ...)
{
  char cmdBuf[CMD_BUFFER_SIZE];

  // Arguments
  va_list args;
  va_start(args, timeout);
  vsnprintf_P(cmdBuf, CMD_BUFFER_SIZE, (char*)cmd, args);
  va_end(args);

  emptyBuffer();

  LOGDEBUG(F("----------------------------------------------"));
  LOGDEBUG1(F(">>"), cmdBuf);

  gsm->println(cmdBuf);

  int idx = read_until(timeout);

  LOGDEBUG1(F("---------------------------------------------- >"), idx);
  LOGDEBUG();

  return idx;
}

bool M590Drv::send_cmd_find(const __FlashStringHelper* cmd, const char * tag, int timeout,...)
{
  char cmdBuf[CMD_BUFFER_SIZE];

  va_list args;
  va_start (args, timeout);
  vsnprintf_P (cmdBuf, CMD_BUFFER_SIZE, (char*)cmd, args);
  va_end (args);

  emptyBuffer();

  LOGDEBUG(F("----------------------------------------------"));
  LOGDEBUG1(F(">>"), cmdBuf);

  gsm->println(cmdBuf);

  int idx = read_until(timeout, tag, false);

  LOGDEBUG1(F("---------------------------------------------- >"), idx);
  LOGDEBUG();

  return idx==NUM_TAGS;
}

void M590Drv::emptyBuffer(bool warn) {
  char c;
  int i = 0;
  while(gsm->available() > 0) {
    c = gsm->read();
    if (warn == true) LOGDEBUG0(c);
    i++;
  }
  if (i > 0 and warn == true) {
    LOGDEBUG(F(""));
    LOGDEBUG1(F("Dirty characters in the serial buffer! >"), i);
  }
}

// copied from Serial::timedRead
int M590Drv::timedRead() {
  int _timeout = 1000;
  int c;
  unsigned long _startMillis = millis();
  do {
    c = gsm->read();
    if (c >= 0) return c;
  } while (millis() - _startMillis < _timeout);

  return -1; // -1 indicates timeout
}

/* vim: set ft=cpp ai ts=2 sts=2 et sw=2 sta nowrap nu : */
