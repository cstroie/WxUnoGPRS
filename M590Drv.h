/* This is a library for the M590E GSM Modem 
Written by Brian Ejike*/

#ifndef M590_H
#define M590_H

#include "Stream.h"
#include "IPAddress.h"
#include "Ring_Buffer.h"

#define SIM_PRESENT 1
#define SIM_ABSENT  0

// Default state value for links
#define NA_STATE -1

// no free link (1 or 0)
#define NO_LINK_AVAIL 255

// selected link not available
#define LINK_NOT_AVAIL  255

// maximum number of links
#define MAX_LINK  2

// maximum size of AT command
#define CMD_BUFFER_SIZE 200

enum tcp_state {
  CLOSED      = 0,
  ESTABLISHED = 1,
};

class M590Drv {
  public:
    M590Drv();
    uint8_t begin(Stream * ss, char sim_state);
    void getGMR(char *str, int len);
    void getIMEI(char *str, int len);
    void getCCLK(char *str, int len);
    void setCCLK(const char *str);
    void getCOPS(char *str, int len);
    int getRSSI();
    void pwrSave();
    void setFUN(int fun, int rst);
    void funSleep();
    void funWork();
    void restart();
    int getFUN();
    uint8_t pppConnect(const char *apn, const char *uname = NULL, const char *pwd = NULL);
    uint8_t pppConnect_P(const char *apn, const char *uname = NULL, const char *pwd = NULL);
    uint8_t getIP(IPAddress &ip);
    uint8_t urlResolve(const char *url, IPAddress &ip);
    bool linkStatus(uint8_t link = 0);
    uint8_t tcpConnect(IPAddress& ip, uint16_t port, uint8_t link = 0);
    uint8_t tcpWrite(const uint8_t *data, uint16_t len, uint8_t link = 0);
    bool tcpWrite(const __FlashStringHelper *data, uint16_t len, uint8_t link, bool appendCrLf);
    uint16_t avlData(uint8_t link = 0);
    bool readData(uint8_t *data, bool peek = false, uint8_t link = 0, bool *conn_close = NULL);
    int16_t readDataBuf(uint8_t *buf, uint16_t buf_size, uint8_t link = 0);
    uint8_t tcpClose(uint8_t link = 0);
    uint8_t powerDown();
    void interact();
  private:
    uint8_t checkSerial();
    uint8_t checkGPRS();
    int send_cmd(const __FlashStringHelper* cmd, int timeout=1000, ...);
    bool send_cmd_find(const __FlashStringHelper* cmd, const char * tag=NULL, int timeout=1000, ...);
    bool send_cmd_get(const __FlashStringHelper* cmd, const __FlashStringHelper* startTag, const __FlashStringHelper* endTag, char* outStr, int outStrLen, int init_timeout=1000, ...);
    int read_until(int timeout, const char* tag=NULL, bool findTags=true, bool emptySerBuf=false);
    bool locate_tag(const char* startTag, const char* endTag, char* outStr, int outStrLen, int init_timeout=1000, int final_timeout=1000);
    void emptyBuffer(bool warn = true);
    int timedRead();
    uint8_t SIM_PRESENCE;
    Stream * gsm;
    Ring_Buffer ringBuf;
    int _buf_pos;
    uint8_t _curr_link;
    IPAddress _ip_addr;
    bool _ppp_link;
};

#endif

/* vim: set ft=cpp ai ts=2 sts=2 et sw=2 sta nowrap nu : */
