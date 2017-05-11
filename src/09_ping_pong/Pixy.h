/*
  Pixy.h - Library for interfacing with Pixy.
  Cobbled together by Omwah and Jasco May 2014
  Based on code originally created by Scott Robinson, October 22, 2013.
  Released into the public domain.

  Modified for I2C by Byung-Cheol Cho May 2017
*/

#ifndef PIXY_H
#define PIXY_H

#include "TPixy.h"
#include <wiringPiI2C.h>

#define PIXY_SYNC_BYTE              0x5a
#define PIXY_SYNC_BYTE_DATA         0x5b
#define PIXY_OUTBUF_SIZE            6

class LinkI2C {
public:
    int init(uint8_t addr) {
        outLen = 0;
        this->addr = addr;
        fd = wiringPiI2CSetup(addr);
        return fd;
    }

    uint16_t getWord() {
        // assume little endian
        uint16_t w;
        uint8_t c, out = 0;
        c = getByte();
        w = getByte();

        w <<= 8;
        w |= c;

        return w;
    }

    uint8_t getByte() {
        return wiringPiI2CRead(fd);
    }

    int getAddr() {
        return addr;
    }

private:
    int addr;
    int fd;
    uint8_t outBuf[PIXY_OUTBUF_SIZE];
    uint8_t outLen;
    uint8_t outIndex;
};


typedef TPixy<LinkI2C> Pixy;

#endif
