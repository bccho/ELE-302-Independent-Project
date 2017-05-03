//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//

/*
  Pixy.h - Library for interfacing with Pixy.
  Cobbled together by Omwah and Jasco May 2014
  Based on code originally created by Scott Robinson, October 22, 2013.
  Released into the public domain.
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

    /*
    int8_t send(uint8_t *data, uint8_t len) {
        if (len > PIXY_OUTBUF_SIZE || outLen != 0) {
            return -1;
        }

        memcpy(outBuf, data, len);
        outLen = len;
        outIndex = 0;

        wiringPiI2CWrite(fd, out);

        return len;
    }
    */

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
