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

#ifndef _TPIXY_H
#define _TPIXY_H

#include <wiringPi.h>
#include <stdint.h>
#include <iostream>
#include <cstdlib>
#include <cstring>

#define PIXY_INITIAL_ARRAYSIZE      30
#define PIXY_MAXIMUM_ARRAYSIZE      130
#define PIXY_START_WORD             0xaa55
#define PIXY_START_WORDX            0x55aa
#define PIXY_DEFAULT_ADDR           0x54  // I2C

struct Block {
    void print() {
        std::cout << "sig: " << signature << " x: " << x << " y: " << y << " width: " << width << " height: " << height << std::endl;
    }
    uint16_t signature;
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
};


template <class LinkType> class TPixy {
public:
    TPixy(uint8_t addr = PIXY_DEFAULT_ADDR, int channel = 0, int chipSelect = 24);
    ~TPixy();

    uint16_t getBlocks(uint16_t maxBlocks = 1000);
    int8_t setServos(uint16_t s0, uint16_t s1);
    LinkType getLink();

    Block *blocks;

private:
    bool getStart();
    void resize();

    LinkType link;
    bool skipStart;
    uint16_t blockCount;
    uint16_t blockArraySize;
};


template <class LinkType> TPixy<LinkType>::TPixy(uint8_t addr, int channel, int pinSelect)
{
    if (channel < 0 || channel > 1) {
        std::cerr << "Invalid channel. Setting channel = 0" << std::endl;
        channel = 0;
    }
    skipStart = false;
    blockCount = 0;
    blockArraySize = PIXY_INITIAL_ARRAYSIZE;
    blocks = (Block *)malloc(sizeof(Block) * blockArraySize);
    if (link.init(addr, channel, pinSelect) < 0) {
        std::cerr << "Could not initialize SPI on channel " << channel << std::endl;
    }
}

template <class LinkType> TPixy<LinkType>::~TPixy()
{
    free(blocks);
}

template <class LinkType> bool TPixy<LinkType>::getStart()
{
    uint16_t w, lastw;

    lastw = 0xffff;

    while(true) {
        w = link.getWord();

        if (w == 0 && lastw == 0) {
            delayMicroseconds(10);
            return false;
        } else if (w == PIXY_START_WORD && lastw == PIXY_START_WORD) {
            return true;
        } else if (w == PIXY_START_WORDX) {
            std::cerr << "reorder" << std::endl;
            link.getByte(); // resync
        }

        lastw = w;
    }
}

template <class LinkType> void TPixy<LinkType>::resize()
{
    Block *newBlocks;
    blockArraySize += PIXY_INITIAL_ARRAYSIZE;
    newBlocks = (Block *)malloc(sizeof(Block) * blockArraySize);
    memcpy(newBlocks, blocks, sizeof(Block)*blockCount);
    free(blocks);
    blocks = newBlocks;
}

template <class LinkType> uint16_t TPixy<LinkType>::getBlocks(uint16_t maxBlocks)
{
    uint8_t i;
    uint16_t w, checksum, sum;
    Block *block;

    if (!skipStart) {
        if (getStart() == false) {
            return 0;
        }
    } else {
        skipStart = false;
    }

    for(blockCount = 0; blockCount < maxBlocks && blockCount < PIXY_MAXIMUM_ARRAYSIZE;) {
        checksum = link.getWord();

        if (checksum == PIXY_START_WORD) { // we've reached the beginning of the next frame
            skipStart = true;
            return blockCount;
        } else if (checksum == 0) {
            return blockCount;
        }

        if (blockCount > blockArraySize) {
            resize();
        }

        block = blocks + blockCount;

        for (i = 0, sum = 0; i < sizeof(Block) / sizeof(uint16_t); i++) {
            w = link.getWord();
            sum += w;
            *((uint16_t *)block + i) = w;
        }

        if (checksum == sum) {
            blockCount++;
        } else {
            std::cerr << "cs error" << std::endl;
        }

        w = link.getWord();

        if (w != PIXY_START_WORD) {
            return blockCount;
        }
    }
}

template <class LinkType> LinkType TPixy<LinkType>::getLink() {
    return link;
}

template <class LinkType> int8_t TPixy<LinkType>::setServos(uint16_t s0, uint16_t s1)
{
    uint8_t outBuf[6];

    outBuf[0] = 0x00;
    outBuf[1] = 0xff;
    *(uint16_t *)(outBuf + 2) = s0;
    *(uint16_t *)(outBuf + 4) = s1;

    return link.send(outBuf, 6);
}

#endif
