/*
 * Copyright (c) 2014, Majenko Technologies
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 
 *  1. Redistributions of source code must retain the above copyright notice, 
 *     this list of conditions and the following disclaimer.
 * 
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 * 
 *  3. Neither the name of Majenko Technologies nor the names of its contributors may be used
 *     to endorse or promote products derived from this software without 
 *     specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <TLC59116.h>

const uint8_t TLC59116::pinmap[8] = {
    10 << 4 | 14,
    11 << 4 | 15,
    5 << 4 | 1,
    6 << 4 | 2,
    7 << 4 | 3,
    9 << 4 | 13,
    8 << 4 | 12,
    4 << 4 | 0
};

const uint8_t TLC59116::numbers[] = {
    0b00111111,
    0b00000110,
    0b01011011,
    0b01001111,
    0b01100110,
    0b01101101,
    0b01111101,
    0b00100111,
    0b01111111,
    0b01101111
};

static inline void wiresend(unsigned char x) {
#if ARDUINO >= 100
    Wire.write((unsigned char)x);
#else
    Wire.send(x);
#endif
}

static inline unsigned char wirerecv(void) {
#if ARDUINO >= 100
    return Wire.read();
#else
    return Wire.receive();
#endif
}

TLC59116::TLC59116() {
    _addr = 0;
    _begun = 0;
}

TLC59116::TLC59116(uint8_t addr) {
    _addr = addr;
    _begun = 0;
}

uint8_t TLC59116::begin() {
    if (_begun != 0) {
        return 0;
    }

    Wire.begin();

    uint8_t err;
    if ((err = writeRegister(TLC59116_MODE1, 0x01))) return err;

    delay(1);
 
    if ((err = writeRegister(TLC59116_MODE2, 0x00))) return err;
    if ((err = writeRegister(TLC59116_LEDOUT0, 0b10101010))) return err;
    if ((err = writeRegister(TLC59116_LEDOUT1, 0b10101010))) return err;
    if ((err = writeRegister(TLC59116_LEDOUT2, 0b10101010))) return err;
    if ((err = writeRegister(TLC59116_LEDOUT3, 0b10101010))) return err;

    for (uint8_t ch = 0; ch < 16; ch++) {
        if ((err = this->analogWrite(ch, 0))) return err;
    }


    setPinMapping(pinmap);
    _begun = 1;
    return 0;
}


uint8_t TLC59116::writeRegister(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(TLC59116_BASEADDR | (_addr & 0x0F));
    wiresend(reg);
    wiresend(val);
    return Wire.endTransmission();
}

uint8_t TLC59116::analogWrite(uint8_t chan, uint8_t b) {
    return writeRegister(TLC59116_PWM0 + (chan & 0x0F), b);
}

uint8_t TLC59116::displayNumber(uint8_t number, uint8_t b) {
    uint8_t tens  = (number / 10) % 10;
    uint8_t units = number % 10;
    uint8_t err;

    if (tens == 0) {
        for (int i = 0; i < 8; i++) {
            int seg = _currentPinMapping[i] >> 4;
            if ((err = this->analogWrite(seg, 0))) return err;
        }
    } else {
        int num = numbers[tens];
        for (int i = 0; i < 8; i++) {
            int seg = _currentPinMapping[i] >> 4;
            if ((err = this->analogWrite(seg, (num & (1 << i)) ? b : 0))) return err;
        }
    }

    int num = numbers[units];
    for (int i = 0; i < 8; i++) {
        int seg = _currentPinMapping[i] & 0x0F;
        if ((err = this->analogWrite(seg, (num & (1 << i)) ? b : 0))) return err;
    }

    return 0;
}

void TLC59116::setPinMapping(const uint8_t *map) {
    _currentPinMapping = map;
}
