//
//  www.blinkenlight.net
//
//  Copyright 2018 Udo Klein
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program. If not, see http://www.gnu.org/licenses/


// Attention: if you get compiler errors with regards to name mangling
//            then please have a look into the README


#pragma once

static_assert(__BYTE_ORDER__ == __BYTE_ORDER__,
              "MLX90393 library is designed for LITTLE_ENDIAN architecture only");

#define MLX90393_MAJOR_VERSION 1
#define MLX90393_MINOR_VERSION 0
#define MLX90393_PATCH_VERSION 0

// https://gcc.gnu.org/onlinedocs/cpp/Stringification.html
#define EXPAND_THEN_STRINGIFY(s) STRINGIFY(s)
#define STRINGIFY(s) #s
#define MLX90393_VERSION_STRING (EXPAND_THEN_STRINGIFY(MLX90393_MAJOR_VERSION) "." \
                                 EXPAND_THEN_STRINGIFY(MLX90393_MINOR_VERSION) "." \
                                 EXPAND_THEN_STRINGIFY(MLX90393_PATCH_VERSION))

#include <Wire.h>

namespace MLX90393 {

constexpr uint8_t  bitmask(uint8_t bits) {
    return (1<<bits)-1;
}
constexpr uint16_t bitmask(uint8_t bits, uint8_t shift) {
    return uint16_t((1<<bits)-1) << shift;
}

struct Z_SERIES        { enum { REG = 0x0, BITS = 1, SHIFT =  7}; };
struct GAIN_SEL        { enum { REG = 0x0, BITS = 3, SHIFT =  4}; };
struct HALLCONF        { enum { REG = 0x0, BITS = 4, SHIFT =  0}; };

struct TRIG_INT_SEL    { enum { REG = 0x1, BITS = 1, SHIFT = 15}; };
struct COMM_MODE       { enum { REG = 0x1, BITS = 2, SHIFT = 13}; };
struct WOC_DIFF        { enum { REG = 0x1, BITS = 1, SHIFT = 12}; };
struct EXT_TRIG        { enum { REG = 0x1, BITS = 1, SHIFT = 11}; };
struct TCMP_EN         { enum { REG = 0x1, BITS = 1, SHIFT = 10}; };
struct BURST_SEL       { enum { REG = 0x1, BITS = 4, SHIFT =  6}; };
struct BURST_DATA_RATE { enum { REG = 0x1, BITS = 6, SHIFT =  0}; };

struct OSR2            { enum { REG = 0x2, BITS = 2, SHIFT = 11}; };
struct RES_XYZ         { enum { REG = 0x2, BITS = 6, SHIFT =  5}; };
struct RES_X           { enum { REG = 0x2, BITS = 2, SHIFT =  5}; };
struct RES_Y           { enum { REG = 0x2, BITS = 2, SHIFT =  7}; };
struct RES_Z           { enum { REG = 0x2, BITS = 2, SHIFT =  9}; };
struct DIG_FLT         { enum { REG = 0x2, BITS = 3, SHIFT =  2}; };
struct OSR             { enum { REG = 0x2, BITS = 2, SHIFT =  0}; };

struct SENS_TC_HT      { enum { REG = 0x3, BITS = 8, SHIFT =  8}; };
struct SENS_TC_LT      { enum { REG = 0x3, BITS = 8, SHIFT =  0}; };

struct OFFSET_REG {
    enum {
        X = 0x4,
        Y = 0x5,
        Z = 0x6
    };
};

struct THRESHOLD_REG {
    enum {
        WOXY = 0x7,
        WOZ  = 0x8,
        WOT  = 0x9
    };
};

struct AXIS_MASK {
    enum {
        Z = bitmask(1, 3),
        Y = bitmask(1, 2),
        X = bitmask(1, 1),
        T = bitmask(1, 0),
        ALL = Z | Y | X | T
    };
};

struct STATUS_MASK {
    enum {
        BURST          = bitmask(1, 7),
        WAKE_ON_CHANGE = bitmask(1, 6),
        SINGLE_MEASURE = bitmask(1, 5),
        ERROR          = bitmask(1, 4),
        EEC            = bitmask(1, 3),
        RESET_BIT      = bitmask(1, 2),
        D1             = bitmask(1, 1),
        D0             = bitmask(1, 0)
    };
};

struct CMD {
    enum  {
        NOP               = 0x00,
        START_BURST       = 0x10,
        WAKE_ON_CHANGE    = 0x20,
        START_MEASUREMENT = 0x30,
        READ_MEASUREMENT  = 0x40,
        READ_REGISTER     = 0x50,
        WRITE_REGISTER    = 0x60,
        EXIT              = 0x80,
        MEMORY_RECALL     = 0xd0,
        MEMORY_STORE      = 0xe0,
        RESET             = 0xf0
    };
};

struct status_t {
    uint8_t d0:         1;
    uint8_t d1:         1;
    uint8_t rs:         1;
    uint8_t sed:        1;
    uint8_t err:        1;
    uint8_t sm_mode:    1;
    uint8_t woc_mode:   1;
    uint8_t burst_mode: 1;

    bool ok()    const { return err == 0; }
    bool error() const { return err != 0; }

    explicit operator uint8_t const ();
};


union status_u {
    status_t flags;
    uint8_t data;
};

status_t status(uint8_t s) {
    status_u u;
    u.data = s;
    return u.flags;
};

const status_t ERROR = {1, 1, 1, 1, 1, 1, 1, 1};

struct zyxt_t {
    uint16_t z;
    uint16_t y;
    uint16_t x;
    uint16_t t;
};

}  // namespace MLX90393

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
class MLX90393_raw {

public:
    struct I2C {
        enum  {
            BASE_ADDRESS = 0x0c,
            ADDRESS = BASE_ADDRESS | (A1? 2: 0) | (A0? 1: 0)
        };
    };

    static MLX90393::status_t begin();

    // raw command interface
    static MLX90393::status_t sendCommand(uint8_t cmd, uint8_t requested_data_bytes = 0);  // status byte not counted
    static MLX90393::status_t nop();
    static MLX90393::status_t startBurst(uint8_t axis_mask);
    static MLX90393::status_t startWakeOnChange(uint8_t axis_mask);
    static MLX90393::status_t startMeasurement(uint8_t axis_mask);
    static MLX90393::status_t readMeasurement(uint8_t axis_mask, MLX90393::zyxt_t& result);
    static MLX90393::status_t readRegister (uint8_t address, uint16_t& data);
    static MLX90393::status_t writeRegister(uint8_t address, uint16_t  data);
    static MLX90393::status_t exit();
    static MLX90393::status_t memoryRecall();
    static MLX90393::status_t memoryStore();
    static MLX90393::status_t reset();

    // convenience methods
    template <typename PARAMETER_DESCRIPTION> static uint8_t get(uint16_t reg);
    template <typename PARAMETER_DESCRIPTION> static void    set(uint16_t& reg, uint8_t value);

    template <typename PARAMETER_DESCRIPTION, uint8_t size> static uint8_t get (uint16_t(&reg)[size]);
    template <typename PARAMETER_DESCRIPTION, uint8_t size> static void    set (uint16_t(&reg)[size], uint8_t value);

    template <typename PARAMETER_DESCRIPTION> static MLX90393::status_t read(uint8_t& value);
    template <typename PARAMETER_DESCRIPTION> static MLX90393::status_t write(uint8_t value);

    MLX90393::status_t static readResolution (uint8_t& res_x, uint8_t& res_y, uint8_t& res_z);
    MLX90393::status_t static writeResolution(uint8_t  res_x, uint8_t  res_y, uint8_t  res_z);

    bool static dataReady();  // only supported for DRDY_pin >= 0
    void static waitDataReady();  // only supported for DRDY_pin >= 0

};  // class MLX90393_raw


//
//  Implementation starts here
//

MLX90393::status_t::operator uint8_t const() {
    status_u u;
    u.flags = *this;
    return u.data;
};

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
MLX90393::status_t MLX90393_raw<A1, A0, i2c, DRDY_pin>::
begin() {
    if (DRDY_pin >= 0) {
        pinMode(DRDY_pin, INPUT);
    }

    // ensure that MLX90393 will be able to receive commands
    const MLX90393::status_t s = exit();
    if (s.error()) { return s; }

    // ensure that MLX90393 registers will be as expected
    return reset();
}

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
MLX90393::status_t MLX90393_raw<A1, A0, i2c, DRDY_pin>::
sendCommand(uint8_t cmd, uint8_t requested) {
    // request one additional byte (for status)
    const uint8_t required = requested+1;
    i2c.beginTransmission(I2C::ADDRESS);
    if (i2c.write(cmd) == 1)
    if (!i2c.endTransmission())
    if (i2c.requestFrom(uint8_t(I2C::ADDRESS), required) == required) {
        return MLX90393::status(i2c.read());
    }

   return MLX90393::ERROR;
}

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
MLX90393::status_t MLX90393_raw<A1, A0, i2c, DRDY_pin>::
nop() {
    return sendCommand(MLX90393::CMD::NOP);
}

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
MLX90393::status_t MLX90393_raw<A1, A0, i2c, DRDY_pin>::
exit() {
    return sendCommand(MLX90393::CMD::EXIT);
}

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
MLX90393::status_t MLX90393_raw<A1, A0, i2c, DRDY_pin>::
startBurst(uint8_t axis_mask) {
    using namespace MLX90393;
    return sendCommand(CMD::START_BURST | (axis_mask & AXIS_MASK::ALL));
}

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
MLX90393::status_t MLX90393_raw<A1, A0, i2c, DRDY_pin>::
startWakeOnChange(uint8_t axis_mask) {
    using namespace MLX90393;
    return sendCommand(CMD::WAKE_ON_CHANGE | (axis_mask & AXIS_MASK::ALL));
};

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
MLX90393::status_t MLX90393_raw<A1, A0, i2c, DRDY_pin>::
startMeasurement(uint8_t axis_mask) {
    using namespace MLX90393;
    return sendCommand(CMD::START_MEASUREMENT | (axis_mask & AXIS_MASK::ALL));
};

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
MLX90393::status_t MLX90393_raw<A1, A0, i2c, DRDY_pin>::
readMeasurement(uint8_t axis_mask, MLX90393::zyxt_t& result) {
    using namespace MLX90393;

    const uint8_t bytes = (((axis_mask & AXIS_MASK::Z)?2:0) +
                           ((axis_mask & AXIS_MASK::Y)?2:0) +
                           ((axis_mask & AXIS_MASK::X)?2:0) +
                           ((axis_mask & AXIS_MASK::T)?2:0) );
    const uint8_t cmd = CMD::READ_MEASUREMENT | (axis_mask & AXIS_MASK::ALL);
    const status_t s = sendCommand(cmd, bytes);

    if (s.ok()) {
        union data_t {
            uint8_t buffer[8];
            struct {
                zyxt_t zyxt;
            };
        };
        data_t data = {.buffer = {}};

        for (int8_t i = 7; i >= 0; --i) {
            if (axis_mask & bitmask(1, 3)) {
                data.buffer[i] = i2c.read();
            }
            // shift mask bits whenever we have fetched a word
            if (!(i & 1)) { axis_mask <<= 1; }
        }
        result = data.zyxt;
    }

    return s;
}

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
MLX90393::status_t MLX90393_raw<A1, A0, i2c, DRDY_pin>::
readRegister(uint8_t address, uint16_t& data) {
    using namespace MLX90393;

    i2c.beginTransmission(I2C::ADDRESS);
    if (i2c.write(CMD::READ_REGISTER) == 1)
    if (i2c.write((address & 0x3f)<<2)== 1)
    if (!i2c.endTransmission())
    if (i2c.requestFrom(uint8_t(I2C::ADDRESS), uint8_t(3)) == 3) {
        const status_t s = status(i2c.read());
        const uint8_t high = i2c.read();
        const uint8_t low  = i2c.read();

        data = (uint16_t(high)<<8) | low;
        return s;
    }

    return ERROR;
}

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
MLX90393::status_t MLX90393_raw<A1, A0, i2c, DRDY_pin>::
writeRegister(uint8_t address, uint16_t data) {
    using namespace MLX90393;

    i2c.beginTransmission(I2C::ADDRESS);
    if (i2c.write(CMD::WRITE_REGISTER) == 1)
    if (i2c.write(data >> 8) == 1)
    if (i2c.write(uint8_t(data)) == 1)
    if (i2c.write((address & 0x3f) << 2 ) == 1)
    if (!i2c.endTransmission())
    if (i2c.requestFrom(uint8_t(I2C::ADDRESS), uint8_t(1)) == 1) {
        return status(i2c.read());
    }

    return ERROR;
}

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
MLX90393::status_t MLX90393_raw<A1, A0, i2c, DRDY_pin>::
memoryRecall() {
    return sendCommand(MLX90393::CMD::MEMORY_RECALL);
}

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
MLX90393::status_t MLX90393_raw<A1, A0, i2c, DRDY_pin>::
memoryStore() {
    return sendCommand(MLX90393::CMD::MEMORY_STORE);
}

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
MLX90393::status_t MLX90393_raw<A1, A0, i2c, DRDY_pin>::
reset() {
    using namespace MLX90393;
    const status_t s = sendCommand(CMD::RESET);
    // Give device time to complete reset.
    // According to datasheet power on reset takes at
    // most 1.6 ms. We assume software reset will not take
    // longer.
    delay(2);
    return s;
}

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
template <typename PD>
uint8_t MLX90393_raw<A1, A0, i2c, DRDY_pin>::
get(uint16_t reg_data) {
    enum { MASK = MLX90393::bitmask(PD::BITS) };
    return (reg_data >> PD::SHIFT) & MASK;
}

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
template <typename PD>
void MLX90393_raw<A1, A0, i2c, DRDY_pin>::
set(uint16_t& reg, uint8_t value) {
    using namespace MLX90393;
    enum { MASK = bitmask(PD::BITS),
           SHIFTED_MASK = bitmask(PD::BITS, PD::SHIFT) };

    reg &= ~SHIFTED_MASK;
    reg |= ((uint16_t(value & MASK) << PD::SHIFT));
}

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
template <typename PD, uint8_t size>
uint8_t MLX90393_raw<A1, A0, i2c, DRDY_pin>::
get (uint16_t(&reg)[size]) {
    static_assert(PD::REG < sizeof(reg) / 2,
                  "reg[] index out of bounds");
    return get<PD>(reg[PD::REG]);
}

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
template <typename PD, uint8_t size>
void MLX90393_raw<A1, A0, i2c, DRDY_pin>::
set (uint16_t(&reg)[size], uint8_t value) {
    static_assert(PD::REG < sizeof(reg) / 2,
                  "reg[] index out of bounds");
    set<PD>(reg[PD::REG], value);
}

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
template <typename PD>
MLX90393::status_t MLX90393_raw<A1, A0, i2c, DRDY_pin>::
read(uint8_t& value) {
    uint16_t reg_data;
    const MLX90393::status_t status = readRegister(PD::REG, reg_data);
    value = get<PD>(reg_data);
    return status;
}

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
template <typename PD>
MLX90393::status_t MLX90393_raw<A1, A0, i2c, DRDY_pin>::
write(uint8_t value) {
    uint16_t reg_data;
    const MLX90393::status_t status = readRegister(PD::REG, reg_data);
    if (status.error()) { return status; };

    set<PD>(reg_data, value);
    return writeRegister(PD::REG, reg_data);
}

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
MLX90393::status_t MLX90393_raw<A1, A0, i2c, DRDY_pin>::
readResolution (uint8_t& res_x, uint8_t& res_y, uint8_t& res_z) {
    using namespace MLX90393;

    uint16_t reg_data;
    const status_t status = readRegister(RES_XYZ::REG, reg_data);

    res_x = get<RES_X>(reg_data);
    res_y = get<RES_Y>(reg_data);
    res_z = get<RES_Z>(reg_data);

    return status;
}

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
MLX90393::status_t MLX90393_raw<A1, A0, i2c, DRDY_pin>::
writeResolution(uint8_t res_x, uint8_t  res_y, uint8_t  res_z) {
    using namespace MLX90393;

    uint16_t reg_data;
    const status_t status = readRegister(RES_XYZ::REG, reg_data);
    if (status.error()) { return status; };

    set<RES_X>(reg_data, res_x);
    set<RES_Y>(reg_data, res_y);
    set<RES_Z>(reg_data, res_z);

    return writeRegister(RES_XYZ::REG, reg_data);
}

template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
bool MLX90393_raw<A1, A0, i2c, DRDY_pin>::
dataReady() {
    static_assert(DRDY_pin >= 0,
                  "dataReady() not supported without valid DRDY_pin");
    return digitalRead(DRDY_pin);

}


template <uint8_t A1, uint8_t A0, TwoWire &i2c, int8_t DRDY_pin>
void MLX90393_raw<A1, A0, i2c, DRDY_pin>::
waitDataReady() {
    static_assert(DRDY_pin >= 0,
                  "waitDataReady() not supported without valid DRDY_pin");
    while (!dataReady()) { /* busy waiting*/ }
}