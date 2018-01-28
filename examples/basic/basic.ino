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

#include <MLX90393_raw.h>

const int8_t drdy_pin = 10;
const uint32_t baudrate = 57600;

// I2C
// Arduino A4 = SDA
// Arduino A5 = SCL
typedef MLX90393_raw<0, 0, Wire, drdy_pin> MLX;

void belly_up() {
    // The proper way to error handling is of course NOT
    // to go belly up. Instead you might want to introduce something
    // slightly more sophisticated.
    Serial.println(F("Something failed"));
    while (true);
}

void setup() {
    using namespace MLX90393;

    Serial.begin(baudrate);

    if (MLX::begin().error()) {
        Serial.print(F("Failed to properly initialize MLX90393"));
        belly_up();
    };

    uint16_t reg[5];

    // uncomment the block below if you only want to modify some registers
    // leave it untouched if you want to explicitly set all of them
    /*
    for (uint8_t adr=0; adr < 5; ++adr) {
        if (MLX::readRegister(adr, reg[adr]).error()) {
            Serial.println(F("Failed to read register: "));
            Serial.println(adr);
            belly_up();
        }
    }
    */

    // set whatever you need here
    // you migth want to use the debug helper to figure out what you need / want
    MLX::set<Z_SERIES>(reg, 0);
    MLX::set<GAIN_SEL>(reg, 0);
    MLX::set<HALLCONF>(reg, 0);
    MLX::set<TRIG_INT_SEL>(reg, 0);
    MLX::set<COMM_MODE>(reg, 0);
    MLX::set<WOC_DIFF>(reg, 0);
    MLX::set<EXT_TRIG>(reg, 0);
    MLX::set<TCMP_EN>(reg, 0);
    MLX::set<BURST_SEL>(reg, 0);
    MLX::set<BURST_DATA_RATE>(reg, 0);
    MLX::set<OSR2>(reg, 0);
    MLX::set<RES_X>(reg, 0);
    MLX::set<RES_Y>(reg, 0);
    MLX::set<RES_Z>(reg, 0);
    MLX::set<DIG_FLT>(reg, 0);
    MLX::set<OSR>(reg, 0);
    MLX::set<SENS_TC_HT>(reg, 0);
    MLX::set<SENS_TC_LT>(reg, 0);

    for (uint8_t adr=0; adr < 5; ++adr) {
        if (MLX::writeRegister(adr, reg[adr]).error()) {
            Serial.println(F("Failed to write register: "));
            Serial.println(adr);
            belly_up();
        }
    }

    // alternatively you might want to call
    // template <typename PARAMETER_DESCRIPTION> static MLX90393::status_t write(uint8_t value);
    // e.g.
    // MLX::write(OSR, 0);
    // to modify just some parameters.
    // However this is less efficient if you need to set all values
    // as it requires to read the register, set the value
    // and write it back.
}

void loop() {
    if (MLX::startMeasurement(MLX90393::AXIS_MASK::ALL).error()) {
        Serial.println(F("Failed to start measurement"));
        belly_up();
    }
    MLX::waitDataReady();

    MLX90393::zyxt_t result;
    if (MLX::readMeasurement(MLX90393::AXIS_MASK::ALL, result).error()) {;
        Serial.println(F("Failed to read measurement"));
        belly_up();
    }

    Serial.print(F("X, Y, Z, T: "));
    Serial.print(result.x);
    Serial.print(F(", "));
    Serial.print(result.y);
    Serial.print(F(", "));
    Serial.print(result.z);
    Serial.print(F(", "));
    Serial.println(result.t);
}