/*
 SparkFun Si7021 Temperature and Humidity Breakout 
 By: Joel Bartlett
 SparkFun Electronics
 Date: December 10, 2015
 
 This is an Arduino library for the Si7021 Temperature and Humidity Sensor Breakout
 
 This library is based on the following libraries:

 HTU21D Temperature / Humidity Sensor Library
 By: Nathan Seidle
 https://github.com/sparkfun/HTU21D_Breakout/tree/master/Libraries

 Arduino Si7010 relative humidity + temperature sensor
 By: Jakub Kaminski, 2014
 https://github.com/teoqba/ADDRESS

 This Library is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This Library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 For a copy of the GNU General Public License, see
 <http://www.gnu.org/licenses/>.
 */

#ifndef MJS_SI7021_H
#define MJS_SI7021_H

#include <Arduino.h>
#include <SoftWire.h>

/****************Si7021 & HTU21D Definitions***************************/

#define ADDRESS      0x40

#define TEMP_MEASURE_HOLD	0xE3
#define HUMD_MEASURE_HOLD	0xE5
#define TEMP_MEASURE_NOHOLD	0xF3
#define HUMD_MEASURE_NOHOLD	0xF5
#define TEMP_PREV	0xE0

#define WRITE_USER_REG	0xE6
#define READ_USER_REG	0xE7
#define WRITE_HEAT_REG	0x51
#define READ_HEAT_REG	0x11
#define SOFT_RESET	0xFE

#define HTRE        0x02
#define _BV(bit) (1 << (bit))

#define CRC_POLY 0x988000 // Shifted Polynomial for CRC check

// Error codes
#define I2C_TIMEOUT 	998
#define BAD_CRC		999


/****************Si7021 & HTU21D Class**************************************/
class mjs_si7021 {
public:
    // Constructor
    mjs_si7021(SoftWire*);

    char *begin();

    // Si7021 & HTU21D Public Functions
    float    getHumi();
    float    readTemp();
    float    getTemp();
    float    readTempF();
    float    getTempF();
    void     heaterOn();
    void     heaterOff();
    bool     staHeater();
    void     setHeater(uint8_t);
    uint8_t  getHeater();
    void     changeResolution(uint8_t);
    void     reset();
    uint8_t  checkID();
    uint32_t getSerial();
    uint8_t  serial[8];
    char     typenaam[8];
    char     fmtTemp[16];
    char     fmtHumi[16];


private:
    //Si7021 & HTU21D Private Functions
    SoftWire* _wire;
    uint16_t makeMeasurement(uint8_t command);
    void     writeReg(uint8_t value);
    uint8_t  readReg();
    void     writeHReg(uint8_t value);
    uint8_t  readHReg();
};

#endif
