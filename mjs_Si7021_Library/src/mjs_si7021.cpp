/*
 SparkFun Si7021 Temperature and HUmidity Breakout 
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

#if defined(ARDUINO)
// #include "Wire.h"
// #include "SparkFun_Si7021_Breakout_Library.h"
#include "mjs_si7021.h"
#include "SoftWire.h"
// #elif defined(SPARK)
// #include "SparkFun_Si7021_Breakout_Library/SparkFun_Si7021_Breakout_Library.h"
#endif


#define DEBUG false


//Initialize
mjs_si7021::mjs_si7021(SoftWire* swp) {
    _wire = swp;
}


char *mjs_si7021::begin(void) {
    _wire->setDelay_us(5);
    _wire->setTimeout_ms(200);
    _wire->begin();
    fmtTemp[0] = 0;
    fmtHumi[0] = 0;
    delay(300);

    uint8_t ID_Temp_Hum = checkID();

    //int x = 0;
    if(ID_Temp_Hum == 0x15)  //Ping CheckID register
        sprintf(typenaam, "Si7021");
    else if(ID_Temp_Hum == 0x32)
        sprintf(typenaam, "HTU21D");
    else
        sprintf(typenaam, "0x%X", ID_Temp_Hum);

    return typenaam;
}


/****************Si7021 & HTU21D Functions**************************************/


float mjs_si7021::getHumi() {
    // Measure the relative humidity
    uint16_t RH_Code = makeMeasurement(HUMD_MEASURE_NOHOLD);
    float result = (125.0*RH_Code/65536)-6;
    int16_t humi = 100. * result;
    if (humi >= 0) {
        sprintf(fmtHumi, "+%d.%02d", humi/100, humi%100);
    } else {
        sprintf(fmtHumi, "-%d.%02d", abs(humi)/100, abs(humi)%100);
    }
    return result;
}

float mjs_si7021::readTemp() {
    // Read temperature from previous RH measurement.
    uint16_t temp_Code = makeMeasurement(TEMP_PREV);
    float result = (175.72*temp_Code/65536)-46.85;
    return result;
}

float mjs_si7021::getTemp() {
    // Measure temperature
    uint16_t temp_Code = makeMeasurement(TEMP_MEASURE_NOHOLD);
    float result = (175.72*temp_Code/65536)-46.85;
    int16_t temp = 100. * result;
    if (temp >= 0) {
        sprintf(fmtTemp, "+%d.%02d", temp/100, temp%100);
    } else {
        sprintf(fmtTemp, "-%d.%02d", abs(temp)/100, abs(temp)%100);
    }
    return result;
}

//Give me temperature in Fahrenheit
float mjs_si7021::readTempF() {
    return((readTemp() * 1.8) + 32.0); // Convert celsius to fahrenheit
}

float mjs_si7021::getTempF() {
    return((getTemp() * 1.8) + 32.0); // Convert celsius to fahrenheit
}


void mjs_si7021::heaterOn() {
    // Turns on the ADDRESS heater
    uint8_t regVal = readReg();
    regVal |= _BV(HTRE);
    //turn on the heater
    writeReg(regVal);
}

void mjs_si7021::heaterOff() {
    // Turns off the ADDRESS heater
    uint8_t regVal = readReg();
    regVal &= ~_BV(HTRE);
    writeReg(regVal);
}

bool mjs_si7021::staHeater() {
    // Status of the ADDRESS heater
    uint8_t regVal = readReg();
    regVal &= _BV(HTRE);
    bool result = (regVal != 0);
    if (DEBUG) {
        Serial.print("staHeater:"); 
        Serial.println(result);
    }
    return (result);   // true = on
}

void mjs_si7021::setHeater(uint8_t volume) {
    // writes heater
    if (volume > 15) volume = 15;
    uint8_t regVal = readHReg();
    regVal &= 0xF0;
    regVal |= volume & 0xF;
    if (DEBUG) {
        Serial.print("setHeater:volume="); 
        Serial.println(volume);
        Serial.print("setHeater:regVal="); 
        Serial.println(regVal);
    }
    writeHReg(regVal);
}

uint8_t mjs_si7021::getHeater() {
    // reads heater volume
    uint8_t regVal = readHReg();
    if (DEBUG) {
        Serial.print("getHeater:HReg="); 
        Serial.println(regVal);
    }
    return regVal & 0xF;
}


void mjs_si7021::changeResolution(uint8_t i) {
    // Changes to resolution of ADDRESS measurements.
    // Set i to:
    //      RH         Temp
    // 0: 12 bit       14 bit (default)
    // 1:  8 bit       12 bit
    // 2: 10 bit       13 bit
    // 3: 11 bit       11 bit
    // bug: first humidity measurement after startup is negative !

    uint8_t regVal = readReg();
    // zero resolution bits
    regVal &= 0b011111110;
    switch (i) {
        case 1:
            regVal |= 0b00000001;
            break;
        case 2:
            regVal |= 0b10000000;
            break;
        case 3:
            regVal |= 0b10000001;
        default:
            regVal |= 0b00000000;
        break;
    }
    // write new resolution settings to the register
    writeReg(regVal);
}


void mjs_si7021::reset() {
    //Reset user resister
    _wire->beginTransmission(ADDRESS);   // copied from Adafruit
    writeReg(SOFT_RESET);
    _wire->endTransmission();   // copied from Adafruit
}


uint8_t mjs_si7021::checkID() {
    uint8_t ID_1;

    // Check device Type
    _wire->beginTransmission(ADDRESS);
    _wire->write(0xFC);
    _wire->write(0xC9);
    _wire->endTransmission();

    _wire->requestFrom(ADDRESS,1);

    ID_1 = _wire->read();

    return(ID_1);
}


uint32_t mjs_si7021::getSerial() {
    uint32_t myserial;
    uint8_t tel;

    for (tel = 0; tel < 8; tel++) serial[tel] = 0;

    // Check Type e.g. 15FFB5FF
    _wire->beginTransmission(ADDRESS);
    _wire->write(0xFC);
    _wire->write(0xC9);
    _wire->endTransmission();
    _wire->requestFrom(ADDRESS,4);
    for (tel = 0; tel < 4; tel++) {
        serial[tel] = _wire->read();
        if (DEBUG) Serial.print(serial[tel], HEX);
    }
    if (DEBUG) Serial.print("-");

    // Check device ID (serial)
    _wire->beginTransmission(ADDRESS);
    _wire->write(0xFA);
    _wire->write(0x0F);
    _wire->endTransmission();
    _wire->requestFrom(ADDRESS,4);
    for (tel = 4; tel < 8; tel++) {
        serial[tel] = _wire->read();
        if (DEBUG) Serial.print(serial[tel], HEX);
    }

    if (DEBUG) Serial.println();
    myserial = ((uint32_t)serial[4])<<24 | ((uint32_t)serial[5])<<16 | ((uint32_t)serial[6])<<8 | (uint32_t)serial[7];
    return myserial;
}


uint16_t mjs_si7021::makeMeasurement(uint8_t command) {
    // Take one ADDRESS measurement given by command.
    // It can be either temperature or relative humidity
    // TODO: implement checksum checking

    int16_t nBytes = 3;
    // if we are only reading old temperature, read only msb and lsb
    if (command == TEMP_PREV) nBytes = 2;

    _wire->beginTransmission(ADDRESS);
    _wire->write(command);
    _wire->endTransmission();
    // When not using clock stretching (*_NOHOLD commands) delay here
    // is needed to wait for the measurement.
    // According to datasheet the max. conversion time is ~22ms
    delay(100);   // Adafruit 20

    _wire->requestFrom(ADDRESS,nBytes);
    if(_wire->available() < nBytes)
          return 0xFFFF;
    
    unsigned int msb = _wire->read();
    unsigned int lsb = _wire->read();
    // uint8_t chk = 
    _wire->read();   // not checked yet
    // Clear the last to bits of LSB to 00.
    // According to datasheet LSB of RH is always xxxxxx10
    lsb &= 0xFC;
    unsigned int measurement = msb << 8 | lsb;

    return measurement;
}


void mjs_si7021::writeReg(uint8_t value) {
    // Write to user register on ADDRESS
    _wire->beginTransmission(ADDRESS);
    _wire->write(WRITE_USER_REG);
    _wire->write(value);
    _wire->endTransmission();
}

uint8_t mjs_si7021::readReg() {
    // Read from user register on ADDRESS
    _wire->beginTransmission(ADDRESS);
    _wire->write(READ_USER_REG);
    _wire->endTransmission();
    _wire->requestFrom(ADDRESS,1);
    uint8_t regVal = _wire->read();
    return regVal;
}
// Adafruit has readReg16()

void mjs_si7021::writeHReg(uint8_t value) {
    // Write to user register on ADDRESS
    _wire->beginTransmission(ADDRESS);
    _wire->write(WRITE_HEAT_REG);   // 
    // _wire->write(value & 0xF);
    _wire->write(value);
    _wire->endTransmission();
}

uint8_t mjs_si7021::readHReg() {
    // Read from user register on ADDRESS
    _wire->beginTransmission(ADDRESS);
    _wire->write(READ_HEAT_REG);
    _wire->endTransmission();
    _wire->requestFrom(ADDRESS,1);
    uint8_t regVal = _wire->read();
    return regVal;
}
