/*!
 * @file CS1237.cpp
 *
 * @mainpage JW CS1237 Library
 *
 * @section intro_sec Introduction
 *
 * This is a library for the CS1237 24bit Amplifier for loadcells
 *
 * The Chip communicates through a 2-wire SPI interface SCLK, DRDY/DOUT, 
 * 2 GPIO-pins are required to interface with the CS1237-IC.
 *
 * @section author Author
 *
 * Written by Julian Weidinger for an University Project.
 * 
 * @section license License
 *
 * BSD license, all text above must be included in any redistribution
 */

#include "CS1237.h"

#define WAITING_TIME_AFTER_READ     27 //[us] the time to wait for the chip to be ready for a new reading, in the datasheet its said 27us
#define WAITING_TIME_AFTER_POWERON  2   //[ms] the time to wait for the chip to be ready after POWERON, or a changed configuration, in the datasheet its said 2ms
#define WAITING_TIME_AFTER_SLEEP    10  //[us] the time to wait for the chip to be ready after sleep, in the datasheet its said 10us
#define WAITING_TIME_TO_SLEEP       100 //[us] the time to the chip needs to go to sleep, in the datasheet its said 100us

CS1237::CS1237(uint8_t count, uint8_t sck, uint8_t *dout){
    _count = count;
    _dout = dout;
    _sck = sck;

    _last_meassure_time = (uint64_t *) malloc(_count*sizeof(uint64_t)); //specify the needed location

    pinMode(_sck, OUTPUT);
    digitalWrite(_sck, LOW);
    for(uint8_t i = 0; i<count; i++) {
        pinMode(dout[i], INPUT_PULLUP);
    }
    delayMicroseconds(WAITING_TIME_AFTER_SLEEP);
}   

void CS1237::send_clk_pulses(byte count){
    
    for (uint8_t i = 0; i < count; i++) {
        digitalWrite(_sck, HIGH);
        delayMicroseconds(1);
        digitalWrite(_sck, LOW);
        delayMicroseconds(1);
    }
}

void CS1237::raw_configure(bool write, byte* register_value, int32_t *value, byte gain, byte speed, byte channel){
    //first 24 pulses read the ADC data
    if(_sleep){ delayMicroseconds(WAITING_TIME_AFTER_SLEEP); _sleep = false;}
    while(!ready());
    read(value);

    noInterrupts();

    //the ADC DOUT-Pin switches to input
    for(uint8_t i = 0; i<_count; i++) {
        pinMode(_dout[i], OUTPUT);
    }
    send_clk_pulses(2);

    //turn dout to output and send the read or write commands to
    byte write_ = (write) ? WRITE_ADC_CONFIG : READ_ADC_CONFIG;
    for (uint8_t i = 0; i < 7; i++) {
        digitalWrite(_sck, HIGH);
        delayMicroseconds(1);
        for(uint8_t j = 0; j<_count; j++) {
           digitalWrite(_dout[j], ((write_ >> (6-i)) & 0b00000001));
        }
        digitalWrite(_sck, LOW);
        delayMicroseconds(1);
    }
    //send 1 clock pulse, to set the Pins of the ADCs to input
    send_clk_pulses(1);
    
    //send or read the configuration to/of the ADCs
    for(uint8_t i = 0; i<_count; i++) {
        pinMode(_dout[i], (write) ? OUTPUT : INPUT_PULLUP);
        register_value[i] = (write) ? (gain | speed | channel) : 0;
    }
    
    for (uint8_t i = 0; i < 8; i++) {
        digitalWrite(_sck, HIGH);
        delayMicroseconds(1);
        for(uint8_t j = 0; j < _count; j++) {
            if(write)   digitalWrite(_dout[j], ((register_value[j] >> (7-i)) & 0b00000001));
            else        register_value[j] |= digitalRead(_dout[j]) << (7-i);   
        }
        digitalWrite(_sck, LOW);
        delayMicroseconds(1);
    }

    //send 1 clock pulse, to set the Pins of the ADCs to output and pull high
    send_clk_pulses(1);
    interrupts();
    //put the ADC-pins of the MCU in INPUT_Pullup mode again
    for(uint8_t i = 0; i<_count; i++) {
        pinMode(_dout[i], INPUT_PULLUP);
        _last_meassure_time[i] = micros();
    }
    delay(WAITING_TIME_AFTER_POWERON);
    
    
}

bool CS1237::configure(int32_t *result, byte gain, byte speed, byte channel){
    bool succeed;

    //try to configure the ADC for at least three times to set an error
    byte try_count = 3;

    byte conf_write[_count];
    byte conf_read[_count];
    for(uint8_t i = 0; (i < try_count) && !succeed; i++) {
        raw_configure(true, conf_write, result, gain, speed, channel);
        raw_configure(false,conf_read, result, gain, speed, channel);
        succeed = true;
        for(uint8_t j = 0; j < _count; j++){
            if(conf_write[j] != conf_read[j])   succeed = false;
        }
    }

    return succeed;
}

bool CS1237::ready(void){
    
    for(uint8_t i = 0; i<_count; i++) {
        if(micros() - _last_meassure_time[i] <= WAITING_TIME_AFTER_READ) return false;
        if(digitalRead(_dout[i]))   return false;
    }
    return true;
}

void CS1237::read(int32_t *value){
    if(_sleep){ delayMicroseconds(WAITING_TIME_AFTER_SLEEP); _sleep = false;}
    //set every value to zero
    for(uint8_t i = 0; i<_count; i++) {
        value[i] = 0;
    }

    //wait until the data of all chips is available
    while(!ready());

    //turn of interrupts and begin reading of all ADCs in parallel
    noInterrupts();
    for (uint8_t i = 0; i < 24; i++) {
        digitalWrite(_sck, HIGH);
        delayMicroseconds(1);
        for(uint8_t j = 0; j<_count; j++) {
            value[j] |= digitalRead(_dout[j]) << (23 - i);
        }
        digitalWrite(_sck, LOW);
        delayMicroseconds(1);
    }

    send_clk_pulses(3);

    interrupts();
    for(uint8_t i = 0; i<_count; i++) {
        if (value[i] > 0x7FFFFF) value[i] -= 0xFFFFFF;
        _last_meassure_time[i] = micros();
    }
}

void CS1237::sleep(void){
    digitalWrite(_sck, HIGH);
    _sleep = true;
}