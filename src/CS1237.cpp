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

#define WAITING_TIME_AFTER_READ 27   //[us] the time to wait for the chip to be ready for a new reading, in the datasheet its said 27us
#define WAITING_TIME_AFTER_POWERON 2 //[ms] the time to wait for the chip to be ready after POWERON, or a changed configuration, in the datasheet its said 2ms
#define WAITING_TIME_AFTER_SLEEP 10  //[us] the time to wait for the chip to be ready after sleep, in the datasheet its said 10us
#define WAITING_TIME_TO_SLEEP 100    //[us] the time to the chip needs to go to sleep, in the datasheet its said 100us

#define NO_READING_ALLOWED_TIME 100 //[us] the time when no reading is allowed this is 100us before a new reading is predicted, this depends on the configured sample rate of the ADC

#ifndef CS1237_global
#define CS1237_global
void (*CS1237_ISRfunc[MAX_ADCS])(void) = {CS1237::ISR0, CS1237::ISR1, CS1237::ISR2, CS1237::ISR3, CS1237::ISR4, CS1237::ISR5, CS1237::ISR6, CS1237::ISR7};
void (*CS1237_timer_ISRfunc[MAX_ADCS])(void) = {CS1237::timer_ISR0, CS1237::timer_ISR1, CS1237::timer_ISR2, CS1237::timer_ISR3, CS1237::timer_ISR4, CS1237::timer_ISR5, CS1237::timer_ISR6, CS1237::timer_ISR7};
bool CS1237_ISRUsed[MAX_ADCS] = {false, false, false, false, false, false, false, false};
CS1237 *CS1237_instance[MAX_ADCS];
hw_timer_t *CS1237_timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
#endif

CS1237::CS1237(uint8_t sck, uint8_t dout)
{
    // search for one of 8 free object spots
    for (uint8_t i = 0; i < MAX_ADCS; i++)
    {
        if (!CS1237_ISRUsed[i])
        {
            _object_number = i;
            CS1237_instance[_object_number] = this;
            break;
        }
    }

    // set the private values of the ADC-configuration variables
    _dout = dout;
    _sck = sck;
    _sleep = true;

    // configure the ADC-Pins
    pinMode(_sck, OUTPUT);
    digitalWrite(_sck, LOW);
    pinMode(_dout, INPUT_PULLUP);
    _value = 0;
    // wait for the chip
    while (digitalRead(_dout))
        ;
    _sleep = false;
}

CS1237::~CS1237()
{
    // save, that this space is free for another object
    CS1237_ISRUsed[_object_number] = false;
    detachInterrupt(_dout);
}

int32_t CS1237::reading()
{
start_sending_reading:
    // wait until the datatransmission is finished
    while (_block_value)
        ;
    noInterrupts();
    if ((_value > 0x7FFFFF) && !_block_value)
        return (_value - 0xFFFFFF);
    else if (!_block_value)
        return _value;
    else // if in the time of the last if statment the _blockvalue has switched go to the beginning and wait for the data transmission
        goto start_sending_reading;
    interrupts();
}

int32_t CS1237::read_without_interrupt(void)
{
    int32_t reading_ = 0;
    // make sure no interrupt reading is in process, if so turn it off
    if (_interrupt_reading)
        end_reading();
    // wait for an interrupt to happen, so that the interrupt won't collide with the data transmission
    sleep(false);
    // turn of interrupts and begin reading of the ADC
    noInterrupts();
    for (uint8_t i = 0; i < 24; i++)
    {
        digitalWrite(_sck, HIGH);
        delayMicroseconds(1);
        reading_ |= digitalRead(_dout) << (23 - i);
        digitalWrite(_sck, LOW);
        delayMicroseconds(1);
    }
    interrupts();
    if (reading_ > 0x7FFFFF)
        reading_ -= 0xFFFFFF;
    return reading_;
}

void CS1237::send_clk_pulses(byte count)
{
    for (uint8_t i = 0; i < count; i++)
    {
        digitalWrite(_sck, HIGH);
        delayMicroseconds(1);
        digitalWrite(_sck, LOW);
        delayMicroseconds(1);
    }
}

byte CS1237::raw_configure(bool write, int32_t *value, byte gain, byte speed, byte channel)
{
    byte register_value;
    value[0] = read_without_interrupt();
    noInterrupts();

    send_clk_pulses(3);
    
    // the ADC DOUT-Pin switches to input
    pinMode(_dout, OUTPUT);
    send_clk_pulses(2);
    // turn dout to output and send the read or write commands to
    byte write_ = (write) ? WRITE_ADC_CONFIG : READ_ADC_CONFIG;
    for (uint8_t i = 0; i < 7; i++)
    {
        digitalWrite(_sck, HIGH);
        delayMicroseconds(1);
        digitalWrite(_dout, ((write_ >> (6 - i)) & 0b00000001));
        digitalWrite(_sck, LOW);
        delayMicroseconds(1);
    }
    // send 1 clock pulse, to set the Pins of the ADCs to input
    send_clk_pulses(1);

    // send or read the configuration to/of the ADCs

    pinMode(_dout, (write) ? OUTPUT : INPUT_PULLUP);
    register_value = (write) ? (gain | speed | channel) : 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        digitalWrite(_sck, HIGH);
        delayMicroseconds(1);
        if (write)
            digitalWrite(_dout, ((register_value >> (7 - i)) & 0b00000001));
        else
            register_value |= digitalRead(_dout) << (7 - i);
        digitalWrite(_sck, LOW);
        delayMicroseconds(1);
    }

    // send 1 clock pulse, to set the Pins of the ADCs to output and pull high
    send_clk_pulses(1);
    interrupts();
    // put the ADC-pins of the MCU in INPUT_Pullup mode again
    pinMode(_dout, INPUT_PULLUP);
    return register_value;
}

bool CS1237::configure(int32_t *value, byte gain, byte speed, byte channel)
{
    bool succeed;
    // try to configure the ADC for at least three times to set an error
    byte try_count = 3;

    byte conf_write;
    byte conf_read;
    for (uint8_t i = 0; (i < try_count) && !succeed; i++)
    {
        conf_write = raw_configure(true, value, gain, speed, channel);
        conf_read = raw_configure(false, value, gain, speed, channel);
        succeed = true;
        if (conf_write != conf_read)
            succeed = false;
    }
    return succeed;
}

void CS1237::start_reading(void)
{
    // wake the ADC up
    sleep(false);

    // the new meausrement is ready
    _time_old_measurement = micros();

    // start the timer and read the first value
    detachInterrupt(digitalPinToInterrupt(_dout));
    _clock_count = 0;
    _block_value = true;
    _value = 0;
    timer_init();
    _interrupt_reading = true;
}

void CS1237::end_reading(void)
{
    timer_stop();
    detachInterrupt(digitalPinToInterrupt(_dout));
    sleep();
    _interrupt_reading = false;
}

void CS1237::sleep(bool sleep_)
{
    if (sleep_)
    {
        digitalWrite(_sck, HIGH);
        delayMicroseconds(WAITING_TIME_TO_SLEEP);
        _sleep = true;
    }
    else
    {
        digitalWrite(_sck, LOW);
        // Wait for an one interrupt to happen, than the chip is surely awake
        while (!digitalRead(_dout))
            ;
        while (digitalRead(_dout))
            ;
        _sleep = false;
    }
}

void CS1237::instanceISR(void)
{
    portENTER_CRITICAL_ISR(&timerMux);
    _block_value = true;
    _time_old_measurement = micros();
    detachInterrupt(digitalPinToInterrupt(_dout));
    _clock_count = 0;
    
    _value = 0;
    timer_init();

    portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR CS1237::timer_init(void)
{
    CS1237_timer = timerBegin(0, TIMER_PRESCALER, true);
    timerAttachInterrupt(CS1237_timer, CS1237_timer_ISRfunc[_object_number], true);
    timerAlarmWrite(CS1237_timer, 2, true);
    timerAlarmEnable(CS1237_timer);
}

void IRAM_ATTR CS1237::timer_stop(void)
{
    timerAlarmDisable(CS1237_timer);
    timerDetachInterrupt(CS1237_timer);
    timerEnd(CS1237_timer);
}

void IRAM_ATTR CS1237::instance_timer_ISR(void)
{
    portENTER_CRITICAL_ISR(&timerMux);

    if (_clock_count % 2) // only every time the _SCK is HIGH read the Dout-Pin
        _value |= digitalRead(_dout) << (23 - int((_clock_count - 1) / 2));

    _clock_count++;
    digitalWrite(_sck, (_clock_count % 2));

    // end the data transmission, if there were 24 clk cycles
    if (_clock_count >= 48)
    {
        _block_value = false;
        attachInterrupt(digitalPinToInterrupt(_dout), CS1237_ISRfunc[_object_number], FALLING);
        timer_stop();
    }

    portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR CS1237::timer_ISR0(void) { CS1237_instance[0]->instance_timer_ISR(); }
void IRAM_ATTR CS1237::timer_ISR1(void) { CS1237_instance[1]->instance_timer_ISR(); }
void IRAM_ATTR CS1237::timer_ISR2(void) { CS1237_instance[2]->instance_timer_ISR(); }
void IRAM_ATTR CS1237::timer_ISR3(void) { CS1237_instance[3]->instance_timer_ISR(); }
void IRAM_ATTR CS1237::timer_ISR4(void) { CS1237_instance[4]->instance_timer_ISR(); }
void IRAM_ATTR CS1237::timer_ISR5(void) { CS1237_instance[5]->instance_timer_ISR(); }
void IRAM_ATTR CS1237::timer_ISR6(void) { CS1237_instance[6]->instance_timer_ISR(); }
void IRAM_ATTR CS1237::timer_ISR7(void) { CS1237_instance[7]->instance_timer_ISR(); }

void IRAM_ATTR CS1237::ISR0(void) { CS1237_instance[0]->instanceISR(); }
void IRAM_ATTR CS1237::ISR1(void) { CS1237_instance[1]->instanceISR(); }
void IRAM_ATTR CS1237::ISR2(void) { CS1237_instance[2]->instanceISR(); }
void IRAM_ATTR CS1237::ISR3(void) { CS1237_instance[3]->instanceISR(); }
void IRAM_ATTR CS1237::ISR4(void) { CS1237_instance[4]->instanceISR(); }
void IRAM_ATTR CS1237::ISR5(void) { CS1237_instance[5]->instanceISR(); }
void IRAM_ATTR CS1237::ISR6(void) { CS1237_instance[6]->instanceISR(); }
void IRAM_ATTR CS1237::ISR7(void) { CS1237_instance[7]->instanceISR(); }