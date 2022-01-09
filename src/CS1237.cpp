/*!
 * @file CS1237.cpp
 *
 * @mainpage ADC (CS1237) Library from Julian Weidinger 
 *
 * @section intro_sec Introduction
 *
 * This is the library for the ADC "CS1237" from Chipsea. the translated datasheet can be found [here](https://github.com/rafaellcoellho/cs1237-datasheet/blob/master/cs1237_datasheet.pdf)
 * 
 * The library is designed to be used with one or two ADCs. The reason is, that if high frequency mode (1280Hz) is used, the timing is critical. 
 * 
 * The gain (1,2,64,128), the sample rate(10Hz,40Hz,640Hz,1280Hz), the reference voltage (internal, or external) 
 * and the channel(external diferential signal, temperature sensor) can be independent configured.
 * 
 * You have to use one clock- and one data-line for each ADC. I have experimented with multiple ADC-Chips on one clock-line, but it is impossible to snchronize these chips.
 * I think the internal cristals are not exact. Synchronisation is very important, because depending on the configured sample rate there is a regular interrupt from the ADC Chip on the data line. 
 * If the data transmission is in progress, while the interrupt happens, the transmitted value is wrong.
 * 
 * This library is written with hardware and timer interrupt service routines, so it can run in the background.
 * 
 * The source code and one example is available on [my github page](https://github.com/JulianWeidinger/CS1237_ADC_Library)
 * 
 * @section dependencies Dependencies
 * 
 * This library was only tested on the ESP32 DEV_KIT_V4
 * 
 *  - At this time it is only allowed to connect two ADCs with one clock and one data line each.
 *  - you could use nearly any GPIO for the clock lines and any GPIO with an interrupt function for the Data line.
 *
 * @section author Author
 *
 * Written by Julian Weidinger for an university Project at the University of applied science.
 *
 */

#include "CS1237.h"


#ifndef CS1237_global
//! define the global variables only once
#define CS1237_global

//! @param CS1237_ISRfunc a global function pointer to the specific static hardware interrupt function.
void (*CS1237_ISRfunc[MAX_ADCS])(void) = {CS1237::ISR0, CS1237::ISR1};
//! @param CS1237_timer_ISRfunc a global function pointer to the specific static timer interrupt function.
void (*CS1237_timer_ISRfunc[MAX_ADCS])(void) = {CS1237::timer_ISR0, CS1237::timer_ISR1};
//! @param CS1237_ISR_used a global flag to know, wich interrupt functions are already blocked by other instances.
bool CS1237_ISR_used[MAX_ADCS] = {false, false};
//! @param CS1237_instance a global class pointer, to the specific object.
CS1237 *CS1237_instance[MAX_ADCS];
//! @param CS1237_timer the global timer pointer to the specific timer.
hw_timer_t *CS1237_timer[MAX_ADCS];
//! @param timerMux a global variable of portMUX_TYPE to synchronize between the interrupt and the main code.
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
//! @param block_timer a global flag to know, if the timer is blocked, or which instance waiting for the timer.
volatile bool block_timer[MAX_ADCS] = {false, false};

#endif  //CS1237_global


/**************************************************************************/
/*!
    @brief  this is the constructor for the CS1237 class.

            It defines and initializes the clock and data GPIOs for the ADC.
            The function waits for the chip to wake up from sleep- or power-down mode.
            Furthermore it creates one instance to know which interrupt functions have to be used.

    @param    sck
              The GPIO number of the clock line.

    @param    dout
              The GPIO number of the data line.
*/
/**************************************************************************/

CS1237::CS1237(uint8_t sck, uint8_t dout)
{
    // search for one of 2 free object spots
    for (uint8_t i = 0; i < MAX_ADCS; i++)
    {
        if (!CS1237_ISR_used[i])
        {
            CS1237_ISR_used[i] = true;
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

/**************************************************************************/
/*!
    @brief  this is the destructor for the CS1237 class.

            It stops the background reading, makes sure that the interrupt functions 
            are free to use for another object and deletes the created object of the CS1237 class,
*/
/**************************************************************************/

CS1237::~CS1237()
{
    // save, that this space is free for another object
    if(_interrupt_reading)
        end_reading();
    CS1237_ISR_used[_object_number] = false;
}

/**************************************************************************/
/*!
    @brief  this is the function to return the last value of the measured analog signal.
            
            The function waits until the reading is finished, if a reading is in progress.
            The function converts the measured data to a signed 32 bit integer and subtracts the offset, 
            if the tare function has been called before. This value is returned.

    @param  reading_
            the local variable for the read value

    @return the last value of the measured analog signal
*/
/**************************************************************************/

int32_t CS1237::reading()
{
start_sending_reading:
    // wait until the datatransmission is finished
    while (_block_value)
        ;

    noInterrupts();
    if ((_value > 0x7FFFFF) && !_block_value)
        return (_value - 0xFFFFFF - _offset);
    else if (!_block_value)
        return (_value - _offset);
    else // if in the time of the last if statment the _blockvalue has switched go to the beginning and wait for the data transmission
        goto start_sending_reading;
    interrupts();
}

/**************************************************************************/
/*!
    @brief  this is a read command for ADC-data without interrupts.
            
            This function uses delays, so it is intended for the initialization and tare process.
            This function sends 24 clock cycles to read the data. 
            If the read value is negative it is also converted to a "normal" int32_t.

    @param  reading_
            the local variable for the read value

    @return the read value of the ADC (reading_)
*/
/**************************************************************************/

int32_t CS1237::read_without_interrupt(void)
{
    int32_t reading_ = 0;
    // make sure no interrupt reading is in process, if so turn it off
    if (_interrupt_reading)
        end_reading();
    // wait for an interrupt of the ADC to happen, so that the interrupt won't collide with the data transmission
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

/**************************************************************************/
/*!
    @brief  this is a function to send clock cycles.

    @param    count_
              This variable is the used variable for the number of clock cycles
*/
/**************************************************************************/

void CS1237::send_clk_pulses(byte count_)
{
    for (uint8_t i = 0; i < count_; i++)
    {
        digitalWrite(_sck, HIGH);
        delayMicroseconds(1);
        digitalWrite(_sck, LOW);
        delayMicroseconds(1);
    }
    return;
}

/**************************************************************************/
/*!
    @brief  this is the functipn to configure the CS1237-Chip.
            
           The gain (1,2,64,128), the sample rate(10Hz,40Hz,640Hz,1280Hz), the reference voltage 
           (internal, or external) and the channel(external diferential signal, temperature sensor) 
           can be configured. This function uses delays so it needs a lot CPU time and is intended
           to be used only at the beginning. You can also read the last measured value of the ADC, 
           but this is measured with the last configuration.

    @param    write
              This is the flag, if the function should read or write the configuration register.

    @param    value   
                This is a pointer to the variable where the function should store the read value in.
    
    @param    gain   
                This is the register value of the gain, which has to be set.

    @param    speed   
                This is the register value of the sample rate, which has to be set.

    @param    channel
                This is the register value of the channel, which has to be set.

    @return The value of the configuration ragister is returned as a byte variable
*/
/**************************************************************************/

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

/**************************************************************************/
/*!
    @brief  this is the configuration function you should call to make sure the right configuration is set.
            
           This function calls the raw_configure-function three times and writes and reads the configuration register value
           to make sure the right configuration is set. If the third time the wrote and read registers are different,
           it returns a zero, to indicate, that the configuration failed.

    @param    value   
                This is a pointer to the variable where the function should store the read value in.
    
    @param    gain   
                This is the register value of the gain, which has to be set.

    @param    speed   
                This is the register value of the sample rate, which has to be set.

    @param    channel
                This is the register value of the channel, which has to be set.

    @return a flag to show, if the configuration succeed, or failed.
*/
/**************************************************************************/

bool CS1237::configure(int32_t *value, byte gain, byte speed, byte channel)
{
    bool succeed = false;
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

/**************************************************************************/
/*!
    @brief  this function is used to set the offset value of the analog signal.
            
           The value is read for a specific time to get and builds the average of the measured values.
           The average is stored in the public variabele _offset.

    @param    time
              This is time, how long this function is in progress. 
              It correlates with the quantity of the measured values for the average calculation.

    @param    _interrupt_reading_already_startet   
                This is a flag to know, if the interrupt and timer reading has already started,
                to make sure the interrupt and timer reading will go on or stop after this function.
    
    @param    offset_   
                This is the sum value for building the average.

    @param    offset_count_   
                This is the counter to know whats the quantity of the offset_-variabel.

    @param    start_time
                This is the variable to know, when to stop the function.

    @return The value of the configuration ragister is returned as a byte variable
*/
/**************************************************************************/

void CS1237::tare(uint16_t time)
{
    bool _interrupt_reading_already_startet = false;
    if (!_interrupt_reading)
        start_reading();
    else
        _interrupt_reading_already_startet = true;

    int64_t offset_ = 0;
    uint16_t offset_count_ = 0;
    uint32_t start_time = millis();
    while ((millis() - start_time) <= time)
    {
        delayMicroseconds(781); // wait for 1000000/1280 us to read a value only one time, when the sample rate is 1280Hz
        offset_ += reading();
        offset_count_++;
    }

    _offset = offset_ / offset_count_;
    if (!_interrupt_reading_already_startet)
        end_reading();
    return;
}

/**************************************************************************/
/*!
    @brief  this function will start the background reading of the ADC-values.

            In this function some flags are set, to make sure, 
            the reading won't collide with readings from other instances, 
            because the same timer is shared with every instance. 
            If no other instance is in the reading process, 
            the timer will start instant to read the ADC-value.
*/
/**************************************************************************/

void CS1237::start_reading(void)
{
    // wake the ADC up
    sleep(false);
    detachInterrupt(digitalPinToInterrupt(_dout));
    _block_value = true;
    _interrupt_reading = true;
    _clock_count = 0;
    for (uint8_t i = 0; i < MAX_ADCS; i++)
    {
        if (block_timer[i])
        {
            block_timer[_object_number] = true;
            break;
        }
    }
    if (!block_timer[_object_number])
    {
        block_timer[_object_number] = true;
        _value = 0;
        timer_init(_object_number);
    }
    return;
}

/**************************************************************************/
/*!
    @brief  this function ends the backgorund reading process.
            
           The timer and the GPIO interrupt will be disabled and the ADC is set to sleep mode.
*/
/**************************************************************************/

void CS1237::end_reading(void)
{
    timer_stop(_object_number);
    detachInterrupt(digitalPinToInterrupt(_dout));
    sleep();
    _interrupt_reading = false;
}

/**************************************************************************/
/*!
    @brief  this function sets the ADC to sleep mode or wakes the ADC up.
            
           Furthermore the sleep function makes sure, that after the return 
           the interrupt of the ADC has happend to make sure a read operation 
           won't collide with this interrupt.

    @param    sleep_
              This is a local flag to either put the device in sleep mode or wake it up.
*/
/**************************************************************************/

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
    return;
}

/**************************************************************************/
/*!
    @brief  this is the hardware interrupt function of the specific instance.
            
           If this function is called, there is a new reading available in the ADC.
           The flag to block the reading()-function is set, the hardware interrupt is detached.
           and the timer is started. Furthermore this function is attached to the RAM, to be as fast as possible.

    @param    sleep_
              This is a local flag to either put the device in sleep mode or wake it up.
*/
/**************************************************************************/

void IRAM_ATTR CS1237::instance_ISR(void)
{
    portENTER_CRITICAL_ISR(&timerMux);

    _block_value = true;
    detachInterrupt(digitalPinToInterrupt(_dout));
    _clock_count = 0;
    //make sure no other timer is running
    for (uint8_t i = 0; i < MAX_ADCS; i++)
    {
        if (block_timer[i])
        {
            block_timer[_object_number] = true;
            break;
        }
    }
    //if no other timer is running, set the block timer flag and start the timer
    if (!block_timer[_object_number])
    {
        block_timer[_object_number] = true;
        _value = 0;
        timer_init(_object_number);
    }

    portEXIT_CRITICAL_ISR(&timerMux);
}

/**************************************************************************/
/*!
    @brief  this function sets the timer for the reading process

            Furthermore this function is attached to the RAM, to be as fast as possible.


    @param    object_number_
              This is the number of the object, where the timer should be attached, 
              to make sure the right nonstatic function will be called.

*/
/**************************************************************************/

void IRAM_ATTR CS1237::timer_init(uint8_t object_number_)
{
    CS1237_timer[object_number_] = timerBegin(0, TIMER_PRESCALER, true);
    timerAttachInterrupt(CS1237_timer[object_number_], CS1237_timer_ISRfunc[object_number_], true);
    timerAlarmWrite(CS1237_timer[object_number_], CALL_TIMER_INTERRUPT, true);
    timerAlarmEnable(CS1237_timer[object_number_]);
}

/**************************************************************************/
/*!
    @brief  this function disabley the timer

            Furthermore this function is attached to the RAM, to be as fast as possible.

    @param    object_number_
              This is the number of the object, where the timer should be disabled.
*/
/**************************************************************************/

void IRAM_ATTR CS1237::timer_stop(uint8_t object_number_)
{
    timerAlarmDisable(CS1237_timer[object_number_]);
    timerDetachInterrupt(CS1237_timer[object_number_]);
    timerEnd(CS1237_timer[object_number_]);
}

/**************************************************************************/
/*!
    @brief  this function will be called every time the timer will be called.

            the function will send 24 clock pulses and read the value of the ADC.
            After the reading process is finished, the flag to block the reading 
            process is released, the hardware interrupt is attached again, the timer 
            will be released and if a another instance is waiting to get the timer, 
            the timer is set for the other instance.
            Furthermore this function is attached to the RAM, to be as fast as possible.
*/
/**************************************************************************/

void IRAM_ATTR CS1237::instance_timer_ISR(void)
{
    portENTER_CRITICAL_ISR(&timerMux);

    if (_clock_count % 2) // only every time the _SCK is HIGH read the Dout-Pin
        _value |= digitalRead(_dout) << (23 - (_clock_count - 1) / 2);

    _clock_count++;
    digitalWrite(_sck, (_clock_count % 2));

    // end the data transmission, if there were 24 clk cycles
    if (_clock_count >= 48)
    {
        _block_value = false;
        block_timer[_object_number] = false;
        attachInterrupt(digitalPinToInterrupt(_dout), CS1237_ISRfunc[_object_number], FALLING);
        timer_stop(_object_number);
        for (uint8_t i = 0; i < MAX_ADCS; i++)
        {
            if (block_timer[i])
            {
                CS1237_instance[i]->_value = 0;
                timer_init(i);
                break;
            }
        }
    }

    portEXIT_CRITICAL_ISR(&timerMux);
}

/**************************************************************************/
/*!
    @brief  this function will be called when a hardware interrupt of the first instance happens.
            
            Because the function is static, it cannot acces the nonstatic variables, 
            so a nonstatic function pointer is given and will call this nonstatic function.
            Furthermore this function is attached to the RAM, to be as fast as possible.

*/
/**************************************************************************/

void IRAM_ATTR CS1237::timer_ISR0(void) { CS1237_instance[0]->instance_timer_ISR(); }

/**************************************************************************/
/*!
    @brief  this function will be called when a hardware interrupt of the second instance happens.
            
            Because the function is static, it cannot acces the nonstatic variables, 
            so a nonstatic function pointer is given and will call this nonstatic function.
            Furthermore this function is attached to the RAM, to be as fast as possible.

*/
/**************************************************************************/

void IRAM_ATTR CS1237::timer_ISR1(void) { CS1237_instance[1]->instance_timer_ISR(); }


/**************************************************************************/
/*!
    @brief  this function will be called when a timer interrupt of the first instance happens.
            
            Because the function is static, it cannot acces the nonstatic variables, 
            so a nonstatic function pointer is given and will call this nonstatic function.
            Furthermore this function is attached to the RAM, to be as fast as possible.

*/
/**************************************************************************/

void IRAM_ATTR CS1237::ISR0(void) { CS1237_instance[0]->instance_ISR(); }

/**************************************************************************/
/*!
    @brief  this function will be called when a timer interrupt of the second instance happens.
            
            Because the function is static, it cannot acces the nonstatic variables, 
            so a nonstatic function pointer is given and will call this nonstatic function.
            Furthermore this function is attached to the RAM, to be as fast as possible.

*/
/**************************************************************************/

void IRAM_ATTR CS1237::ISR1(void) { CS1237_instance[1]->instance_ISR(); }