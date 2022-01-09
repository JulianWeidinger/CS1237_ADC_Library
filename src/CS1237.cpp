/*!
 * @file CS1237.cpp
 *
 * @mainpage ADC (CS1237) Library from Julian Weidinger
 *
 * @section intro_sec Introduction
 *
 * This is the library for the ADC "CS1237" from Chipsea. the translated datasheet can be found [here](https://github.com/rafaellcoellho/cs1237-datasheet/blob/master/cs1237_datasheet.pdf)
 * Every interface and timing information of the CS1237-Chip is taken out of this datasheet and isn't extra cited.
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
 * The is written with doxygen comments to create a html documentation of the library.
 * The style is from the doxygen [manuel from Ardafruit](https://learn.adafruit.com/the-well-automated-arduino-library/doxygen-tips)
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

//! @param CS1237_ISRfunc the global function pointer to the specific static hardware interrupt function. [Quelle](https://arduinoplusplus.wordpress.com/2021/02/05/interrupts-and-c-class-instances/)
void (*CS1237_ISRfunc[MAX_ADCS])(void) = {CS1237::ISR0, CS1237::ISR1};
//! @param CS1237_timer_ISRfunc the global function pointer to the specific static timer interrupt function.  [Quelle](https://arduinoplusplus.wordpress.com/2021/02/05/interrupts-and-c-class-instances/)
void (*CS1237_timer_ISRfunc[MAX_ADCS])(void) = {CS1237::timer_ISR0, CS1237::timer_ISR1};
//! @param CS1237_ISR_used the global flag to know, wich interrupt functions are already blocked by other instances.
bool CS1237_ISR_used[MAX_ADCS] = {false, false};
//! @param CS1237_instance the global class pointer, to the specific object. [Quelle](https://arduinoplusplus.wordpress.com/2021/02/05/interrupts-and-c-class-instances/)
CS1237 *CS1237_instance[MAX_ADCS];
//! @param CS1237_timer the global timer pointer to the specific timer. [Quelle](https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/)
hw_timer_t *CS1237_timer[MAX_ADCS];
//! @param Mux the global variable of portMUX_TYPE to synchronize between the interrupt and the main code. [Quelle](https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/)
portMUX_TYPE Mux = portMUX_INITIALIZER_UNLOCKED;
//! @param block_timer a global flag to know, if the timer is blocked, or which instance waiting for the timer.
volatile bool block_timer[MAX_ADCS] = {false, false};

#endif // CS1237_global

/**************************************************************************/
/*!
    @brief  this is the constructor for the CS1237 class.

            It defines and initializes the clock and data GPIO-pins for the ADC.
            The function waits for the chip to wake up from sleep- or power-down mode.
            Furthermore the instance is given a number, to know which interrupt functions has to be used.

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

    // wait for the chip to wake up
    sleep(false);
}

/**************************************************************************/
/*!
    @brief  this is the destructor for the CS1237 class.

            It stops the background reading, makes sure that the interrupt functions
            are free to use for another object and deletes the created object of the CS1237 class.
*/
/**************************************************************************/

CS1237::~CS1237()
{
    // end the background reading, if it is still in progess
    if (_interrupt_reading)
        end_reading();

    // save, that this space is free for another object
    CS1237_ISR_used[_object_number] = false;
}

/**************************************************************************/
/*!
    @brief  this is the function to return the last value of the measured analog signal.

            The function waits until the reading is finished, if a reading is in progress.
            The function converts the measured data to a signed 32-bit integer and subtracts the offset,
            if the tare function has been called before. This value (reading_) is returned.

    @param  reading_
            the local variable to store the measured analog signal value in

    @return the last value of the measured analog signal (reading_)
*/
/**************************************************************************/

int32_t CS1237::reading()
{
start_sending_reading:
    // wait until the datatransmission is finished
    while (_block_value)
        ;

    // return the tared value as a signed 32-bit integer
    if ((_value > 0x7FFFFF) && !_block_value)
        return (_value - 0xFFFFFF - _offset);
    else if (!_block_value)
        return (_value - _offset);

    // if in the time of the last if-statment the _blockvalue has switched go to the beginning and wait for the data transmission
    else
        goto start_sending_reading;
}

/**************************************************************************/
/*!
    @brief  this is the function to read the measured analog signal without interrupts.

            This function uses delays, so it is intended for the initialization and tare process.
            This function sends 24 clock cycles to read the data.
            If the read value is negative it is also converted to a signed 32-bit integer.
            The offset is also taken in account.

    @param  reading_
            the local variable for the read value

    @return the read value of measured analog signal (reading_)
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

    // start the data transmission.
    for (uint8_t i = 0; i < 24; i++)
    {
        digitalWrite(_sck, HIGH);
        delayMicroseconds(1);
        reading_ |= digitalRead(_dout) << (23 - i);
        digitalWrite(_sck, LOW);
        delayMicroseconds(1);
    }
    if (reading_ > 0x7FFFFF)
        reading_ -= 0xFFFFFF;
    return reading_;
}

/**************************************************************************/
/*!
    @brief  this is a function to send a specific number of clock cycles.

    @param    count_
              This is the variable for the number of clock cycles to send
*/
/**************************************************************************/

void CS1237::send_clk_pulses(byte count_)
{
    // send the clock cycles
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
    @brief  this is the function to configure the CS1237-Chip.

           The gain (1,2,64,128), the sample rate(10Hz,40Hz,640Hz,1280Hz), the reference voltage
           (internal, or external) and the channel(external diferential signal, temperature sensor)
           can be configured. This function uses delays so it needs a lot CPU time and is intended
           to be used only at the beginning. You can also read the last measured value of the ADC,
           but this is measured with the last configuration. 
           The register configutation can be looked up in the [datasheet](https://github.com/rafaellcoellho/cs1237-datasheet/blob/master/cs1237_datasheet.pdf)

    @param    write
              This is the flag, if the function should read or write the configuration register.

    @param    value
                This is a pointer to the variable where the function should store the measured analog signal.

    @param    gain
                This is the register value of the gain, which has to be set.

    @param    speed
                This is the register value of the sample rate, which has to be set.

    @param    channel
                This is the register value of the channel, which has to be set.

    @param    register_value
                This is the value of the register, where all "handed over variables" are combined to get the full register values

                Depending on the write-flag this is the sent, or the read register-value.

    @return The value of the configuration register is returned as a byte variable. 
    
                Depending on the write-flag this is the sent, or the read register-value.
*/
/**************************************************************************/

byte CS1237::raw_configure(bool write, int32_t *value, byte gain, byte speed, byte channel)
{

    byte register_value;
    value[0] = read_without_interrupt();

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
    
    // put the ADC-pins of the MCU in INPUT_Pullup mode again and return the register value
    pinMode(_dout, INPUT_PULLUP);
    return register_value;
}

/**************************************************************************/
/*!
    @brief  this function should be called to make sure, the right configuration is set.

           This function calls the raw_configure-function three times and writes and reads the configuration register value
           to make sure the right configuration is set. If the third time the wrote and read registers are different,
           it returns a "false", to indicate, that the configuration failed.

    @param    value
                This is a pointer to the variable, where the function should store the read value.

    @param    gain
                This is the register value of the gain, which has to be set.

    @param    speed
                This is the register value of the sample rate, which has to be set.

    @param    channel
                This is the register value of the channel, which has to be set.

    @return the flag to show, if the configuration succeeded, or failed.
*/
/**************************************************************************/

bool CS1237::configure(int32_t *value, byte gain, byte speed, byte channel)
{
    bool succeed = false;

    // try to configure the ADC for at least three times to set an error
    byte try_count = 3;
    byte conf_write = 0;
    byte conf_read = 0;
    for (uint8_t i = 0; (i < try_count) && !succeed; i++)
    {
        conf_write = raw_configure(true, value, gain, speed, channel);
        conf_read = raw_configure(false, value, gain, speed, channel);
        succeed = true;

        // if the configuration is wrong the for loop will start over again.
        if (conf_write != conf_read)
            succeed = false;
    }
    return succeed;
}

/**************************************************************************/
/*!
    @brief  this function is used, to set the offset value of the analog signal.

           The measured analog signal is read for a specific time and added,
           to build the average of the measured values.
           The average is stored in the public variabele _offset.

    @param    time_
              This  variable sets, how long this function is in progress.
              
              It correlates with the quantity of the measured values for the average calculation.

    @param    _interrupt_reading_already_startet
                This is a flag to know, if the background reading has already started,
                to make sure the background reading will go on or stop after this function.

    @param    offset_
                This is the sum value for building the average.

    @param    offset_count_
                This is the counter, to know whats the quantity of the offset_-variabel.

    @param    start_time
                This is the time variable to know, when to stop the function.

    @return The value of the configuration register is returned as a byte variable
*/
/**************************************************************************/

void CS1237::tare(uint16_t time_)
{
    //start background reading, if it hasn't already started
    bool _interrupt_reading_already_startet = false;
    if (!_interrupt_reading)
        start_reading();
    else
        _interrupt_reading_already_startet = true;

    //start to add the read value, to calculate the average later on
    int64_t offset_ = 0;
    uint16_t offset_count_ = 0;
    uint32_t start_time = millis();
    while ((millis() - start_time) <= time_)
    {
        delayMicroseconds(781); // wait for 1000000/1280 us to read a value only one time, when the sample rate is 1280Hz
        offset_ += reading();
        offset_count_++;
    }

    //calculate the offset and end the background reading, if the _interrupt_reading_already_startet-flag is set to false
    _offset += offset_ / offset_count_;
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
            the timer will start instant to read the last ADC-value.
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

    // make sure no other timer is running
    for (uint8_t i = 0; i < MAX_ADCS; i++)
    {
        if (block_timer[i])
        {
            block_timer[_object_number] = true;
            break;
        }
    }

    // if no other timer is running, set the block timer flag and start the timer
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
           the interrupt of the ADC has happend, to make sure a read operation
           won't collide with this interrupt, but only if the sleep_-flag is set to false.

    @param    sleep_
              This is a local flag to either put the device in sleep mode or wake it up 
              and wait for the first interrupt.
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

        // Wait for an one interrupt to happen, then the chip is surely awake
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
           The flag to block the reading()-function is set, the hardware interrupt is detached
           and the timer is started. Furthermore this function is attached to the RAM, to be as fast as possible.

    @param    sleep_
              This is a local flag to either put the device in sleep mode or wake it up.
*/
/**************************************************************************/

void IRAM_ATTR CS1237::instance_ISR(void)
{
    portENTER_CRITICAL_ISR(&Mux);

    _block_value = true;
    detachInterrupt(digitalPinToInterrupt(_dout));
    _clock_count = 0;

    // make sure no other timer is running
    for (uint8_t i = 0; i < MAX_ADCS; i++)
    {
        if (block_timer[i])
        {
            block_timer[_object_number] = true;
            break;
        }
    }

    // if no other timer is running, set the block timer flag and start the timer
    if (!block_timer[_object_number])
    {
        block_timer[_object_number] = true;
        _value = 0;
        timer_init(_object_number);
    }

    portEXIT_CRITICAL_ISR(&Mux);
}

/**************************************************************************/
/*!
    @brief  this function sets the timer for the reading process

            Furthermore this function is attached to the RAM, to be as fast as possible.
            [Quelle](https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/)

    @param    object_number_
              This is the number of the object, where the timer should be attached,
              to make sure the correct nonstatic function will be called.

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
    @brief  this function disables the timer

            Furthermore this function is attached to the RAM, to be as fast as possible.
            [Quelle](https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/)

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
    @brief  this function will be called every time the timer interrrupt will be called.

            the function will send 24 clock pulses and read the value of the ADC.
            After the reading process is finished, the flag to block the reading
            process is released, the hardware interrupt is attached again, the timer
            is released and if a another instance is waiting to get the timer,
            the timer is set for the other instance.
            Furthermore this function is attached to the RAM, to be as fast as possible.
*/
/**************************************************************************/

void IRAM_ATTR CS1237::instance_timer_ISR(void)
{
    portENTER_CRITICAL_ISR(&Mux);

    // read the dout-Pin only every time the _SCK is HIGH 
    if (_clock_count % 2) 
        _value |= digitalRead(_dout) << (23 - (_clock_count - 1) / 2);
    
    //increment the clock count and toggle the clock line.
    _clock_count++;
    digitalWrite(_sck, (_clock_count % 2));

    // end the data transmission, if there were 24 clk cycles (2*24=48 toggles).
    if (_clock_count >= 48)
    {
        _block_value = false;
        block_timer[_object_number] = false;
        attachInterrupt(digitalPinToInterrupt(_dout), CS1237_ISRfunc[_object_number], FALLING);
        timer_stop(_object_number);

        //if there is another instance waiting for the timer to start the datatransmission, initialize the timer for the other object.
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

    portEXIT_CRITICAL_ISR(&Mux);
}

/**************************************************************************/
/*!
    @brief  this function will be called, when a hardware interrupt of the first instance happens.

            Because the function is static, it can not access the nonstatic variables,
            so a nonstatic function pointer is given and will call this nonstatic function.
            Furthermore this function is attached to the RAM, to be as fast as possible.
            [Quelle_1](https://arduinoplusplus.wordpress.com/2021/02/05/interrupts-and-c-class-instances/)
            [Quelle_2](https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/)

*/
/**************************************************************************/

void IRAM_ATTR CS1237::timer_ISR0(void) { CS1237_instance[0]->instance_timer_ISR(); }

/**************************************************************************/
/*!
    @brief  this function will be called when a hardware interrupt of the second instance happens.

            Because the function is static, it cannot acces the nonstatic variables,
            so a nonstatic function pointer is given and will call this nonstatic function.
            Furthermore this function is attached to the RAM, to be as fast as possible.
            [Quelle_1](https://arduinoplusplus.wordpress.com/2021/02/05/interrupts-and-c-class-instances/)
            [Quelle_2](https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/)

*/
/**************************************************************************/

void IRAM_ATTR CS1237::timer_ISR1(void) { CS1237_instance[1]->instance_timer_ISR(); }

/**************************************************************************/
/*!
    @brief  this function will be called when a timer interrupt of the first instance happens.

            Because the function is static, it cannot acces the nonstatic variables,
            so a nonstatic function pointer is given and will call this nonstatic function.
            Furthermore this function is attached to the RAM, to be as fast as possible.
            [Quelle_1](https://arduinoplusplus.wordpress.com/2021/02/05/interrupts-and-c-class-instances/)
            [Quelle_2](https://techtutorialsx.com/2017/09/30/esp32-arduino-external-interrupts/)
            

*/
/**************************************************************************/

void IRAM_ATTR CS1237::ISR0(void) { CS1237_instance[0]->instance_ISR(); }

/**************************************************************************/
/*!
    @brief  this function will be called when a timer interrupt of the second instance happens.

            Because the function is static, it cannot acces the nonstatic variables,
            so a nonstatic function pointer is given and will call this nonstatic function.
            Furthermore this function is attached to the RAM, to be as fast as possible.
            [Quelle_1](https://arduinoplusplus.wordpress.com/2021/02/05/interrupts-and-c-class-instances/)
            [Quelle_2](https://techtutorialsx.com/2017/09/30/esp32-arduino-external-interrupts/)

*/
/**************************************************************************/

void IRAM_ATTR CS1237::ISR1(void) { CS1237_instance[1]->instance_ISR(); }