/*!
 * @file CS1237.cpp
 *
 * @mainpage ADC (CS1237) Library from Julian Weidinger
 *
 * @section intro_sec Introduction
 *
 * As part of my bachelor thesis, I developed a device to measure the hardness differences in the snowpack. 
 * This can be attached to a ski touring probe. To take a measurement, the probe is inserted into the snow. 
 * The result is a hardness profile of the snowpack over the depth. The hardness measurement was carried out 
 * with the help of two load cells, two instrument amplifiers and two AD converters integrated in the microcontroller. 
 * Errors occurred because  * the analog cables of the load cells were probably picking up electromagnetic interference. 
 * For this reason I selected an AD converter chip (CS1237) with an integrated amplifier and created a circuit board. 
 * The translated datasheet can be found [here](https://github.com/rafaellcoellho/cs1237-datasheet/blob/master/cs1237_datasheet.pdf).
 * Every interface and timing information of the CS1237-Chip is taken out of this datasheet and isn't extra cited.
 * The standard ADC-coverter (HX711) has a too slow sample rate, to use in my measurement device and the chips from other 
 * manufacturers ware too expensive. I wrote the following Arduino library as part of a study project "data communication" 
 * to create the digital interface between the ADC chip and the ESP32-microcontroller. breakoutboard for 
 * the ADC-chip was developed by myself, because I couldn't find a breakout to buy. In the following pictures you can see circuit diagram, 
 * the front and back schematic and a 3D-model of the breakoutboard: 
 * <div>
 * <img src="..\images\circuit_diagram.png" alt="circuit_diagram couldn't be load!" style="width:22.3%;" title="Bildbeschreibung">
 * <img src="..\images\schematic_front.png" alt="schematic_front couldn't be load!" style="width:24%;">
 * <img src="..\images\schematic_back.png" alt="schematic_back couldn't be load!" style="width:24%;">
 * <img src="..\images\breakout_board_3D.png" alt="breakout_board_3D couldn't be load!" style="width:26.3%;">
 * </div>
 * images of the circuit diagram, the front and back schematic and a 3D-model of the breakoutboard
 * 
 * @section describtion Theme of the project
 * 
 * The aim of the work is, to write a library, to configure the ADC and get the measured analog signal. 
 * The best solution would be an interrupt driven library, where you can connect two ADCs to the MCU and read it parralell in the background.
 * Another nice feature would be to connect multiple ADCs to one clock line and read the ADCs synchronous.
 * 
 * 
 * @section description_ADC Description of the ADC (CS1237)
 * 
 * The gain (1,2,64,128), the sample rate(10Hz,40Hz,640Hz,1280Hz), the reference voltage (internal, or external)
 * and the channel(external diferential signal, temperature sensor) of the ADC can be independent configured.
 * The interface of the ADC chip has a unidirectional clock line “CLK” and a bidirectional data line "DOUT/DIN". The clock-
 * line is used for the clock signal and for switching the chip on and off (sleep mode). The data line is used on the one 
 * hand to transmit the AD measurement and on the other hand to configure the chip. In addition, an interrupt signal is 
 * outputted via the data line for 26,13 us, if a new measurement is available, depending on the configured measurement 
 * frequency. The interrupt of the ADC is also outputed during digital data transmission. It must therefore 
 * be ensured, that the data transmission is completed at this point, otherwise the result will be incorrect. In the following 
 * image a oscilloscope capture of the finished library and three datatransmission processes with the interrupts of the ADC chip
 * can be seen:
 * 
 * <img src="..\images\Oszi_measurement.jpg" alt="circuit_diagram couldn't be load!" style="width:100%;"> Diagramm of an oscilloscope measurement while a data transmission process
 * 
 * @section description_library Description of the setup, the code and the attemps to get it working
 * 
 * The library is written in Visual Studio Code in C++. For flashing the ESP32 the Arduino extension from Microsoft was used.
 * 
 * My first idea was to connect several chips to one clock line in order to save GPIOs. Unfortunately, after several attempts, 
 * this was impossible because the internal clock of the chips are not synchronized and the output interrupt occurrs at different 
 * times. This is particularly critical with the desired output frequency of 1280 Hz. That's why I decided to write a C ++ library, 
 * in which several instances with different clock lines can be created. The number of ADCs to connect to one MCU is limited to two. 
 * The reason is, that the timing is critical especially, when the sample rate is set to 1280. The library could be written to allow 
 * more than two instances running in parallel, if the high sample rates are blocked, but that is a future work.
 * 
 * This current library is written with hardware and timer interrupt service routines, so it can run in the background, while the main code is running.
 * And the last measured analog value, which is updated in the background, can be called up anytime. In the next picture you can see a screenshot 
 * of the Arduino-Serial-Plotter, while the example code runs on the ESP32 with two ADCs. There is some noise in the signal. This is probably 
 * due to the long unshielded cables or due to the PCB layout. The signal is periodic so, in a future work it could be filtered 
 * with fourier transformation: The setup can be seen in the next picture. For the differential analog signal were two 10kg loadcells used one is 
 * screwed in the negativ and the other in the positive direction to show, that the ADC-library woks in both directions: 
 * 
 * <img src="..\images\serial_plotter.png" alt="serial_plotter image couldn't be load!" style="width:100%;">  
 * A screenshot of the serial plotter, while the example code runs on the ESP32 with two ADCs
 * 
 * <img src="..\images\test_setup_schematic.svg" alt="schematic_back couldn't be load!" style="width:100%;">
 * A schematic sketch to show the test setup
 * <div>
 * <img src="..\images\test_setup.jpg" alt="circuit_diagram couldn't be load!" style="width:49%;" title="Bildbeschreibung">
 * <img src="..\images\3D_printed_scale_with_loadcell.jpg" alt="3D_printed_scale_with_loadcell couldn't be load!" style="width:49%;">
 * </div>
 * fotos of the test setup with the two scales, the loadcells, the ADCs and the NodeMCU
 * 
 * The next picture shows the block diagram of the example code. The detailed describtions and graphs of the class functions are showed on the class page.
 *  <img src="..\images\block_diagram_of_example_code.svg" alt="block diagram of the example code couldn't be load!" style="width:99%;">
 * block diagram of the example code
 * 
 * @section dependencies Dependencies
 *
 * This library was only tested on the ESP32 DEV_KIT_V4
 *
 *  - At this time it is only allowed to connect two ADCs with one clock and one data line each.
 *  - you could use nearly any GPIO for the clock lines and any GPIO with an interrupt function for the Data line.
 *  - to the ADC were two 10kg loadcells connected.
 *
 * @section author Author
 *
 * Written by Julian Weidinger (matriculation-number : 11259117) for an university Project at the University of applied science.
 * 
 * The source code and one example is available on [my github page](https://github.com/JulianWeidinger/CS1237_ADC_Library)
 * 
 * The code is written with doxygen comments to create a html documentation of the library.
 * The style is from the doxygen [manuel from Ardafruit](https://learn.adafruit.com/the-well-automated-arduino-library/doxygen-tips)
 *
 * 
 *  @section sources Sources
 * 
 * 1. datasheet: https://github.com/rafaellcoellho/cs1237-datasheet/blob/master/cs1237_datasheet.pdf
 * 
 * 2. external hardware interrupt: https://techtutorialsx.com/2017/09/30/esp32-arduino-external-interrupts/
 * 
 * 3. timer interrupts: https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
 * 
 * 4. attach interrupts to class instances: https://arduinoplusplus.wordpress.com/2021/02/05/interrupts-and-c-class-instances/
 * 
 * 5. doxygen documentation guide: https://learn.adafruit.com/the-well-automated-arduino-library/doxygen-tips
 * 
 * @date 09.01.2022 \n \n
 */

#include "CS1237.h"

//! the global function pointer to the specific static hardware interrupt function. [Quelle](https://arduinoplusplus.wordpress.com/2021/02/05/interrupts-and-c-class-instances/)
void (*CS1237_ISRfunc[MAX_ADCS])(void) = {CS1237::ISR0, CS1237::ISR1};
//! the global function pointer to the specific static timer interrupt function.  [Quelle](https://arduinoplusplus.wordpress.com/2021/02/05/interrupts-and-c-class-instances/)
void (*CS1237_timer_ISRfunc[MAX_ADCS])(void) = {CS1237::timer_ISR0, CS1237::timer_ISR1};
//!  the global flag to know, wich interrupt functions are already blocked by other instances.
bool CS1237_ISR_used[MAX_ADCS] = {false, false};
//! the global class pointer, to the specific object. [Quelle](https://arduinoplusplus.wordpress.com/2021/02/05/interrupts-and-c-class-instances/)
CS1237 *CS1237_instance[MAX_ADCS];
//! the global timer pointer to the specific timer. [Quelle](https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/)
hw_timer_t *CS1237_timer[MAX_ADCS];
//! the global variable of portMUX_TYPE to synchronize between the interrupt and the main code. [Quelle](https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/)
portMUX_TYPE Mux = portMUX_INITIALIZER_UNLOCKED;
//! a global flag to know, if the timer is blocked, or which instance waiting for the timer.
volatile bool block_timer[MAX_ADCS] = {false, false};

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
CS1237::~CS1237()
{
    // end the background reading, if it is still in progess
    if (_interrupt_reading)
        end_reading();

    // save, that this space is free for another object
    CS1237_ISR_used[_object_number] = false;
}

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
void CS1237::end_reading(void)
{
    timer_stop(_object_number);
    detachInterrupt(digitalPinToInterrupt(_dout));
    sleep();
    _interrupt_reading = false;
}

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
void IRAM_ATTR CS1237::timer_init(uint8_t object_number_)
{
    CS1237_timer[object_number_] = timerBegin(0, TIMER_PRESCALER, true);
    timerAttachInterrupt(CS1237_timer[object_number_], CS1237_timer_ISRfunc[object_number_], true);
    timerAlarmWrite(CS1237_timer[object_number_], CALL_TIMER_INTERRUPT, true);
    timerAlarmEnable(CS1237_timer[object_number_]);
}

/**************************************************************************/
void IRAM_ATTR CS1237::timer_stop(uint8_t object_number_)
{
    timerAlarmDisable(CS1237_timer[object_number_]);
    timerDetachInterrupt(CS1237_timer[object_number_]);
    timerEnd(CS1237_timer[object_number_]);
}

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
void IRAM_ATTR CS1237::timer_ISR0(void) { CS1237_instance[0]->instance_timer_ISR(); }

/**************************************************************************/
void IRAM_ATTR CS1237::timer_ISR1(void) { CS1237_instance[1]->instance_timer_ISR(); }

/**************************************************************************/
void IRAM_ATTR CS1237::ISR0(void) { CS1237_instance[0]->instance_ISR(); }

/**************************************************************************/
void IRAM_ATTR CS1237::ISR1(void) { CS1237_instance[1]->instance_ISR(); }