/*!
 * @file CS1237.h
 *
 * This is a library for the CS1237-IC to convert the analog readings
 * of a diferential signal in 24bit digital numbers
 *
 *
 * The Chip communicates through a 2-wire SPI interface SCLK, DRDY/DOUT,
 * two GPIO-pins are required to interface with the CS1237-IC.
 *
 * Written by Julian Weidinger for an University Project.
 */

#ifndef CS1237_h
#define CS1237_h

// include the standard arduino library
#include <Arduino.h>

// read or write the configuration register
#define READ_ADC_CONFIG 0x56  ///< command to read the configuration register.
#define WRITE_ADC_CONFIG 0x65 ///< command to write to the the configuration register.

//! library specific definitions
#define MAX_ADCS 2                ///< maximum usable ADCs
#define TIMER_PRESCALER 80        ///< 12.5ns*80 = 1000ns, so every 1000ns the timer will be count up.
#define CALL_TIMER_INTERRUPT 6    ///< so after 6 counts of the timer the interrupt function will be called. --> so every 6us
#define WAITING_TIME_TO_SLEEP 100 ///< [us] the time the chip needs to go to sleep, in the datasheet its said 100us

// configuration commands
#define SPEED_10 0b00000000   ///< The register value, if the sample rate is set to 10 Hz.
#define SPEED_40 0b00010000   ///< The register value, if the sample rate is set to 40 Hz.
#define SPEED_640 0b00100000  ///< The register value, if the sample rate is set to 640 Hz.
#define SPEED_1280 0b00110000 ///< The register value, if the sample rate is set to 1280 Hz.

#define PGA_1 0b00000000   ///< The register value, if the gain is set to 1.
#define PGA_2 0b00000100   ///< The register value, if the gain is set to 2.
#define PGA_64 0b00001000  ///< The register value, if the gain is set to 64.
#define PGA_128 0b00001100 ///< The register value, if the gain is set to 128.

#define CHANNEL_A 0b00000000    ///< The register value, if the channel A is selected.
#define CHANNEL_Temp 0b00000010 ///< The register value, if the internal temperature channel is selected.

/*!
 * @brief CS1237 class
 *          \n \n The class is intended to configure and read the values of the ADC,
 *          \n especially to read it in the background with hardware and timer interrupts,
 *          \n while the main loop is running and other actions are in progress.
 *          \n The number of instances is limited to two, because the timing
 *          \n of the data transmission is critical, when the high sample rate is set,
 *          \n if you have more than two instances.
 */

class CS1237
{
private:
    // variables for ADC configuration

    //! private variable of the clock-pin
    uint8_t _sck;
    //! private variable of the dout-pin of the first ADC
    uint8_t _dout;
    //! private variabel of the sleep state of the chip
    uint8_t _sleep;

    // variables for the object definition
    //! private variable to identify the object and the dedicated interrupt functions
    uint8_t _object_number;

    // variables for the data transfer process and interrupts //https://arduinoplusplus.wordpress.com/2021/02/05/interrupts-and-c-class-instances/
    
    //! private variable to know, when a data_transfer is running
    volatile bool _block_value;
    //! private variable to count the clocks, which ara send to the ADC
    volatile uint8_t _clock_count;
    //! private variable for the measured value of the analog signal
    volatile int32_t _value;
    //! private flag to know, if there is an interrupt reading in process or not
    bool _interrupt_reading = false;

public:
    // #########################################################################
    // ########################### public variables ############################
    // #########################################################################

    //! public variable for the offset of the measured analog signal
    int32_t _offset = 0;

    // #########################################################################
    // ########################### public functions ############################
    // #########################################################################

    /**************************************************************************/
    /*!
        @brief  This is the constructor for the CS1237 class.
                \n \n It defines and initializes the clock and data GPIO-pins for the ADC.
                \n The function waits for the chip to wake up from sleep- or power-down mode.
                \n Furthermore the instance is given a number, to know which interrupt functions has to be used.

        @param    sck
                  The GPIO number of the clock line.

        @param    dout
                  The GPIO number of the data line.
    */
    CS1237(uint8_t sck, uint8_t dout);

    /**************************************************************************/
    /*!
        @brief  This is the destructor for the CS1237 class.
                \n \n It stops the background reading, makes sure that the interrupt functions
                \n are free to use for another object and deletes the created object of the CS1237 class.
    */
    ~CS1237();

    /**************************************************************************/
    /*!
        @brief  This is the function to return the last value of the measured analog signal.
                \n \n The function waits until the reading is finished, if a reading is in progress.
                \n The function converts the measured data to a signed 32-bit integer and subtracts the offset,
                \n if the tare function has been called before. This value (reading_) is returned.

        @param  reading_
                the local variable to store the measured analog signal value in

        @return the last value of the measured analog signal (reading_)
    */
    int32_t reading();

    /**************************************************************************/
    /*!
        @brief  This is a function to send a specific number of clock cycles.

        @param    count_
                  This is the variable for the number of clock cycles to send
    */
    void send_clk_pulses(byte count_);

    /**************************************************************************/
    /*!
        @brief  This is the function to configure the CS1237-Chip.
                \n \n The gain (1,2,64,128), the sample rate(10Hz,40Hz,640Hz,1280Hz), the reference voltage
               \n (internal, or external) and the channel(external diferential signal, temperature sensor)
               \n can be configured. This function uses delays so it needs a lot CPU time and is intended
               \n to be used only at the beginning. You can also read the last measured value of the ADC,
               \n but this is measured with the last configuration.
               \n The register configutation can be looked up in the [datasheet](https://github.com/rafaellcoellho/cs1237-datasheet/blob/master/cs1237_datasheet.pdf).

        @param    write
                  This is the flag, if the function should read or write the configuration register.

        @param    result
                    This is a pointer to the variable where the function should store the measured analog signal.

        @param    gain
                    This is the register value of the gain, which has to be set.

        @param    speed
                    This is the register value of the sample rate, which has to be set.

        @param    channel
                    This is the register value of the channel, which has to be set.

        @return The value of the configuration register is returned as a byte variable.
                    \n Depending on the write-flag this is the sent, or the read register-value.
    */
    byte raw_configure(bool write, int32_t *result = NULL, byte gain = PGA_128, byte speed = SPEED_1280, byte channel = CHANNEL_A);

    /**************************************************************************/
    /*!
        @brief  This function should be called to make sure, the right configuration is set.
               \n  \n This function calls the raw_configure-function three times and writes and reads the configuration register value
               \n to make sure the right configuration is set. If the third time the wrote and read registers are different,
               \n it returns a "false", to indicate, that the configuration failed.

        @param    result
                    This is a pointer to the variable, where the function should store the read value.

        @param    gain
                    This is the register value of the gain, which has to be set.

        @param    speed
                    This is the register value of the sample rate, which has to be set.

        @param    channel
                    This is the register value of the channel, which has to be set.

        @return the flag to show, if the configuration succeeded, or failed.
    */
    bool configure(int32_t *result, byte gain = PGA_128, byte speed = SPEED_1280, byte channel = CHANNEL_A);

    /**************************************************************************/
    /*!
        @brief  This function is used, to set the offset value of the analog signal.
                \n \n The measured analog signal is read for a specific time and added,
               \n to build the average of the measured values.
               \n The average is stored in the public variabele _offset.

        @param    time_
                  This  variable sets, how long this function is in progress.
                  \n \n It correlates with the quantity of the measured values for the average calculation.
        @return The value of the configuration register is returned as a byte variable
    */
    void tare(uint16_t time_);

    /**************************************************************************/
    /*!
        @brief  This is the function to read the measured analog signal without interrupts.
                \n \n This function uses delays, so it is intended for the initialization and tare process.
                \n This function sends 24 clock cycles to read the data.
                \n If the read value is negative it is also converted to a signed 32-bit integer.
                \n The offset is also taken in account.

        @return the read value of measured analog signal
    */
    int32_t read_without_interrupt(void);

    /**************************************************************************/
    /*!
        @brief  This function will start the background reading of the ADC-values.
                \n \n In this function some flags are set, to make sure,
                \n the reading won't collide with readings from other instances,
                \n because the same timer is shared with every instance.
                \n If no other instance is in the reading process,
                \n the timer will start instant to read the last ADC-value.
    */
    void start_reading(void);

    /**************************************************************************/
    /*!
        @brief  This function ends the backgorund reading process.
                \n \n The timer and the GPIO interrupt will be disabled
                \n and the ADC is set to sleep mode.
    */
    void end_reading(void);

    /**************************************************************************/
    /*!
        @brief  This function sets the ADC to sleep mode or wakes the ADC up.
                \n \n Furthermore the sleep function makes sure, that after the return
               \n the interrupt of the ADC has happend, to make sure a read operation
               \n won't collide with this interrupt, but only if the sleep_-flag is set to false.

        @param    sleep_
                  This is a local flag to either put the device in sleep mode or wake it up
                  and wait for the first interrupt.
    */
    void sleep(bool sleep_ = true);

    // #########################################################################
    // ##################### interrupt and timer functions #####################
    // #########################################################################

    /**************************************************************************/
    /*!
        @brief  This is the hardware interrupt function of the specific instance.
                \n \n If this function is called, there is a new reading available in the ADC.
               \n The flag to block the reading()-function is set, the hardware interrupt is detached
               \n and the timer is started. Furthermore this function is attached to the RAM, to be as fast as possible.
    */
    void IRAM_ATTR instance_ISR(void);

    /**************************************************************************/
    /*!
        @brief  This function will be called every time the timer interrrupt will be called.
                \n \n the function will send 24 clock pulses and read the value of the ADC.
                \n After the reading process is finished, the flag to block the reading
                \n process is released, the hardware interrupt is attached again, the timer
                \n is released and if a another instance is waiting to get the timer,
                \n the timer is set for the other instance.
                \n Furthermore this function is attached to the RAM, to be as fast as possible.
    */
    void IRAM_ATTR instance_timer_ISR(void);

    /**************************************************************************/
    /*!
        @brief  This function sets the timer for the reading process
                \n \n Furthermore this function is attached to the RAM, to be as fast as possible.
                \n \n [Quelle](https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/)

        @param    object_number_
                  This is the number of the object, where the timer should be attached,
                  \n to make sure the correct nonstatic function will be called.
    */
    /**************************************************************************/
    void IRAM_ATTR timer_init(uint8_t object_number_);

    /**************************************************************************/
    /*!
        @brief  This function disables the timer
                \n \n Furthermore this function is attached to the RAM, to be as fast as possible.
                \n \n [Quelle](https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/)

        @param    object_number_
                  This is the number of the object, where the timer should be disabled.
    */
    void IRAM_ATTR timer_stop(uint8_t object_number_);

    /**************************************************************************/
    /*!
        @brief  This function will be called, when a hardware interrupt of the first instance happens.
                \n \n Because the function is static, it can not access the nonstatic variables,
                \n so a nonstatic function pointer is given and will call this nonstatic function.
                \n Furthermore this function is attached to the RAM, to be as fast as possible.
                \n \n [Quelle_1](https://arduinoplusplus.wordpress.com/2021/02/05/interrupts-and-c-class-instances/)
                \n [Quelle_2](https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/)
    */
    static void IRAM_ATTR timer_ISR0(void);

    /**************************************************************************/
    /*!
        @brief  This function will be called when a hardware interrupt of the second instance happens.
                \n \n Because the function is static, it cannot acces the nonstatic variables,
                \n so a nonstatic function pointer is given and will call this nonstatic function.
                \n Furthermore this function is attached to the RAM, to be as fast as possible.
                \n \n  [Quelle_1](https://arduinoplusplus.wordpress.com/2021/02/05/interrupts-and-c-class-instances/)
                \n [Quelle_2](https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/)
    */
    static void IRAM_ATTR timer_ISR1(void);

    /**************************************************************************/
    /*!
        @brief  This function will be called when a timer interrupt of the first instance happens.
                \n \n Because the function is static, it cannot acces the nonstatic variables,
                \n so a nonstatic function pointer is given and will call this nonstatic function.
                \n Furthermore this function is attached to the RAM, to be as fast as possible.
                \n \n [Quelle_1](https://arduinoplusplus.wordpress.com/2021/02/05/interrupts-and-c-class-instances/)
                \n [Quelle_2](https://techtutorialsx.com/2017/09/30/esp32-arduino-external-interrupts/)
    */
    static void IRAM_ATTR ISR0(void);

    /**************************************************************************/
    /*!
        @brief  This function will be called when a timer interrupt of the second instance happens.
                \n \n Because the function is static, it cannot acces the nonstatic variables,
                \n so a nonstatic function pointer is given and will call this nonstatic function.
                \n Furthermore this function is attached to the RAM, to be as fast as possible.
                \n \n [Quelle_1](https://arduinoplusplus.wordpress.com/2021/02/05/interrupts-and-c-class-instances/)
                \n [Quelle_2](https://techtutorialsx.com/2017/09/30/esp32-arduino-external-interrupts/)
    */
    static void IRAM_ATTR ISR1(void);
};

#endif // CS1237