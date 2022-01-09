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

//include the standart arduino library
#include <Arduino.h>

//read or write the configuration register
#define READ_ADC_CONFIG 0x56        ///< command to read the configuration register.
#define WRITE_ADC_CONFIG 0x65       ///< command to write to the the configuration register.

//! library specific definitions
#define MAX_ADCS 2                      ///< maximum usable ADCs
#define TIMER_PRESCALER 80              ///< 12.5ns*80 = 1000ns, so every 1000ns the timer will be count up.
#define CALL_TIMER_INTERRUPT 6          ///< so after 6 counts of the timer the interrupt function will be called. --> so every 6us
#define WAITING_TIME_TO_SLEEP 100       ///< [us] the time the chip needs to go to sleep, in the datasheet its said 100us


// configuration commands
#define SPEED_10 0b00000000         ///< The register value, if the sample rate is set to 10 Hz.
#define SPEED_40 0b00010000         ///< The register value, if the sample rate is set to 40 Hz.
#define SPEED_640 0b00100000        ///< The register value, if the sample rate is set to 640 Hz.
#define SPEED_1280 0b00110000       ///< The register value, if the sample rate is set to 1280 Hz.

#define PGA_1 0b00000000            ///< The register value, if the gain is set to 1.
#define PGA_2 0b00000100            ///< The register value, if the gain is set to 2.
#define PGA_64 0b00001000           ///< The register value, if the gain is set to 64.
#define PGA_128 0b00001100          ///< The register value, if the gain is set to 128.

#define CHANNEL_A 0b00000000        ///< The register value, if the channel A is selected.
#define CHANNEL_Temp 0b00000010     ///< The register value, if the internal temperature channel is selected.



/*!
 * @brief CS1237 class
 *  
 *          The class is intended to configure and read the values of the ADC, 
 *          especially to read it in the background with hardware and timer interrupts, 
 *          while the main loop is running and other actions are in progress.
 *          The number of instances is limited to two, because ther are the timing
 *          of the data transmission, when you use the high sample rate is set is critical, 
 *          if you have more than two instances.
 */

class CS1237
{
private:

    //variables for ADC configuration
    //! @param _sck private variable of the clock-pin
    uint8_t _sck;
    //! @param _dout private variable of the dout-pin of the first ADC
    uint8_t _dout;
    //! @param sleep private variabel of the sleep state of the chip
    uint8_t _sleep;

    //variables for the object definition
    //! @param _object_number private variable to identify the object and the dedicated interrupt functions
    uint8_t _object_number;

    //variables for the data transfer process and interrupts //https://arduinoplusplus.wordpress.com/2021/02/05/interrupts-and-c-class-instances/
    //! @param _block_value private variable to know, when a data_transfer is running
    volatile bool _block_value;
    //! @param _clock_count private variable to count the clocks, which ara send to the ADC
    volatile uint8_t _clock_count;
    //! @param _vlaue private variable for the measured value of the analog signal
    volatile int32_t _value;
    //! @param _interrupt_reading private flag to know, if there is an interrupt reading in process or not
    bool _interrupt_reading = false;


public:
    //public variables 
    //! @param _offset public variable for the offset of the measured analog signal
    int32_t _offset = 0;

    //public functions
    CS1237(uint8_t sck, uint8_t dout);
    ~CS1237();
    int32_t reading();
    void send_clk_pulses(byte count_);
    byte raw_configure(bool write, int32_t *result = NULL, byte gain = PGA_128, byte speed = SPEED_1280, byte channel = CHANNEL_A);
    bool configure(int32_t *result, byte gain = PGA_128, byte speed = SPEED_1280, byte channel = CHANNEL_A);
    void tare(uint16_t time_);
    int32_t read_without_interrupt(void);
    void start_reading(void);
    void end_reading(void);
    void sleep(bool sleep_ = true);

    //interrupt and timer functions
    void IRAM_ATTR instance_ISR(void);                                 
    void IRAM_ATTR instance_timer_ISR(void);                         
    void IRAM_ATTR timer_init(uint8_t object_number_);  	
    void IRAM_ATTR timer_stop(uint8_t object_number_);

    static void IRAM_ATTR timer_ISR0(void);
    static void IRAM_ATTR timer_ISR1(void);

    static void IRAM_ATTR ISR0(void);
    static void IRAM_ATTR ISR1(void);
};
   
#endif  //CS1237