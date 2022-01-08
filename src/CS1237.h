/*!
 * @file CS1237.h
 *
 * This is a library for the CS1237-IC converts the readings of a loadcell in
 * 24bit digital numbers
 *
 *
 * The Chip communicates through a 2-wire SPI interface SCLK, DRDY/DOUT,
 * 2 GPIO-pins are required to interface with the CS1237-IC.
 *
 * Written by Julian Weidinger for an University Project.
 * BSD license, all text above must be included in any redistribution
 */

#ifndef CS1237_h
#define CS1237_h


// command to read or write the configuration
#define READ_ADC_CONFIG 0x56
#define WRITE_ADC_CONFIG 0x65

//
#define MAX_ADCS 2
#define TIMER_PRESCALER 80 // 12.5ns*80 = 1000ns

// configuration commands
#define SPEED_10 0b00000000
#define SPEED_40 0b00010000
#define SPEED_640 0b00100000
#define SPEED_1280 0b00110000

#define PGA_1 0b00000000
#define PGA_2 0b00000100
#define PGA_64 0b00001000
#define PGA_128 0b00001100

#define CHANNEL_A 0b00000000
#define CHANNEL_Temp 0b00000010

#include <Arduino.h>

/*!
 * @brief Main BMP085 class
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
    /*!
     * @brief test to establish a connection to the CS1237
     * @param mode Mode to set, ultra high-res by default
     * @param wire The I2C interface to use, defaults to Wire
     * @return Returns true if the gain was set successfully and the chip is ready
     */

    




    CS1237(uint8_t sck, uint8_t dout);
    ~CS1237();
    int32_t reading();
    void send_clk_pulses(byte count);
    byte raw_configure(bool write, int32_t *result = NULL, byte gain = PGA_128, byte speed = SPEED_1280, byte channel = CHANNEL_A);
    bool configure(int32_t *result, byte gain = PGA_128, byte speed = SPEED_1280, byte channel = CHANNEL_A);
    int32_t read_without_interrupt(void);
    void start_reading(void);
    void end_reading(void);
    void sleep(bool sleep_ = true);
    
    void instanceISR(void); // Instance ISR handler called from static ISR globalISRx
    void instance_timer_ISR(void);
    void IRAM_ATTR timer_init(uint8_t object_number_);
    void IRAM_ATTR timer_stop(uint8_t object_number_);

    static void IRAM_ATTR timer_ISR0(void);
    static void IRAM_ATTR timer_ISR1(void);

    static void IRAM_ATTR ISR0(void);
    static void IRAM_ATTR ISR1(void);
};
    //use global variables because static doesn't work
    
    
#endif  //CS1237