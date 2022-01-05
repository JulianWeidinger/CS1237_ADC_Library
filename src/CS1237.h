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
    //! @param _count private variable of the count of the ADC conected to one SCK-line
    uint8_t _count;
    //! @param _dout private variable of the dout-pin of the first ADC
    uint8_t *_dout;
    //! @param _sck private variable of the clock-pin
    uint8_t _sck;
    //! @param _last_meassure_time private variabel of the time of the last ADC measurement
    uint64_t *_last_meassure_time;
    //! @param sleep private variabel of the sleep state of the chip
    uint8_t _sleep = true;
    //! 

public:
    /*!
     * @brief test to establish a connection to the CS1237
     * @param mode Mode to set, ultra high-res by default
     * @param wire The I2C interface to use, defaults to Wire
     * @return Returns true if the gain was set successfully and the chip is ready
     */


    //! @param last_meassure_time private variabel of the last ADC measurement
    


    CS1237(uint8_t count, uint8_t sck, uint8_t *dout);
    void send_clk_pulses(byte count);
    void raw_configure(bool write, byte *register_value, int32_t *result = NULL, byte gain = PGA_128, byte speed = SPEED_1280, byte channel = CHANNEL_A);
    bool configure(int32_t *result = NULL, byte gain = PGA_128, byte speed = SPEED_1280, byte channel = CHANNEL_A);
    bool ready(void);
    void read(int32_t *result = NULL);
    void sleep(void);
};

#endif // CS1237_h