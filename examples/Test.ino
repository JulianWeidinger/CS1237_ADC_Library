/***************************************************************************
  This is a library for the CS1237 ADC-Chip. Designed specifically to work
  with the ESP32 and two loadcells with a sample rate of 1280Hz.
  The ADC-Chip use a SPI like interface with one clock- and one data-line.

  You can find an english datasheet here:
  https://github.com/rafaellcoellho/cs1237-datasheet/blob/master/cs1237_datasheet.pdf


  Written by Julian Weidinger for an university project
 ***************************************************************************/

#include <CS1237.h>

// ADC-GPIOs
#define SCK_1 12
#define SCK_2 25
#define DOUT_1 32
#define DOUT_2 33

#define ADC_COUNT 2

// create the two ADC objects
CS1237 ADC_1(SCK_1, DOUT_1);
CS1237 ADC_2(SCK_2, DOUT_2);

void setup()
{
    Serial.begin(115200);
    Serial.print("configuration: ");

//configure the ADCs: CS1237.configure(pointer to 24bit reading, gain setting, sample rate, channel)
    int32_t values[ADC_COUNT];
    if (ADC_1.configure(&values[0], PGA_128, SPEED_1280, CHANNEL_A) && ADC_2.configure(&values[1], PGA_128, SPEED_1280, CHANNEL_A))
        Serial.println("succeed");
    else
        Serial.println("failed");

    //if you want to see the bit values of the confguraten uncomment the following lines
    /*
        byte write[ADC_COUNT];
        byte read[ADC_COUNT];
        write[0] = ADC_1.raw_configure(true, &values[0], byte(PGA_128), byte(SPEED_1280), byte(CHANNEL_A));
        read[0] = ADC_1.raw_configure(false, &values[0], byte(PGA_128), byte(SPEED_1280), byte(CHANNEL_A));
        write[1] = ADC_2.raw_configure(true, &values[1], byte(PGA_128), byte(SPEED_1280), byte(CHANNEL_A));
        read[1] = ADC_2.raw_configure(false, &values[1], byte(PGA_128), byte(SPEED_1280), byte(CHANNEL_A));

        for (uint8_t i = 0; i < ADC_COUNT; i++)
        {
            Serial.print("loadcell ");
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(write[i], BIN);
            Serial.print("; ");
        }
        Serial.println("");
        for (uint8_t i = 0; i < ADC_COUNT; i++)
        {
            Serial.print("loadcell ");
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(read[i], BIN);
            Serial.print("; ");
        }
    */

//start the backround reading of both ADCs
    Serial.println("");
    ADC_1.start_reading();
    ADC_1.tare(1000);
    Serial.println("ADC1 started and tared");
    ADC_2.start_reading();
    ADC_2.tare(1000);
    Serial.println("ADC2 started and tared");
    Serial.println("");

    
    


}

void loop()
{
    //print the meausred values constantly
    Serial.print(ADC_1.reading());
    Serial.print(",");
    Serial.println(ADC_2.reading());

    // uncomment the following line to delay for lower frequencys, for 1280Hz the Serial.print comment tooks a similar time like the delay
    // delayMicroseconds(int(1000000 / 1280));
}