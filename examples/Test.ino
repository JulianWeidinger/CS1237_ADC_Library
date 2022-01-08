
#include <CS1237.h>

#define SCK_1 12
#define SCK_2 25
#define DOUT_1 32
#define DOUT_2 33

#define ADC_COUNT 2

#define READ 0x56
#define WRITE 0x65

CS1237 ADC_2(SCK_1, DOUT_1);
CS1237 ADC_1(SCK_2, DOUT_2);

void setup()
{
    int32_t values[ADC_COUNT];
    Serial.begin(115200);
    Serial.println("configuration: ");

    byte write[ADC_COUNT];
    byte read[ADC_COUNT];

    if (ADC_1.configure(values, PGA_128, SPEED_1280, CHANNEL_A) && ADC_2.configure(values, PGA_128, SPEED_1280, CHANNEL_A))
        Serial.println("succeed");
    else
        Serial.println("failed");
    /*
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
    Serial.println("");
    ADC_1.start_reading();
    Serial.println("ADC1 started");
    ADC_2.start_reading();
    Serial.println("ADC2 started");
}

void loop()
{
    // Serial.print("ADC1: ");
    Serial.print(ADC_1.reading());
    Serial.print(",");
    Serial.println(ADC_2.reading());

    //delayMicroseconds(int(1000000 / 1280));
}