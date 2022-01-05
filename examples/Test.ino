#include <CS1237.h>

#define SCK 25
#define DOUT_1 33
#define DOUT_2 32


#define READ  0x56
#define WRITE 0x65

uint8_t DOUTS[3] = {DOUT_1, DOUT_2};

#define CHANNEL_COUNT sizeof(DOUTS)/sizeof(uint8_t)

CS1237 ADCs(CHANNEL_COUNT, SCK, DOUTS);

void setup() {
    Serial.begin(115200);
    Serial.println("Starting");
}

void loop() {
    ADCs.read();
    delay(100);
}