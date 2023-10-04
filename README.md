## Description:

This is the library for the CS1237-IC from Chipsea. the translated datasheet can be found [here](https://github.com/rafaellcoellho/cs1237-datasheet/blob/master/cs1237_datasheet.pdf)

The library is designed to be used with one or two CS1237-ADCs-Chips, because if you uses the high frequency mode (1280Hz), the timing is though. The gain (1,2,64,128), the sample rate(10Hz,40Hz,640Hz,1280Hz), the reference voltage (internal, or external) 
and the channel(external differential signal, temperature sensor) can be configured.

You have to use one Clock line for each ADC. I have experimented with multiple ADC-Chips on one clock-line, but it is impossible to snchronize these chips.
Synchronisation is very important, because depending on the configured sample rate there is a regular interrupt from the ADC Chip on the data line. 
If the data transmission is in progress, while the interrupt happens, the transmitted value is wrong.

This library is written with hardware interrupts and timers, so it can run in the background.


## Preconditions

- At this time it is only allowed to connect only two ADCs with one clock and one data line each.
- you could use nearly any GPIO for the clock lines and any GPIO with an interrupt function for the Data line.


## Compatibility

This library was only tested on the ESP32 DEV_KIT_V4

## Author
Written by Julian Weidinger for an University Project.
