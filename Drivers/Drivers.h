#include "mbed.h"
#include "rtos.h"

double AD7792(mbed::SPI spi);
int TCA6424(mbed::I2C i2c, int addr, bool *port0, bool *port1, bool *port2);
int TCA6424(mbed::I2C i2c, int addr, bool *port0, unsigned char n);
int TCA6424(mbed::I2C i2c, int addr, bool *port0, bool *port1, unsigned char n);
void DAC7554(mbed::SPI spi, mbed::DigitalOut Sync, float Refin, float voutA, float voutB, float voutC, float voutD);