#include "Drivers.h"

double AD7792(mbed::SPI spi, mbed::DigitalOut cs){
    int data;
    int reference = 42011;
    double delta = 161.2622;
    double temp;
    cs = 1;
    spi.frequency(1000000);
    spi.format(16,3);
    cs = 0;
    
    // AD7792 Configuration process
    
    // MODE REGISTER (16 bits) – 0x000A (Default) Send nothing
    
    // COMMUNICATIONS REGISTER (8bits) setting to CONFIGURATION REGISTER
    spi.write(0x10);
    
    // CONFIGURATION REGISTER (16 bits) – 0x1410
    spi.write(0x1410);
    
    // COMMUNICATIONS REGISTER (8bits) setting to IO REGISTER
    spi.write(0x28);
    
    // IO REGISTER (8 bits) – 0x03
    spi.write(0x03);

    cs = 0;
    // command to read data register 0x58
    data = spi.write(0x58);
    cs = 1;
    //printf("\nFrom slave: %X\n", data);
    temp = ((data)-reference)/delta;
    return temp;
}

int TCA6424(mbed::I2C i2c, int addr, bool *port0, bool *port1, bool *port2) {
    // Write all 3 ports starting from Output Port 0
    
    int outputPort;

    // start transmission
    i2c.start();
    
    // address = |0|1|0|0|0|1|addr|R/W|
    //ADDR pin = Low => addr = 0. ADDR pin = High => addr = 1
    // R/W: read = 1 or write = 0 bit
    if (i2c.write( (addr==1) ? 0x46 : 0x44 ) == 0) return 0;
    
    // sending command to read/write starting from Output Port 0
    if (i2c.write(0x04) == 0) return 0;
    
    // mounting the Output Port 0
    outputPort = port0[7]*1 + port0[6]*2 + port0[5]*4 + port0[4]*8 + port0[3]*16 + port0[2]*32 + port0[1]*64 + port0[0]*128;
    if (i2c.write(outputPort) == 0) return 0;
    
    // mounting the Output Port 1
    outputPort = port1[7]*1 + port1[6]*2 + port1[5]*4 + port1[4]*8 + port1[3]*16 + port1[2]*32 + port1[1]*64 + port1[0]*128;
    if (i2c.write(outputPort) == 0) return 0;
    
    // mounting the Output Port 2
    outputPort = port2[7]*1 + port2[6]*2 + port2[5]*4 + port2[4]*8 + port2[3]*16 + port2[2]*32 + port2[1]*64 + port2[0]*128;
    if (i2c.write(outputPort) == 0) return 0;
    
    // ending transmission
    i2c.stop();
    
    return 1;
}

int TCA6424(mbed::I2C i2c, int addr, bool *port0, unsigned char n) {
    // Write to 1 port starting from Output Port n, n = 0, 1 or 2
    
    if (n > 2) return 0;
    int outputPort;

    // start transmission
    i2c.start();
    
    // address = |0|1|0|0|0|1|addr|R/W|
    //ADDR pin = Low => addr = 0. ADDR pin = High => addr = 1
    // R/W: read = 1 or write = 0 bit
    if (i2c.write( (addr==1) ? 0x46 : 0x44 ) == 0) return 0;
    
    // sending command to read/write starting from Output Port 0
    if (i2c.write(0x04+n) == 0) return 0;
    
    // mounting the Output Port 0
    outputPort = port0[7]*1 + port0[6]*2 + port0[5]*4 + port0[4]*8 + port0[3]*16 + port0[2]*32 + port0[1]*64 + port0[0]*128;
    if (i2c.write(outputPort) == 0) return 0;
    
    // ending transmission
    i2c.stop();
    
    return 1;
}

int TCA6424(mbed::I2C i2c, int addr, bool *port0, bool *port1, unsigned char n) {
    // Write to 2 ports starting from Output Port n, n = 0, 1 or 2
    
    if (n > 2) return 0;
    int outputPort;

    // start transmission
    i2c.start();
    
    // address = |0|1|0|0|0|1|addr|R/W|
    //ADDR pin = Low => addr = 0. ADDR pin = High => addr = 1
    // R/W: read = 1 or write = 0 bit
    if (i2c.write( (addr==1) ? 0x46 : 0x44 ) == 0) return 0;
    
    // sending command to read/write starting from Output Port 0
    if (i2c.write(0x04+n) == 0) return 0;
    
    // mounting the Output Port 0
    outputPort = port0[7]*1 + port0[6]*2 + port0[5]*4 + port0[4]*8 + port0[3]*16 + port0[2]*32 + port0[1]*64 + port0[0]*128;
    if (i2c.write(outputPort) == 0) return 0;
    
    // mounting the Output Port 1
    outputPort = port1[7]*1 + port1[6]*2 + port1[5]*4 + port1[4]*8 + port1[3]*16 + port1[2]*32 + port1[1]*64 + port1[0]*128;
    if (i2c.write(outputPort) == 0) return 0;
    
    // ending transmission
    i2c.stop();
    
    return 1;
}

void DAC7554(mbed::SPI spi, mbed::DigitalOut Sync, float Refin, float voutA, float voutB, float voutC, float voutD)
// Control bits         Data bits   DAC     Function
// 1 0 0 0 (0x8000)     12 bits      A      Input register and DAC register updated, output updated
// 1 0 0 1 (0x9000)     12 bits      B      Input register and DAC register updated, output updated
// 1 0 1 0 (0xA000)     12 bits      C      Input register and DAC register updated, output updated
// 1 0 1 1 (0xB000)     12 bits      D      Input register and DAC register updated, output updated
// vout = Refin * data  / 4096
// data = vout * 4096 / Refin
{
    int data;
    // DAC A
    
    spi.frequency(1000000);
    spi.format(16,1); 
    
    // Calculating data
    data = int(voutA*4096/Refin);
    // Control bits and data
    data = 0x8000 + data;
    // Enabling transmition
    Sync = 1;
    Sync = 0;
    spi.write(data);

    // DAC B
    
    // Calculating data
    data = int(voutB*4096/Refin);
    // Control bits and data
    data = 0x9000 + data;
    // Enabling transmition
    Sync = 1;
    Sync = 0;
    spi.write(data);

    // DAC C
    
    // Calculating data
    data = int(voutC*4096/Refin);
    // Control bits and data
    data = 0xA000 + data;
    // Enabling transmition
    Sync = 1;
    Sync = 0;
    spi.write(data);

    // DAC D
    
    // Calculating data
    data = int(voutD*4096/Refin);
    // Control bits and data
    data = 0xB000 + data;
    // Enabling transmition
    Sync = 1;
    Sync = 0;
    spi.write(data);
}
