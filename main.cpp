#include "pcbnAPI.h"
#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "USBHostSerial.h"
#include "Drivers.h"
#include "server.h"
#include <bsmp/server.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "PID.h"

#define BUFSIZE 140
#define SERVER_PORT 6791

//Variables ID's
#define SwitchingID 0
#define Att1ID 1
#define Att2ID 2
#define Temp1ID 3
#define Temp2ID 4
#define Temp3ID 5
#define Temp4ID 6
#define Set_Point1ID 7
#define Set_Point2ID 8
#define Temp_ControlID 9
#define Output1ID 10
#define Output2ID 11
#define ResetID 12
#define ReprogrammingID 13
#define DataID 14
#define VersionID 15
#define Switch_LevelID 16
#define VarCount 17

// Constants
#define Rate 1.0
#define MaxPIDout 3.3
#define MinPIDout 0.0
#define Refin 3.3
#define DataSize 127

extern "C" void mbed_reset();

// Struct of the variables
struct bsmp_var dummy[VarCount];
uint8_t Switching[1];
uint8_t Switch_Level[1];
uint8_t Att1[8];
uint8_t Att2[8];
uint8_t Temp1[8];
uint8_t Temp2[8];
uint8_t Temp3[8];
uint8_t Temp4[8];
uint8_t Set_Point1[8];
uint8_t Set_Point2[8];
uint8_t Temp_Control[1];
uint8_t Output1[8];
uint8_t Output2[8];
uint8_t Reset[1];
uint8_t Reprogramming[1];
uint8_t Data[DataSize];
uint8_t Version[8];

LocalFileSystem localdir("local");               // Create the local filesystem under the name "local"
FILE *fp;
Ethernet *cable = new Ethernet;

// Inicializations - MBED

// MBED Leds
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);

// MBED pins

DigitalOut clk_en(p8); // Enable clock for the Switching. LVTTL, low = disable, init = low.Switching
DigitalIn clk_0(p9); // Input of switching clock. LVTTLReceive switching clock
DigitalOut dataC(p10); // Data line to attenuator. LVTTL, low = reset, init = low.Set first attenuators (Att1)
DigitalOut dataA(p11); // Data line to attenuator. LVTTL, low = reset, init = low.Set first attenuators (Att1)
DigitalOut clk(p12); // Digital control attenuation. LVTTL, low = reset, init = low.Digital control attenuation
DigitalOut LE(p13); // Digital control calibration. LVTTL, low = reset, init = low.Digital control calibration
DigitalOut CSac(p14); // Chip select for ADT7320UCPZ-R2. LVTTL, high = disable, init = high.Temp. measurement in RFFE_AC
DigitalOut SHDN_temp(p15); // Shut down the temperature current boost output amplifier. LVTTL, low = disable, init = low.Digital control
DigitalOut sw1(p17);     // Control pin for the RF switch 1
DigitalOut sw2(p18);    //Control pin for the RF switch 2
DigitalOut led_g(p19); // Green LED
DigitalOut led_r(p20); // Red LED
DigitalOut S2(p21); // Set input of JK2. LVTTL, init = high.Switching
DigitalOut K2(p22); // K input of JK2. LVTTLSwitching
DigitalOut J2(p23); // J input of JK2. LVTTLSwitching
DigitalOut R2(p24); // Reset pin of JK2. LVTTL, init = low.Digital control
DigitalOut CSbd(p25); // . LVTTL, high = disable, init = high.
DigitalOut dataB(p26); // Data line to attenuator. LVTTL, low = reset, init = low.Set first attenuators (Att1)
DigitalOut dataD(p27); // Data line to attenuator. LVTTL, low = reset, init = low.Set first attenuators (Att1)
DigitalOut CS_dac(p16); // Chip select for DAC.. LVTTL, low = Selected, init = high.Chip select
DigitalOut LedY(p29); // Yellow led of the Ethernet connector. LVTTLIndicate transmiting data
DigitalOut LedG(p30); // Green led of the Ethernet connector. LVTTLIndicate active connection
Serial pc(USBTX, USBRX);

// spi(mosi, miso, sclk)
SPI spi1(p5,p6,p7);

//********************************************** Drivers functions **********************************************************
void ADT7320_config(mbed::DigitalOut cs)
{
    spi1.frequency(1000000);
    cs = 1;
    Thread::wait(1);
    cs = 0;

    // Reseting SPI interface
    spi1.format(16,3);
    spi1.write(0xFFFF);
    spi1.write(0xFFFF);
    spi1.write(0xFFFF);

    Thread::wait(1);
    cs = 1;

    spi1.format(8,3);

    cs = 0;
    Thread::wait(1);

    // Configuration process

    // CONFIGURATION REGISTER (16 bits) – 0x01
    spi1.write(0x08);

    // COMMUNICATIONS REGISTER (8bits) setting to 16-bit resolution
    spi1.write(0x80);

    Thread::wait(1);
    cs = 1;
    Thread::wait(1);
}

double ADT7320_read(mbed::DigitalOut cs)
{
    int data;
    int reference = 0;
    double delta = 128;
    double temp;

    cs = 1;
    spi1.frequency(1000000);
    spi1.format(16,3);

    cs = 0;
    spi1.write(0x50);
    data = spi1.write(0x00);
    cs = 1;
    //printf("\nFrom slave: %X\n", data);
    temp = (float(data)-reference)/delta;
    return temp;
}

void DAC8552_write(mbed::DigitalOut cs, double voutA, double voutB)
{
    // SPI config
    spi1.frequency(1000000);
    spi1.format(16,1);

    int data;

    // Calculating data to voutA
    data = int(voutA*65536/Refin);
    // Transmition
    cs = 0;
    Thread::wait(1);
    // control - write data to voutA
    spi1.write(0x00);
    spi1.write(data);
    Thread::wait(1);
    cs = 1;
    Thread::wait(1);

    // Calculating data to voutB
    data = int(voutB*65536/Refin);
    // Transmition
    cs = 0;
    Thread::wait(1);
    // control - write data to voutA
    spi1.write(0x04);
    spi1.write(data);
    Thread::wait(1);
    cs = 1;
    Thread::wait(1);
}

//********************************************** Thread functions **********************************************************
void Temp_Feedback_Control(void const *args)
{
    // Control bits         Data bits   DAC     Function
    // 1 0 0 0 (0x8000)     12 bits      A      Input register and DAC register updated, output updated
    // 1 0 0 1 (0x9000)     12 bits      B      Input register and DAC register updated, output updated
    // 1 0 1 0 (0xA000)     12 bits      C      Input register and DAC register updated, output updated
    // 1 0 1 1 (0xB000)     12 bits      D      Input register and DAC register updated, output updated
    // vout = Refin * data  / 4096
    // data = vout * 4096 / Refin

    // Init. config
    ADT7320_config(CSac);
    ADT7320_config(CSbd);

    float ProcessValueA, ProcessValueB;
    double voutA, voutB;
    int state = 2;
    PID pidA(1.0, 1.0, 1.0, Rate);
    pidA.setInputLimits(0.0 , 80.0);
    pidA.setOutputLimits(MinPIDout , MaxPIDout);
    PID pidB(1.0, 1.0, 1.0, Rate);
    pidB.setInputLimits(0.0 , 80.0);
    pidB.setOutputLimits(MinPIDout , MaxPIDout);

    while (1) {
        // Read temp from ADT7320 in RFFE_AC
        set_value(Temp1,ADT7320_read(CSac));

        // Read temp from ADT7320 in RFFE_BD
        set_value(Temp2,ADT7320_read(CSbd));

        //printf("\n Temp1(RFFE_AC): %f\n Temp2(RFFE_BD): %f\n",get_value64(Temp1),get_value64(Temp2));

        if (state != Temp_Control[0]) {
            state = Temp_Control[0];
            if (Temp_Control[0] == 0) {
                set_value(Output1,0.0);
                set_value(Output2,0.0);
            }
        }
        if ((Temp_Control[0] == 0) && (get_value64(Output1) == 0.0) && (get_value64(Output2) == 0.0)) {
            SHDN_temp = 0;
            Thread::wait(int(1000*Rate));
            continue;
        } else
            SHDN_temp = 1;

        if (Temp_Control[0] == 1) {
            // Calculating the Process Values
            // Compoundind de process value from the two temperature measure
            ProcessValueA = (float)( get_value64(Temp2) );
            // Compoundind de process value from the two temperature measure
            ProcessValueB = (float)( get_value64(Temp1) );

            // Vout A - Output to heater BD
            // Temperature measurements from Temp3 and Temp4

            pidA.setSetPoint((float)get_value64(Set_Point2));
            pidA.setProcessValue(ProcessValueA);
            // Control output
            set_value(Output2,pidA.compute());

            // Vout B - Output to heater AC
            // Temperature measurements from Temp1 and Temp2

            pidB.setSetPoint((float)get_value64(Set_Point1));
            pidB.setProcessValue(ProcessValueB);
            // Control output
            set_value(Output1,pidB.compute());
        }

        // Update control output
        if (get_value64(Output2) > MaxPIDout)
            set_value(Output2, MaxPIDout);
        if (get_value64(Output1) > MaxPIDout)
            set_value(Output1, MaxPIDout);
        if (get_value64(Output2) < MinPIDout)
            set_value(Output2, MinPIDout);
        if (get_value64(Output1) < MinPIDout)
            set_value(Output1, MinPIDout);

        voutA = get_value64(Output2);
        voutB = get_value64(Output1);
        printf("\n vout(heat AC): %f \n", voutB);
        printf("\n vout(heat BD): %f \n", voutA);
        DAC8552_write(CS_dac, voutA, voutB);

        Thread::wait(int(1000*Rate));
    }
}

void Switching_set(int state)
{
    switch (state) {
        case 0:
            // Initial state: switches on the direct position
            sw1 = 1;
            sw2 = 1;
            // Leds show the switch state
            led1 = 0;
            led2 = 0;
            led_g=1;
            led_r=0;
            //printf("\nSwitches direct\n");
            break;

        case 1:
            // Set switches on the inverse position
            led_g=0;
            led_r=1;
            sw1 = 0;
            sw2 = 0;

            led1 = 0;
            led2 = 1;
            //printf("\nSwitches inverted\n");
            break;

    }
}

void int2bin6(int value, bool * bin)
{
    int resul;
    bin[0] = 0;
    bin[1] = 0;
    bin[2] = 0;
    bin[3] = 0;
    bin[4] = 0;
    bin[5] = 0;
    for (int i = 5; i >= 0; i--) {
        bin[i] = 1;
        resul = bin[5]*32+bin[4]*16+bin[3]*8+bin[2]*4+bin[1]*2+bin[0]*1;
        if (resul == value) return;
        if (resul > value) bin[i] = 0;
    }
}

void Clk_att(void const *arg)
{
    clk = !clk;
}

void Switching_Attenuators_Control(void const *arg)
{
    int state = 4;
    double prev_att1 = get_value64(Att1)+1;
    double prev_att2 = get_value64(Att2)+1;
    bool attVec1[6];
    bool attVec2[6];
    RtosTimer Clock_thread(Clk_att);


    while (1) {

        // Ethernet link test
        if (cable->link())
            LedY = 1;
        else
            LedY = 0;

        // Attenuators set
        if ( (prev_att1 != get_value64(Att1)) || (prev_att2 != get_value64(Att2))) {
            // Checking and setting attenuators value to fisable values
            set_value(Att1,(float)(int(get_value64(Att1)*2))/2);
            set_value(Att2,(float)(int(get_value64(Att2)*2))/2);
            // Updating previous values
            prev_att1 = get_value64(Att1);
            prev_att2 = get_value64(Att2);
            //printf("\nAtt values requested: \n Att1: %f \n Att2: %f.\n",prev_att1,prev_att2);
            //printf("\nAtt values updated to: \n Att1: %f \n Att2: %f.\n",(float)(int(prev_att1*2))/2,(float)(int(prev_att2*2))/2);
            int2bin6(int(prev_att1*2), attVec1);
            int2bin6(int(prev_att2*2), attVec2);
            LE = 0;
            clk = 0;
            Clock_thread.start(10);
            // Serial data to attenuators
            for (int i = 5; i >= 0; i--) {
                dataA = attVec1[i];
                dataB = attVec1[i];
                dataC = attVec1[i];
                dataD = attVec1[i];
                while (clk == 0) Thread::wait(1);// Waiting clock change
                while (clk == 1) Thread::wait(1);// Waiting clock change
            }
            LE = 1;
            Thread::wait(1);
            LE = 0;
            Clock_thread.stop();
        }
//        Switching_set(3);
        // Switching set
        if (state != Switching[0]) {
            state = Switching[0];
            Switching_set(state);
        }
        Thread::wait(1000);
        
    }
}

void set_var(bsmp_var * dummy, int ID, bool writable, int size, uint8_t * value)
{
    dummy[ID].info.id = ID;
    dummy[ID].info.writable = writable;
    dummy[ID].info.size = size;
    dummy[ID].data = value;
}

void echo_command(char * cmd)
{
    if (cmd[0] == 0x20) {
        printf("\nId: %d",dummy[cmd[3]].info.id);
        if (dummy[cmd[3]].info.size == 1) {
            printf("\nValue: %d",*dummy[cmd[3]].data);
        } else {
            printf("\nValue: %f",get_value64(dummy[cmd[3]].data));
        }
        printf("\n");
    }
}

int check_name(char * name)
{
    if ((name[0] == 'V') && isdigit(name[1]) && (name[2] == '_') && isdigit(name[3]) && isdigit(name[4]) && isdigit(name[5]) && isdigit(name[6]))
        return ((name[3]-48)*1000+(name[2]-48)*100+(name[1]-48)*10+(name[0]-48));
    else
        return 0;
}

int str2int(char * str)
{
    return ((str[0]-48)*1000 + (str[1]-48)*100 + (str[2]-48)*10 + (str[3]-48));
}

void int2str(char * str, int num)
{
    str[0] = char(num/1000 + 48);
    num = num%1000;
    str[1] = char(num/100 + 48);
    num = num%100;
    str[2] = char(num/10 + 48);
    num = num%10;
    str[3] = num+48;
}

void Update_Software(char * old_name, char * name)
{
    char path[15] = "/local/";
    if (fopen((char*)strcat(path,name),"rb") == NULL) {
        printf("\nNew firmare not found\n");
        return;
    }
    printf("\nReprogramming...\n");
    strcpy(path,"/local/");
    remove((char*)strcat(path,old_name));
    printf("\nReseting...\n");
    mbed_reset();

}

void Data_Check()
{
    for (int i = 0; i < DataSize; i++) {
        if ((i < (DataSize-4)) && Data[i] == 13 && Data[i+1] == 13 && Data[i+2] == 13 && Data[i+3] == 13)
            break;
        fputc(Data[i], fp);
    }
}

void Serial_Interface(void const*)
{
    USBHostSerial serial;

    while(1) {

        // try to connect a serial device
        while(!serial.connect())
            Thread::wait(500);

        // in a loop, print all characters received
        // if the device is disconnected, we try to connect it again
        while (1) {

            // if device disconnected, try to connect it again
            if (!serial.connected())
                break;

            // print characters received
            while (serial.available()) {
                printf("%c", serial.getc());
            }

            Thread::wait(50);
        }
    }
}

int main()
{
    // Setup of variables
    enum bsmp_err err;
    bsmp_server_t *bsmp = bsmp_server_new();
    led_g=0;
    led_r=0;
    // Variables initialization

    // Switching
    Switching[0] = 0;
    set_var(dummy, SwitchingID, true, 1, Switching);
    err = bsmp_register_variable(bsmp,&dummy[SwitchingID]);
            
    // Att1
    set_value(Att1,30.0);
    set_var(dummy, Att1ID, true, 8, Att1);
    err = bsmp_register_variable(bsmp,&dummy[Att1ID]);

    // Att2
    set_value(Att2,30.0);
    set_var(dummy, Att2ID, true, 8, Att2);
    err = bsmp_register_variable(bsmp,&dummy[Att2ID]);

    // Temp1
    set_value(Temp1,0.0);
    set_var(dummy, Temp1ID, false, 8, Temp1);
    err = bsmp_register_variable(bsmp,&dummy[Temp1ID]);

    // Temp2
    set_value(Temp2,0.0);
    set_var(dummy, Temp2ID, false, 8, Temp2);
    err = bsmp_register_variable(bsmp,&dummy[Temp2ID]);

    // Temp3
    set_value(Temp3,0.0);
    set_var(dummy, Temp3ID, false, 8, Temp3);
    err = bsmp_register_variable(bsmp,&dummy[Temp3ID]);

    // Temp4
    set_value(Temp4,0.0);
    set_var(dummy, Temp4ID, false, 8, Temp4);
    err = bsmp_register_variable(bsmp,&dummy[Temp4ID]);

    // Set_Point1
    set_value(Set_Point1,0.0);
    set_var(dummy, Set_Point1ID, true, 8, Set_Point1);
    err = bsmp_register_variable(bsmp,&dummy[Set_Point1ID]);

    // Set_Point2
    set_value(Set_Point2,0.0);
    set_var(dummy, Set_Point2ID, true, 8, Set_Point2);
    err = bsmp_register_variable(bsmp,&dummy[Set_Point2ID]);

    // Temp_Control
    Temp_Control[0] = 0;
    set_var(dummy, Temp_ControlID, true, 1, Temp_Control);
    err = bsmp_register_variable(bsmp,&dummy[Temp_ControlID]);

    // Output1
    set_value(Output1,0.0);
    set_var(dummy, Output1ID, true, 8, Output1);
    err = bsmp_register_variable(bsmp,&dummy[Output1ID]);

    // Output2
    set_value(Output2,0.0);
    set_var(dummy, Output2ID, true, 8, Output2);
    err = bsmp_register_variable(bsmp,&dummy[Output2ID]);

    // Reset
    Reset[0] = 0;
    set_var(dummy, ResetID, true, 1, Reset);
    err = bsmp_register_variable(bsmp,&dummy[ResetID]);

    // Reprogramming
    Reprogramming[0] = 0;
    set_var(dummy, ReprogrammingID, true, 1, Reprogramming);
    err = bsmp_register_variable(bsmp,&dummy[ReprogrammingID]);

    // Data
    //set_value(Data,"       ");
    set_var(dummy, DataID, true, DataSize, Data);
    err = bsmp_register_variable(bsmp,&dummy[DataID]);

    // Version
    set_value(Version,"V2_0005");
    set_var(dummy, VersionID, false, 8, Version);
    err = bsmp_register_variable(bsmp,&dummy[VersionID]);

    // Switch_Level
    Switch_Level[0] = 0;
    set_var(dummy, Switch_LevelID, true, 1, Switch_Level);
    err = bsmp_register_variable(bsmp,&dummy[Switch_LevelID]);

    int s,numVer;
    uint8_t state = 0;

    /*
    for (s = 0; s < VarCount; s++) {
        printf("\nId: %d",dummy[s].info.id);
        printf("\nWritable: %d",dummy[s].info.writable);
        printf("\nSize: %d bytes",dummy[s].info.size);
        if (dummy[s].info.size == 1) {
            printf("\nValue: %d",*dummy[s].data);
        } else {
            if (s != VersionID)
                printf("\nValue: %f",get_value64(dummy[s].data));
            else
                printf("\nValue: %s",dummy[s].data);
        }
        printf("\n");
    }
    */

    struct bsmp_raw_packet request;
    struct bsmp_raw_packet response;
    uint8_t buf[BUFSIZE];
    uint8_t bufresponse[BUFSIZE];
    char path[15], old_name[15], name[15];

    DIR *d = opendir("/local");
    struct dirent *p;
    numVer = 10000;
    while((p = readdir(d)) != NULL) {
        if (check_name(p->d_name) && check_name(p->d_name) < numVer) {
            strcpy(old_name, p->d_name);
            numVer = check_name(p->d_name);
        }
    }
    printf("\old_name: %s\n",old_name);
    closedir(d);

    // *************************************Threads***************************************

    Thread Switching_Attenuators_thread(Switching_Attenuators_Control);
    printf("\nSwitching_Attenuators_thread\n");

    Thread Temp_Control_thread(Temp_Feedback_Control);
    printf("\nTemp_Control_thread\n");

    //Thread Serial_Interface_thread(Serial_Interface, NULL, osPriorityNormal, 256 * 4);;
    //printf("\nSerial_USB_thread\n");

    // Ethernet initialization

    //EthernetInterface eth;
    static const char* mbedIp       = "10.0.18.59" ;  //IP
    static const char* mbedMask     = "255.255.255.0";  // Mask
    static const char* mbedGateway  = "10.0.18.1";    //Gateway
    //EthernetInterface::init(mbedIp,mbedMask,mbedGateway); //Use  these parameters for static IP
    EthernetInterface::init(); //Use DHCP

    TCPSocketConnection client;
    TCPSocketServer server;

    printf("\n Trying to establish connection...\n");
    while (EthernetInterface::connect(5000)) {
        printf("\n Connection failure\n");
    }
    printf("\nConnected\n");

    while (true) {

        led3 = 1;

        server.bind(SERVER_PORT);
        server.listen();

        printf("    IP Address is %s\n", EthernetInterface::getIPAddress());
        printf("    MAC Address is %s\n", EthernetInterface::getMACAddress());
        printf("    port %d\n", SERVER_PORT);
        printf("\n Wait for new client connection...\n");
        server.accept(client);
        client.set_blocking(false, 1500); // Timeout after (1.5)s

        printf("Connection from client: %s\n", client.get_address());
        printf("port: %d\n", client.get_port());
        while (client.is_connected() && LedY) {
            led4 = 1;
            s = client.receive((char*)buf, BUFSIZE);
            LedG = 1;
            Thread::wait(10);
            if (s <= 0) {
                LedG = 0;
                continue;
            }

            /*
            printf("\nMsg recebida de %d bytes: ", s);
            for (int i = 0; i < s; i++)
                printf("%x ",(char*)buf[i]);
            printf("\n");
            */

            request.data = buf;
            request.len = s;

            response.data = bufresponse;

            bsmp_process_packet (bsmp,&request,&response);

            s = client.send((char*)response.data, response.len);

            /*
            printf("\nMsg enviada de %d bytes: ",s);
            for (int i = 0; i < s; i++)
                printf("%x ",(char*)response.data[i]);
            printf("\n");
            */

            if (s < 0)
                error("ERROR writing to socket");

            //echo_command((char*)buf);

            if (state != Reprogramming[0]) {
                switch (Reprogramming[0]) {
                    case 0:
                        if (state == 1)
                            fclose(fp);
                        state = Reprogramming[0];
                        break;

                    case 1:
                        /*
                            DIR *d = opendir("/local");
                            struct dirent *p;
                            numVer = 10000;
                            while((p = readdir(d)) != NULL) {
                                if (check_name(p->d_name) && check_name(p->d_name) < numVer) {
                                    strcpy(name, p->d_name);
                                    numVer = check_name(p->d_name);
                                }
                            }
                            closedir(d);
                        */
                        strcpy(name,old_name);
                        strcpy(path,"/local/");
                        int2str(&name[3],(str2int(&old_name[3])%9999+1));
                        printf("\name: %s\n",name);
                        fp = fopen((char*)strcat(path,name), "wb");
                        state = Reprogramming[0];
                        break;

                    case 2:
                        if (state == 1)
                            fclose(fp);
                        state = Reprogramming[0];
                        printf("\ndebug1\n");
                        Update_Software(old_name, name);
                        break;

                }
            }

            if ((Reprogramming[0] == 1) && (buf[0] == 0x20) && (buf[3] == DataID))
                Data_Check();

            if (Reset[0] == 1)
                mbed_reset();

            LedG = 0;
        }
        //printf("\nDisconnected: %d\n",eth.disconnect());
        client.close();
        server.close();
        if (!cable->link()) {
            EthernetInterface::disconnect();
            printf("\n Trying to establish connection...\n");
            while (EthernetInterface::connect(5000)) {
                printf("\n Connection failure\n");
            }
            printf("\nConnected\n");
        }
        printf("\nClient disconnected\n");
        led4 = 0;
        led3 = 0;
    }
}
