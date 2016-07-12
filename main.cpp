#include "pcbnAPI.h"
#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "USBHostSerial.h"
#include "Drivers.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "PID.h"
#include "lpc_phy.h"
extern "C" {
#include "server.h"
#include <bsmp/server.h>
}

#define DP8_SPEED10MBPS    (1 << 1)   /**< 1=10MBps speed */
#define DP8_VALID_LINK     (1 << 0)   /**< 1=Link active */

#define BUFSIZE 140
#define SERVER_PORT 6791

//Variables ID's
#define AttID           0
#define TempACID        1
#define TempBDID        2
#define Set_PointACID   3
#define Set_PointBDID   4
#define Temp_ControlID  5
#define HeaterACID      6
#define HeaterBDID      7
#define ResetID         8
#define ReprogrammingID 9
#define DataID          10
#define VersionID       11
#define PID_AC_KcID     12
#define PID_AC_tauIID   13
#define PID_AC_tauDID   14
#define PID_BD_KcID     15
#define PID_BD_tauIID   16
#define PID_BD_tauDID   17
#define VarCount        18

// Constants
#define Rate		1.0
#define MaxPIDout	3.3
#define MinPIDout	0.0
#define Refin		3.3
#define DataSize	127

extern "C" void mbed_reset();

// Struct of the variables
struct bsmp_var dummy[VarCount];
uint8_t Att[8];
uint8_t TempAC[8];
uint8_t TempBD[8];
uint8_t Set_PointAC[8];
uint8_t Set_PointBD[8];
uint8_t Temp_Control[1];
uint8_t HeaterAC[8];
uint8_t HeaterBD[8];
uint8_t Reset[1];
uint8_t Reprogramming[1];
uint8_t Data[DataSize];
uint8_t Version[8];
uint8_t PID_AC_Kc[8];
uint8_t PID_AC_tauI[8];
uint8_t PID_AC_tauD[8];
uint8_t PID_BD_Kc[8];
uint8_t PID_BD_tauI[8];
uint8_t PID_BD_tauD[8];

LocalFileSystem localdir("local");               // Create the local filesystem under the name "local"
FILE *fp;

// Inicializations - MBED

// MBED Leds
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);

// MBED pins

DigitalOut dataC(p10); // Data line to attenuator. LVTTL, low = reset, init = low.Set first attenuators (Att)
DigitalOut dataA(p11); // Data line to attenuator. LVTTL, low = reset, init = low.Set first attenuators (Att)
DigitalOut clk(p12); // Digital control attenuation. LVTTL, low = reset, init = low.Digital control attenuation
DigitalOut LE(p13); // Digital control calibration. LVTTL, low = reset, init = low.Digital control calibration
DigitalOut CSac(p14); // Chip select for ADT7320UCPZ-R2. LVTTL, high = disable, init = high.Temp. measurement in RFFE_AC
DigitalOut SHDN_temp(p15); // Shut down the temperature current boost output amplifier. LVTTL, low = disable, init = low.Digital control
DigitalOut sw1(p17);     // Control pin for the RF switch 1
DigitalOut sw2(p18);    //Control pin for the RF switch 2
DigitalOut led_g(p19); // Green LED
DigitalOut led_r(p20); // Red LED
DigitalOut CSbd(p25); // . LVTTL, high = disable, init = high.
DigitalOut dataB(p26); // Data line to attenuator. LVTTL, low = reset, init = low.Set first attenuators (Att)
DigitalOut dataD(p27); // Data line to attenuator. LVTTL, low = reset, init = low.Set first attenuators (Att)
DigitalOut CS_dac(p16); // Chip select for DAC.. LVTTL, low = Selected, init = high.Chip select
DigitalOut LedY(p29); // Yellow led of the Ethernet connector. LVTTLIndicate transmiting data
DigitalOut LedG(p30); // Green led of the Ethernet connector. LVTTLIndicate active connection
Serial pc(USBTX, USBRX);

// spi(mosi, miso, sclk)
SPI spi1(p5,p6,p7);

bool get_eth_link_status(void)
{
    return (lpc_mii_read_data() & DP8_VALID_LINK) ? true : false;
}

//********************************************** Drivers functions **********************************************************
void ADT7320_config(mbed::DigitalOut cs)
{
    spi1.frequency(1000000);
    cs = 1;

    /* Reseting SPI interface - Write 32 1's to the IC */
    spi1.format(16,3);
    cs = 0;
    Thread::wait(1);
    spi1.write(0xFFFF);
    spi1.write(0xFFFF);
    Thread::wait(1);
    cs = 1;

    // Configuration process
    spi1.format(8,3);
    cs = 0;
    Thread::wait(1);

    // Select CONFIGURATION REGISTER – 0x01
    spi1.write(0x08);

    // Write data to configuration register ( 16-bits resolution + continuous conversion )
    spi1.write(0x80);

    Thread::wait(1);
    cs = 1;
    Thread::wait(1);
}

double ADT7320_read(mbed::DigitalOut cs)
{
    uint16_t data;
    int reference = 0;
    double delta = 128;
    double temp;

    cs = 1;
    spi1.frequency(1000000);
    spi1.format(16,3);

    cs = 0;
    Thread::wait(1);
    // Select Temperature value register
    spi1.write(0x0050);
    data = spi1.write(0x0000);
    cs = 1;
    //printf("\nFrom slave: %X\n", data);
    temp = (float(data)-reference)/delta;
    return temp;
}

#define DAC_AC_SEL 0xA
#define DAC_BD_SEL 0xB

void DAC7554_write(mbed::DigitalOut cs, int dac_sel, double vout)
{
    // Control bits         Data bits   DAC     Function
    // 1 0 0 0 (0x8000)     12 bits      A      Input register and DAC register updated, output updated
    // 1 0 0 1 (0x9000)     12 bits      B      Input register and DAC register updated, output updated
    // 1 0 1 0 (0xA000)     12 bits      C      Input register and DAC register updated, output updated
    // 1 0 1 1 (0xB000)     12 bits      D      Input register and DAC register updated, output updated
    // vout = Refin * data  / 4095
    // data = vout * 4095 / Refin

    uint16_t data;
    uint16_t cfg;

    // SPI config
    spi1.frequency(1000000);
    spi1.format(16,2);
    cs = 1;

    // Calculating data to vout
    data = (uint16_t)(vout*4095/Refin);
    cfg = ( dac_sel << 12 ) | ( data & 0x0FFF );

    // Transmition
    cs = 0;
    Thread::wait(1);
    // control - write data to voutA
    spi1.write( cfg );
    Thread::wait(1);
    cs = 1;
    Thread::wait(1);
}

//********************************************** Thread functions **********************************************************
void Temp_Feedback_Control(void const *args)
{
    // Init. config
    ADT7320_config(CSac);
    ADT7320_config(CSbd);

    float ProcessValueAC, ProcessValueBD;
    double voutAC, voutBD;

    int state = 2;

    /* Create PIDs with generic tuning constants (they will be updated as soon as the control loop starts) */
    PID pidAC( (float)get_value64(PID_AC_Kc), (float)get_value64(PID_AC_tauI), (float)get_value64(PID_AC_tauD), Rate );
    pidAC.setInputLimits( 0.0 , 100.0 );
    pidAC.setOutputLimits( MinPIDout , MaxPIDout );

    PID pidBD( (float)get_value64(PID_BD_Kc), (float)get_value64(PID_BD_tauI), (float)get_value64(PID_BD_tauD), Rate );
    pidBD.setInputLimits( 0.0 , 100.0 );
    pidBD.setOutputLimits( MinPIDout, MaxPIDout );

    while (1) {
        // Read temp from ADT7320 in RFFE_AC
        set_value(TempAC,ADT7320_read(CSac));

        // Read temp from ADT720 in RFFE_BD
        set_value(TempBD,ADT7320_read(CSbd));

        // Update PID tuning values
        pidAC.setTunings( (float)get_value64(PID_AC_Kc), (float)get_value64(PID_AC_tauI), (float)get_value64(PID_AC_tauD) );
        pidBD.setTunings( (float)get_value64(PID_BD_Kc), (float)get_value64(PID_BD_tauI), (float)get_value64(PID_BD_tauD) );

        if (state != Temp_Control[0]) {
            printf ("New temp_control state : %d\n", Temp_Control[0]);
            state = Temp_Control[0];
            if (Temp_Control[0] == 0) {
                printf("Automatic temperature control disabled!\n");
                set_value(HeaterAC,0.0);
                set_value(HeaterBD,0.0);
                continue;
            }
        }
        if ((Temp_Control[0] == 0) && (get_value64(HeaterBD) == 0.0) && (get_value64(HeaterAC) == 0.0)) {
            SHDN_temp = 0;
            led4 = 1;
            Thread::wait(int(1000*Rate));
            continue;
        } else {
            SHDN_temp = 1;
        }

        if (Temp_Control[0] != 0) {
            printf( "\tPID Control enabled\n" );
            // Calculating the Process Values
            ProcessValueAC = (float)( get_value64(TempAC) );
            ProcessValueBD = (float)( get_value64(TempBD) );

            // Vout A - Output to heater BD
            // Temperature measurements from Temp3 and Temp4

            pidAC.setSetPoint((float)get_value64(Set_PointAC));
            pidAC.setProcessValue(ProcessValueAC);
            // Control output
            set_value(HeaterAC,pidAC.compute());

            // Vout B - Output to heater AC
            // Temperature measurements from TempBD and TempAC

            pidBD.setSetPoint((float)get_value64(Set_PointBD));
            pidBD.setProcessValue(ProcessValueBD);
            // Control output
            set_value(HeaterBD,pidBD.compute());
            voutAC = get_value64(HeaterAC);
            voutBD = get_value64(HeaterBD);
        } else {
            printf("\tManual temperature control enabled!\n");
        }

        // Update control output
        if (get_value64(HeaterAC) > MaxPIDout)
            set_value(HeaterAC, MaxPIDout);
        if (get_value64(HeaterBD) > MaxPIDout)
            set_value(HeaterBD, MaxPIDout);
        if (get_value64(HeaterAC) < MinPIDout)
            set_value(HeaterAC, MinPIDout);
        if (get_value64(HeaterBD) < MinPIDout)
            set_value(HeaterBD, MinPIDout);

        voutAC = get_value64(HeaterAC);
        voutBD = get_value64(HeaterBD);

        printf("Writing to DAC:\n");
        printf("\t Vout(heat AC): %f TempAC: %f\n", voutAC, get_value64(TempAC));
        printf("\t Vout(heat BD): %f TempBD: %f\n\n", voutBD, get_value64(TempBD));

        DAC7554_write(CS_dac, DAC_AC_SEL, voutAC);
        DAC7554_write(CS_dac, DAC_BD_SEL, voutBD);

        Thread::wait(int(1000*Rate));
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
    double prev_att1 = get_value64(Att)+1;
    bool attVec1[6];
    RtosTimer Clock_thread(Clk_att);

    while (1) {

        // Ethernet link test
        if (get_eth_link_status())
            LedY = 1;
        else
            LedY = 0;

        // Attenuators set
        if ( prev_att1 != get_value64(Att) ) {
            // Checking and setting attenuators value to fisable values
            set_value(Att,(float)(int(get_value64(Att)*2))/2);
            // Updating previous values
            prev_att1 = get_value64(Att);
            //printf("\nAtt values requested: \n Att: %f \n Att2: %f.\n",prev_att1,prev_att2);
            //printf("\nAtt values updated to: \n Att: %f \n Att2: %f.\n",(float)(int(prev_att1*2))/2,(float)(int(prev_att2*2))/2);
            int2bin6(int(prev_att1*2), attVec1);

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
        printf("New firmare not found\n");
        return;
    }
    printf("Reprogramming...\n");
    strcpy(path,"/local/");
    remove((char*)strcat(path,old_name));
    printf("Reseting...\n");
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
    //Init serial port for info printf
    pc.baud(115200);
    printf("RFFE Control Firmware\n");
    // Setup of variables

    enum bsmp_err err;
    bsmp_server_t *bsmp = bsmp_server_new();
    led_g=0;
    led_r=0;
    // Variables initialization

    // Att
    set_value(Att,30.0);
    set_var(dummy, AttID, true, 8, Att);
    err = bsmp_register_variable(bsmp,&dummy[AttID]);

    // TempBD
    set_value(TempBD,0.0);
    set_var(dummy, TempBDID, false, 8, TempBD);
    err = bsmp_register_variable(bsmp,&dummy[TempBDID]);

    // TempAC
    set_value(TempAC,0.0);
    set_var(dummy, TempACID, false, 8, TempAC);
    err = bsmp_register_variable(bsmp,&dummy[TempACID]);

    // Set_PointBD
    set_value(Set_PointBD,51.5);
    set_var(dummy, Set_PointBDID, true, 8, Set_PointBD);
    err = bsmp_register_variable(bsmp,&dummy[Set_PointBDID]);

    // Set_PointAC
    set_value(Set_PointAC,51.5);
    set_var(dummy, Set_PointACID, true, 8, Set_PointAC);
    err = bsmp_register_variable(bsmp,&dummy[Set_PointACID]);

    // Temp_Control
    Temp_Control[0] = 1;
    set_var(dummy, Temp_ControlID, true, 1, Temp_Control);
    err = bsmp_register_variable(bsmp,&dummy[Temp_ControlID]);

    // HeaterBD
    set_value(HeaterBD,0.0);
    set_var(dummy, HeaterBDID, true, 8, HeaterBD);
    err = bsmp_register_variable(bsmp,&dummy[HeaterBDID]);

    // HeaterAC
    set_value(HeaterAC,0.0);
    set_var(dummy, HeaterACID, true, 8, HeaterAC);
    err = bsmp_register_variable(bsmp,&dummy[HeaterACID]);

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

    //PID_AC Kc parameter
    set_value(PID_AC_Kc, 1.0);
    set_var(dummy, PID_AC_KcID, true, 8, PID_AC_Kc);
    err = bsmp_register_variable(bsmp,&dummy[PID_AC_KcID]);

    //PID_AC tauI parameter
    set_value(PID_AC_tauI, 1.0);
    set_var(dummy, PID_AC_tauIID, true, 8, PID_AC_tauI);
    err = bsmp_register_variable(bsmp,&dummy[PID_AC_tauIID]);

    //PID_AC tauI parameter
    set_value(PID_AC_tauD, 1.0);
    set_var(dummy, PID_AC_tauDID, true, 8, PID_AC_tauD);
    err = bsmp_register_variable(bsmp,&dummy[PID_AC_tauDID]);

    //PID_BD Kc parameter
    set_value(PID_BD_Kc, 1.0);
    set_var(dummy, PID_BD_KcID, true, 8, PID_BD_Kc);
    err = bsmp_register_variable(bsmp,&dummy[PID_BD_KcID]);

    //PID_BD tauI parameter
    set_value(PID_BD_tauI, 1.0);
    set_var(dummy, PID_BD_tauIID, true, 8, PID_BD_tauI);
    err = bsmp_register_variable(bsmp,&dummy[PID_BD_tauIID]);

    //PID_BD tauI parameter
    set_value(PID_BD_tauD, 1.0);
    set_var(dummy, PID_BD_tauDID, true, 8, PID_BD_tauD);
    err = bsmp_register_variable(bsmp,&dummy[PID_BD_tauDID]);

    if (err){
        //Handle Error
    }

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
    //printf("old_name: %s\n",old_name);
    closedir(d);

    // *************************************Threads***************************************

    Thread Switching_Attenuators_thread(Switching_Attenuators_Control);
    printf("Switching_Attenuators_thread\n");

    Thread Temp_Control_thread(Temp_Feedback_Control);
    printf("Temp_Control_thread\n");

    //Thread Serial_Interface_thread(Serial_Interface, NULL, osPriorityNormal, 256 * 4);;
    //printf("\nSerial_USB_thread\n");

    // Ethernet initialization
#if defined(ETH_DHCP)
    EthernetInterface::init(); //Use DHCP
#else
#if defined(ETH_FIXIP)
    EthernetInterface::init(ETH_IP,ETH_MASK,ETH_GATEWAY); //Use  these parameters for static IP
#endif
#endif

    TCPSocketConnection client;
    TCPSocketServer server;

    printf("Trying to establish connection...\n");
    while (EthernetInterface::connect(5000)) {
        //printf("Connection failure\n");
    }
    printf("Connected\n");

    while (true) {
        led3 = 1;

        server.bind(SERVER_PORT);
        server.listen();

        printf("    IP Address is %s\n", EthernetInterface::getIPAddress());
        printf("    MAC Address is %s\n", EthernetInterface::getMACAddress());
        printf("    port %d\n", SERVER_PORT);
        printf(" Wait for new client connection...\n");
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
        //printf("Disconnected: %d\n",eth.disconnect());
        client.close();
        server.close();
        if (!get_eth_link_status()) {
            EthernetInterface::disconnect();
            printf(" Trying to establish connection...\n");
            while (EthernetInterface::connect(5000)) {
                printf(" Connection failure\n");
            }
            printf("Connected\n");
        }
        printf("Client disconnected\n");
        led4 = 0;
        led3 = 0;
    }
}
