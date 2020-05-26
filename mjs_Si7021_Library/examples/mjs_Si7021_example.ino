// Provides access to several si7021 sensors (all with the same I2C address) from Serial
// Maximum 8 sensors can be connected using I2C to e.g. pin 2,3, 4,5, 6,7, 8,9, 10,11, 12,13, A0,A1, A2,A3
// Supports serialid, temperature, humidity, calculated dewpoint, heater write and -read
// Send "help;"  to the Serial for help.
//
// Author: MeetJeStad - Peter Demmer
// Date: 26 May 2020, version: 0526
//
// Tools -> Board: "Arduino Pro Mini"
// Tools -> Processor: "ATmega328P (3.3V, 8MHz)"
// Tools -> Programmer: "AVRisp mkII"




// #define DEBUG

#include <AsyncDelay.h>
#include <SoftWire.h> 
#include <mjs_si7021.h>


// This device's number:
#define NODE 0
// Si7021s i2c address:
#define SENS_ADDR 0x40
// PCB ports in use:
#define MIN_CHILD 1
#define MAX_CHILD 6

#define strstart(string, s) (! strncmp(string, s, strlen(s)))

#define DEBUGLEN 80
#define MSGLEN 64
#define BUFLEN 32

// Serial message buffers:
char rxmsg[MSGLEN+1];
char txmsg[MSGLEN+1];
// debug message buffer:
char debug[DEBUGLEN];
// i2c buffers, reused over all nodes:
char chrxbuf[BUFLEN];
char chtxbuf[BUFLEN];


void Serialbegin(int16_t);
void serialprint(char*);
// read command from Serial port:
bool getSerial(char *);
// command interpreter:
void splitcommands(char*, char*);
void process(char*, char*);
uint8_t discover(bool);
void heatread1(uint8_t, char*);
void heatreads(char*,char*);
void heatwrite(char*,char*);
void help(char*);
void measure1(uint8_t,char*);
void meet(char*, char*);
void sensor(char*, char*);
void sensorid(char*, char*);


// connected i2c devices:
struct { uint8_t SDA; uint8_t SCL; uint32_t serial; } mysws[] = 
    { {2,3,0L}, {4,5,0L}, {6,7,0L}, {8,9,0L}, {10,11,0L}, {12,13,0L}, {A0,A1,0L}, {A2,A3,0L} };
// A2,A3 not working, I don't know why    




void setup() {
    // start serial for output
    Serialbegin(9600);
    
    uint8_t tel = discover(false);
    uint32_t duur = millis();

    sprintf(debug, "-a%02d: 6*Si7021, found %d sensors, took %ld ms, setup done\n", NODE, tel, duur);
    serialprint(debug);
}


// Receive command lines from Serial:
void loop() {
    if (getSerial(rxmsg)) {   
        if (strlen(rxmsg) > 0) {
            splitcommands(rxmsg, txmsg);
        }
    }
   
    delay(100);
}


// process command lines:
bool getSerial(char *rxmsg) {
   char c;
   int myI = 0;
   bool myStop = false, sereceived = false;

   // Read commands string from Serial
   while (Serial.available() && !myStop) {
      c = Serial.read();
      if (c == '\n') {
         myStop = true;
      } else {
         rxmsg[myI++] = c;
      }
      if (myI >= MSGLEN)
         myStop = true;
      sereceived = true;
      delay(10);   // to allow for next char
   }
   rxmsg[myI] = 0;

   return sereceived;
}


// split command line into separate commands separated by ';' :
void splitcommands(char* rxmsg, char* txmsg) {
   // split rxmsg into commands separated by ';' and have them processed

   txmsg[0] = 0;
#ifdef DEBUG
   sprintf(debug, "splitcommands(): rxmsg[%d]=\"%s\"\n", strlen(rxmsg), rxmsg);
   serialprint(debug);
#endif

   int myC = 0;
   char myCmd[MSGLEN + 1];
   for (uint8_t myI = 0; myI < strlen(rxmsg); myI++) {
#ifdef DEBUG2
        sprintf(debug, "myI=%d rxmsg[myI]='%c'\n", myI, rxmsg[myI]);
        serialprint(debug);
#endif
      
      if (rxmsg[myI] == ';' || rxmsg[myI] == 0 || rxmsg[myI] == '\n') {
         myCmd[myC] = 0;
         myC = 0;
         if (myCmd[0] != 0) {
            process(myCmd, txmsg);   // writes and serialprints txmsg
         }
      } else {
         myCmd[myC] = rxmsg[myI];
         myC++;
      }
   }

   rxmsg[0] = 0;
}


// process a single command:
void process(char *rxmsg, char* txmsg) {
#ifdef DEBUG
    sprintf(debug, "process(): rxmsg=%s.\n", rxmsg);
    serialprint(debug);
#endif
    
    if (strstart(rxmsg, "di")) {
        discover(true);
    } else if (strstart(rxmsg, "he")) {
        help(rxmsg,txmsg);
    } else if (strstart(rxmsg, "hr")) {
        heatreads(rxmsg,txmsg);
    } else if (strstart(rxmsg, "hw")) {
        heatwrite(rxmsg,txmsg);
    } else if (strstart(rxmsg, "me")) {
        meet(rxmsg,txmsg);
    } else if (strstart(rxmsg, "wh")) {
        who();
    } else {
        sprintf(txmsg, "%s:err\n", rxmsg);
        serialprint(txmsg);
    }
}




// help commands

void who() {
    sprintf(txmsg, "-a%02d, 6*Si7021 sensor\n", NODE);
    serialprint(txmsg);
}


void help(char* rxmsg, char *txmsg) {
    sprintf(txmsg, "di;  discover\n"); 
    serialprint(txmsg); 
    sprintf(txmsg, "he;  help\n"); 
    serialprint(txmsg); 
    sprintf(txmsg, "hr;  heater read\n"); 
    serialprint(txmsg); 
    sprintf(txmsg, "hw;  heater write\n"); 
    serialprint(txmsg); 
    sprintf(txmsg, "me*; measure all\n"); 
    serialprint(txmsg); 
    sprintf(txmsg, "who; device function\n"); 
    serialprint(txmsg); 
}




// functional procedures

// discover which Si7021 is connected to each I2C port:
uint8_t discover(bool verbose) {
    // discover which sensors are present
    uint8_t tel = 0;
    
    // initialize i2c:
    for (uint8_t iSw = MIN_CHILD; iSw <= MAX_CHILD; iSw++) {
        if (mysws[iSw].SDA > 0) {
            SoftWire sw = SoftWire(mysws[iSw].SDA, mysws[iSw].SCL);
            sw.begin();
            sw.setRxBuffer(chrxbuf,BUFLEN);
            sw.setTxBuffer(chtxbuf,BUFLEN);
            sw.setTimeout_ms(40);
            
            // Signal a sensor read:
            sw.beginTransmission(SENS_ADDR);
            if (sw.endTransmission () != 0) {
                sprintf(debug, "\rat %d:0x%X empty\n", iSw, SENS_ADDR);
                mysws[iSw].SDA = 0;   // disable
                if (verbose) serialprint(debug);
            } else { 
                mjs_si7021 si7021(&sw);
                si7021.begin();
                mysws[iSw].serial = si7021.getSerial();
                
                sprintf(debug, "\rat %d:0x%X found %s serialid=0x%08lX\n", 
                    iSw, SENS_ADDR, si7021.typenaam, mysws[iSw].serial);
                if (verbose) serialprint(debug);
                tel++;
            }
        }
    }  
    
    return(tel);
}


// read 1 Si7021:
void heatread1(uint8_t iSw, char* txbuf) {
    // read heater setting of 1 sensor
    if (mysws[iSw].SDA != 0) {
        SoftWire sw = SoftWire(mysws[iSw].SDA, mysws[iSw].SCL);
        sw.begin();
        sw.setRxBuffer(chrxbuf,BUFLEN);
        sw.setTxBuffer(chtxbuf,BUFLEN);
        sw.setTimeout_ms(40);
        // Signal a sensor read:
        sw.beginTransmission(SENS_ADDR);
             
        mjs_si7021 si7021(&sw);
        si7021.begin();
        uint8_t check = si7021.getHeater()+1;
        // uint8_t sta = si7021.staHeater();
        if (! si7021.staHeater()) check = 0;
                 
        sprintf(debug, "-%d:H=%d\n", iSw, check);
        serialprint(debug);
    } 
}


// read all Si7021s:
void heatreads(char* rxbuf, char* txbuf) {
    // read heater settings of 1 or more sensors
    uint8_t iSw;
    iSw = rxbuf[2] - '0';
#ifdef DEBUG
    sprintf(debug, "heatreads(): rxbuf='%s' iSw=%d\n", rxbuf, iSw);
    Serial.print(debug);
#endif

    if ((rxbuf[2] == '*') | (rxbuf[2] == 0)) {
        for(uint8_t iSw = MIN_CHILD; iSw <= MAX_CHILD; iSw++) {
            heatread1(iSw, txbuf); 
        }
    } else if (iSw < MIN_CHILD || iSw > MAX_CHILD) {
        sprintf(debug, "err:%s: %d out of range\n", rxbuf, iSw);
        serialprint(debug);
    } else {
        heatread1(iSw, txbuf); 
    }
}


// set heater:
void heatwrite(char* rxbuf, char* txbuf) {
    // set the heater of 1 sensor
    uint8_t iSw;
    iSw = rxbuf[2] - '0';
#ifdef DEBUG
    sprintf(debug, "heatwrite(): rxbuf='%s' iSw=%d\n", rxbuf, iSw);
    Serial.print(debug);
#endif

    if (iSw < MIN_CHILD || iSw > MAX_CHILD || rxbuf[3] != '=') {
        sprintf(debug, "err:%s: syntax or %d out of range\n", rxbuf, iSw);
        serialprint(debug);
    } else {
        if (mysws[iSw].SDA != 0) {
            SoftWire sw = SoftWire(mysws[iSw].SDA, mysws[iSw].SCL);
            sw.begin();
            sw.setRxBuffer(chrxbuf,BUFLEN);
            sw.setTxBuffer(chtxbuf,BUFLEN);
            sw.setTimeout_ms(40);
            // Signal a sensor read:
            sw.beginTransmission(SENS_ADDR);
             
            mjs_si7021 si7021(&sw);
            si7021.begin();
            int16_t heater;
            int16_t nr = sscanf(rxbuf+4, "%d", &heater);  
            // uint8_t check;
            // uint8_t stat;
            if (nr > 0) {   // integer found
                if (heater > 0) {
                    si7021.heaterOn();
                    si7021.setHeater(heater-1);
#ifdef DEBUG
                    uint8_t check = si7021.getHeater()+1;
                    uint8_t stat = si7021.staHeater();
                    sprintf(debug, "heatwrite(): heater set=%d check=%d stat=%d\n", 
                        heater, check, stat);
                    serialprint(debug);
#endif  
                } else {
                    si7021.setHeater(0);
                    si7021.heaterOff();
#ifdef DEBUG
                    uint8_t check = si7021.getHeater()+1;
                    uint8_t stat = si7021.staHeater();
                    sprintf(debug, "heatwrite(): check=%d stat=%d\n", check, stat);
                    serialprint(debug);
#endif  
                }
            }
                         
            sprintf(debug, "-%d:H=%d\n", iSw, (si7021.getHeater()+1)*si7021.staHeater());
            serialprint(debug);
        }
    } 
}


// measure several devices:
void meet(char* rxmsg, char* txmsg) {
#ifdef DEBUG
    sprintf(debug, "meet(): rxmsg=%s.\n", rxmsg);
    serialprint(debug);
#endif

    uint8_t iSw;
    // response to "me*" or "me": show measurements of all devices
    if ((rxmsg[2] == '*') | (rxmsg[2] == 0)) {
        for (iSw = MIN_CHILD; iSw <= MAX_CHILD; iSw++) {
            measure1(iSw,txmsg);
            serialprint(txmsg);
        }
    } else {
        // response to "me4": show measurements of device 4
        iSw = rxmsg[2] - '0';
        if ((iSw < MIN_CHILD) || (iSw > MAX_CHILD)) {
            sprintf(txmsg, "%s:%d:err\n", rxmsg, iSw);
        } else {
            measure1(iSw,txmsg);   // overwrites previous measurements on the same line
        }
        serialprint(txmsg);
    }
}


// measure 1 device:
void measure1(uint8_t iSw, char* txmsg) {
    if (mysws[iSw].SDA == 0) {
        sprintf(txmsg, "-%d:err", iSw);
    } else {
        SoftWire sw = SoftWire(mysws[iSw].SDA, mysws[iSw].SCL);
        sw.begin();
        sw.setRxBuffer(chrxbuf,BUFLEN);
        sw.setTxBuffer(chtxbuf,BUFLEN);
        sw.setTimeout_ms(40);
            
        // Signal a sensor read:
        sw.beginTransmission(SENS_ADDR);
        if (sw.endTransmission () == 0) {
            mjs_si7021 si7021(&sw);
            si7021.begin();
            uint32_t myserial = si7021.getSerial();
            
            float temp = si7021.getTemp();
            
            float humi = si7021.getHumi();
            
            char fmtDewp[16];
            if (humi < 0) {
                strcpy(fmtDewp, "null");
            } else {
                // Dewpoint formula from https://1728.org/dewpoint.htm
                float N = (log(humi/100.)+((17.27*temp)/(237.3+temp)))/17.27;
                int16_t iDew = (int) ((237.3*N)/(1.-N)*100.);
                if (iDew >= 0) {
                    sprintf(fmtDewp, "+%d.%02d", iDew/100, iDew%100);
                } else {
                    sprintf(fmtDewp, "-%d.%02d", -iDew/100, (-iDew)%100);
                }
            }            
               
            uint8_t Heat = si7021.getHeater()+1;
            // if (! si7021.staHeater()) Heat = 0;
            uint8_t Sta = si7021.staHeater();
            
            sprintf(txmsg, "-%d:s=0x%08lX;t=%s;h=%s;d=%s;H=%d\n", 
                iSw, myserial, si7021.fmtTemp, si7021.fmtHumi, fmtDewp, Sta*Heat);
        }
    }
}


// Control Serial port

void Serialbegin(int16_t bps) {
// #ifdef CONSOLE
    Serial.begin(bps);
// #endif
}
void serialprint(char *debug) {
// #ifdef CONSOLE
    Serial.print(debug);
// #endif
}


// .-.-.
