/*
 * Testing setup for 6 different temperature and humidity sensors:
 * DHT11, DHT21, DHT22, AM2320, BME280, SI7021, APDS9301, MCP9808.
 * Choice between Adafruit and Sparkfun libraries
 * By: MeetJeStad - Peter Demmer
 * Date: 26 May 2020, version: 0526
 * 
 * Board: Arduino/Genduino Uno
 * Programmer: AVRISP mkII
 * Port: /dev/ttyACM0
 */

#include "printf.h"


#define DEBUG False
#define NODE 4

// Pins and I2C addresses used:
#define PIN_DHT11 4
#define PIN_DHT21 7
#define PIN_DHT22 2
#define ADR_2320 0x5C
#define ADR_BME280 0x77
#define ADR_SI7021 0x40
#define ADR_APDS9301 0x39
#define ADR_MCP9808 0x18

#define strstart(string, s) (! strncmp(string, s, strlen(s)))

// String buffers:
#define DEBUG_LEN 80
#define MSGLEN 64
char rxmsg[MSGLEN+1];
char txmsg[MSGLEN+1];
char debug[DEBUG_LEN];


// void help();
void who();
bool getSerial(char *);
void splitcommands(char*, char*);

//#define LIBS_Adafruit
#define LIBS_Sparkfun

void setup_DHT11(uint8_t);
void setup_DHT21(uint8_t);
void setup_DHT22(uint8_t);
void setup_AM2320(uint8_t);
void setup_BME280(uint8_t);
void setup_SI7021(uint8_t);
void setup_APDS9301(uint8_t);
void setup_MCP9808(uint8_t);

void process(char*);
void process_DHT11(char*);
void process_DHT21(char*);
void process_DHT22(char*);
void process_AM2320(char*);
void process_BME280(char*);
void process_SI7021(char*);
void process_APDS9301(char*);
void process_MCP9808(char*);

bool gestart = false;


#include <Wire.h> 


#include "DHT.h"
DHT dht11(PIN_DHT11, DHT11);
DHT dht21(PIN_DHT21, DHT21);
DHT dht22(PIN_DHT22, DHT22);

#include <Adafruit_AM2320.h>
Adafruit_AM2320 am2320;

#include <Adafruit_BME280.h>
// NOTE: requires Adafruit BME280 library 1.1.0, NOT 2.* !
Adafruit_BME280 bme280;

#ifdef LIBS_Adafruit
#include <Adafruit_Si7021.h>
Adafruit_Si7021 si7021 = Adafruit_Si7021();
#endif
#ifdef LIBS_Sparkfun
#include <SparkFun_Si7021_Breakout_Library.h>
Weather si7021;
#endif

#include <Sparkfun_APDS9301_Library.h>
APDS9301 apds9301;

#include <Adafruit_MCP9808.h>
Adafruit_MCP9808 mcp9808;





 
void setup() {
    Serial.begin(9600);
    printf_begin();

    sprintf(debug, "-id=A%02d\n", NODE);
    Serial.print(debug);

    setup_DHT11(PIN_DHT11);
    setup_DHT21(PIN_DHT21);
    setup_DHT22(PIN_DHT22);
    setup_AM2320(ADR_2320);
    setup_BME280(ADR_BME280);
    setup_SI7021(ADR_SI7021);
    setup_APDS9301(ADR_APDS9301);
    setup_MCP9808(ADR_MCP9808);
}


// Process instructions porvided on the Serial command line:
void loop() {
    if (getSerial(rxmsg)) {   
        if (strlen(rxmsg) > 0) {
            splitcommands(rxmsg, txmsg);
        }
    }
}


void who() {
    sprintf(debug, "-A%02d, 6 temperature and humidity sensors:\n", NODE);
    Serial.print(debug);
    Serial.println("-DHT11, DHT21, DHT22, AM2320, BME280, SI7021,");
    Serial.println("-MCP9808 temperature meter, APDS9301 Lux meter");
    Serial.println("-usage: 'all;' - lists all measurements");
}


// Read commands string from Serial
bool getSerial(char *rxmsg) {
   char c;
   int myI = 0;
   bool myStop = false, sereceived = false;

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


// Split commands string into commands separated by ';':
void splitcommands(char* rxmsg, char* txmsg) {
   // split rxmsg into commands separated by ';' 
   // and writes the response for each command in txmsg
   txmsg[0] = 0;
#if DEBUG
   sprintf(debug, "rxmsg[%d]=\"%s\"\n", strlen(rxmsg), rxmsg);
   Serial.print(debug);
#endif

   uint8_t myC = 0;
   char myCmd[MSGLEN + 1];
   for (uint8_t myI = 0; myI < strlen(rxmsg); myI++) {
      // sprintf(debug, "myI=%d rxmsg[myI]='%c'\n", myI, rxmsg[myI]);
      // Serial.print(debug);
      
      if (rxmsg[myI] == ';' || rxmsg[myI] == 0 || rxmsg[myI] == '\n') {
         myCmd[myC] = 0;
         myC = 0;
         if (myCmd[0] != 0) {
            process(myCmd);   // writes txmsg
         }
      } else {
         myCmd[myC] = rxmsg[myI];
         myC++;
      }
   }
   rxmsg[0] = 0;
     
   if (strlen(txmsg) > 0)
      // write all responses to the Serial 
      Serial.print('-');
      Serial.print(txmsg);
}


// process each command
void process(char *buf) {
#if DEBUG
     sprintf(debug, "cmd[%d]:\"%s\"\n", strlen(buf), buf);
     Serial.print(debug);
#endif

    if (strstart(buf, "DHT11")) {
        process_DHT11(buf);
    } else if (strstart(buf, "DHT21")) {
        process_DHT21(buf);
    } else if (strstart(buf, "DHT22")) {
        process_DHT22(buf);
    } else if (strstart(buf, "AM")) {
        process_AM2320(buf);
    } else if (strstart(buf, "BM")) {
        process_BME280(buf);
    } else if (strstart(buf, "SI")) {
        process_SI7021(buf, 0);
        process_SI7021(buf, 1);
        process_SI7021(buf, 2);
        process_SI7021(buf, 3);
    } else if (strstart(buf, "APDS")) {
        process_APDS9301(buf);
    } else if (strstart(buf, "MCP")) {
        process_MCP9808(buf);
    } else if (strstart(buf, "all")) {
        process_DHT11(buf);
        process_DHT21(buf);
        process_DHT22(buf);
        process_AM2320(buf);
        process_BME280(buf);
        process_SI7021(buf, 0);
        process_SI7021(buf, 1);
        process_SI7021(buf, 2);
        process_SI7021(buf, 3);
        process_APDS9301(buf);
        process_MCP9808(buf);
   } else if (strstart(buf, "help")) {
        who();
   } else if (strstart(buf, "who")) {
        who();
   } else {
        sprintf(&txmsg[strlen(txmsg)], "%s:err;\n", buf);
   }
}




void setup_DHT11(uint8_t pin) {
    dht11.begin();
}


void setup_DHT21(uint8_t pin) {
    dht21.begin();
}


void setup_DHT22(uint8_t pin) {
    dht22.begin();
}


void setup_AM2320(uint8_t addr) {
    Wire.begin();
    am2320.begin();
}


void setup_BME280(uint8_t addr) {
// NOTE: requires Adafruit BME280 library 1.1.0, NOT 2.* !
    if (! bme280.begin()) {  
        sprintf(&txmsg[strlen(txmsg)], "BME280 not found\n");
    }
    sprintf(&txmsg[strlen(txmsg)], "Found BME280 ID=0x%lX\n", bme280.sensorID());
    Serial.print(txmsg); txmsg[0]=0;
}


#ifdef LIBS_Adafruit
void setup_SI7021(uint8_t addr) {
    Wire.begin();
    if (!si7021.begin()) {
        sprintf(&txmsg[strlen(txmsg)], "Did not find Si7021 sensor!\n");
        return;
    }

    sprintf(&txmsg[strlen(txmsg)], "Found model ");
    switch(si7021.getModel()) {
        case SI_Engineering_Samples:
            sprintf(&txmsg[strlen(txmsg)], "SI engineering samples"); break;
        case SI_7013:
            sprintf(&txmsg[strlen(txmsg)], "SI7013"); break;
        case SI_7020:
            sprintf(&txmsg[strlen(txmsg)], "SI7020"); break;
        case SI_7021:
            sprintf(&txmsg[strlen(txmsg)], "SI7021"); break;
        case SI_UNKNOWN:
        default:
            sprintf(&txmsg[strlen(txmsg)], "Unknown");
    }
  
    sprintf(&txmsg[strlen(txmsg)], " Rev(%s)", si7021.getRevision());
    sprintf(&txmsg[strlen(txmsg)], " Serial# %X%X", si7021.sernum_a, si7021.sernum_b);
    // first measurement returns wrong value, ignore:
    si7021.readTemperature();
    si7021.readHumidity();
}
#endif
#ifdef LIBS_Sparkfun
void setup_SI7021(uint8_t addr) {
    Wire.begin();
    si7021.begin();
    // first measurement returns wrong value, ignore:
    si7021.readTemp();
    si7021.getRH();
}
#endif


void setup_APDS9301(uint8_t addr) {
    delay (5);
    Wire.begin(); 
    
    apds9301.begin(addr);
    apds9301.setGain(APDS9301::LOW_GAIN);
    apds9301.setIntegrationTime(APDS9301::INT_TIME_402_MS);
}


void setup_MCP9808(uint8_t addr) {
    Wire.begin();
    
    if (! mcp9808.begin(addr)) {
        Serial.println("Couldn't find MCP9808! Check your connections and verify the address is correct.");
    } else {
        Serial.println("Found MCP9808!");
    }
    
    mcp9808.setResolution(3);
}




void process_DHT11(char* buf) {
    float fTemp = dht11.readTemperature();
    float fHumi = dht11.readHumidity();

// check if returns are valid, if they are NaN (not a number) then something went wrong!
    if (isnan(fTemp) || isnan(fHumi)) {
        sprintf(&txmsg[strlen(txmsg)], "-DHT11:t=null,h=null;");
    } else {
        int16_t iTemp = fTemp * 100.;
        int16_t iHumi = fHumi * 100.;
        sprintf(&txmsg[strlen(txmsg)], "-DHT11:t=%d,h=%d;", iTemp, iHumi);
    }
    Serial.println(txmsg);
    txmsg[0]=0;
}


void process_DHT21(char* buf) {
    float fTemp = dht21.readTemperature();
    float fHumi = dht21.readHumidity();

// check if returns are valid, if they are NaN (not a number) then something went wrong!
    if (isnan(fTemp) || isnan(fHumi)) {
        sprintf(&txmsg[strlen(txmsg)], "-DHT21(DTT21):t=null,h=null;");
    } else {
        int16_t iTemp = fTemp * 100.;
        int16_t iHumi = fHumi * 100.;
        sprintf(&txmsg[strlen(txmsg)], "-DHT21:t=%d,h=%d;", iTemp, iHumi);
    }
    Serial.println(txmsg);
    txmsg[0]=0;
}


void process_DHT22(char* buf) {
    float fTemp = dht22.readTemperature();
    float fHumi = dht22.readHumidity();

// check if returns are valid, if they are NaN (not a number) then something went wrong!
    if (isnan(fTemp) || isnan(fHumi)) {
        sprintf(&txmsg[strlen(txmsg)], "-DHT22:t=null,h=null;");
    } else {
        int16_t iTemp = fTemp * 100.;
        int16_t iHumi = fHumi * 100.;
        sprintf(&txmsg[strlen(txmsg)], "-DHT22:t=%d,h=%d;", iTemp, iHumi);
    }
    Serial.println(txmsg);
    txmsg[0]=0;
}


void process_AM2320(char* buf) {
    float fTemp = am2320.readTemperature();
    float fHumi = am2320.readHumidity();
    
// check if returns are valid, if they are NaN (not a number) then something went wrong!
    if (isnan(fTemp) || isnan(fHumi)) {
        sprintf(&txmsg[strlen(txmsg)], "-AM2320:t=null,h=null;");
    } else {
        uint16_t iTemp = fTemp * 100.;
        uint16_t iHumi = fHumi * 100.;
        sprintf(&txmsg[strlen(txmsg)], "-AM2320:t=%d,h=%d;", iTemp, iHumi);
    }
    Serial.println(txmsg);
    txmsg[0]=0;
}


void process_BME280(char* buf) {
    float fTemp = bme280.readTemperature();
    float fHumi = bme280.readHumidity();
    uint32_t iPres = bme280.readPressure();

    
// check if returns are valid, if they are NaN (not a number) then something went wrong!
    if (isnan(fTemp) || isnan(fHumi) || isnan(iPres)) {
        sprintf(&txmsg[strlen(txmsg)], "-BME280:t=null,h=null,p=null;");
    } else {
        int16_t iTemp = fTemp * 100.;
        int16_t iHumi = fHumi * 100.;
        sprintf(&txmsg[strlen(txmsg)], "-BME280:t=%d,h=%d,p=%ld;", iTemp, iHumi, iPres/100);
    }
    Serial.println(txmsg);
    txmsg[0]=0;
}

    
#ifdef LIBS_Adafruit
void process_SI7021(char* buf) {
    float fTemp = si7021.readTemperature();
    float fHumi = si7021.readHumidity();
        
// check if returns are valid, if they are NaN (not a number) then something went wrong!
    if (isnan(fTemp) || isnan(fHumi)) {
        sprintf(&txmsg[strlen(txmsg)], "-SI7021:t=null,h=null;");
    } else {
        int16_t iTemp = fTemp * 100.;
        int16_t iHumi = fHumi * 100.;
        sprintf(&txmsg[strlen(txmsg)], "-SI7021:t=%d,h=%d;", iTemp, iHumi);
    }
    Serial.println(txmsg);
    txmsg[0]=0;
}
#endif
#ifdef LIBS_Sparkfun
void process_SI7021(char* buf, uint8_t resolution) {
    float fTemp, fHumi;

    si7021.changeResolution(resolution);
    fTemp = si7021.readTemp();
    fHumi = si7021.getRH();
        
    // check if returns are valid, if they are NaN (not a number) then something went wrong!
    if (isnan(fTemp) || isnan(fHumi)) {
        sprintf(&txmsg[strlen(txmsg)], "-SI7021:t=null,h=null;");
    } else {
        int16_t iTemp = fTemp * 100.;
        int16_t iHumi = fHumi * 100.;
        sprintf(&txmsg[strlen(txmsg)], "-SI7021.%d:t=%d,h=%d;", resolution, iTemp, iHumi);
    }
    Serial.println(txmsg);
    txmsg[0]=0;
}
#endif


void process_APDS9301(char* buf) {
    uint16_t iCH0, iCH1, iLux;
    
    iCH0 = (int16_t) apds9301.readCH0Level();
    iCH1 = (int16_t) apds9301.readCH1Level();
    iLux = (int16_t) apds9301.readLuxLevel();
    sprintf(&txmsg[strlen(txmsg)], "-APDS9301:G=lo,ch0=%u,ch1=%u,lux=%d;", iCH0, iCH1, iLux);
    Serial.println(txmsg);
    txmsg[0]=0;
}


void process_MCP9808(char* buf) {
    mcp9808.wake();   // wake up, ready to read, I = 200uA
    uint8_t resolution = mcp9808.getResolution();
    int16_t iTemp = (mcp9808.readTempC() * 100.);
    sprintf(&txmsg[strlen(txmsg)], "-MCP9808:r=%d,t=%d;", resolution, iTemp);
    mcp9808.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 uA, stops temperature sampling
    Serial.println(txmsg);
    txmsg[0]=0;
}


// .-.-.
