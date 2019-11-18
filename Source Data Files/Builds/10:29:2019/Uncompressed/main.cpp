#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <DNSServer.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <FS.h>
#include <ArduinoJson.h>
#include <DoubleResetDetector.h>
#include "ESPAsyncTCP.h"
#include "ESPAsyncWebServer.h"

//===> GLOBAL <=================================================================

//===> constants <--------------------------------------------------------------
#define DRD_TIMEOUT 10
#define DRD_ADDRESS 0
#define fanPin D8
#define mosfetPinOne D7
#define mosfetPinTwo D6
#define mosfetPinThree D5
#define mosfetPinLid D0
#define LEDpin D4
#define THERMISTORNOMINAL 100000
#define TEMPERATURENOMINAL 25
#define NUMSAMPLES 5
#define BCOEFFICIENT 3950
#define SERIESRESISTOR 100000

const byte DNS_PORT = 53;

//===> variables <--------------------------------------------------------------

DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws"); // access at ws://[esp ip]/ws
Adafruit_ADS1115 ads;
DNSServer dnsServer;

boolean isPaused = false;
String runTime = "0";
int websockMillis=500;
String JSONtxt;
unsigned long wait001=0UL;

int samples[NUMSAMPLES];
float tempone;
float temptwo;
float tempthree;
float templid;
float steinhart;
float Input;

double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
double Setpoint3, Input3, Output3;
double Setpointlid, Inputlid, Outputlid;
double Setpointfan, Inputfan, Outputfan;

bool resetOutput1 = true;
bool resetOutput2 = true;
bool resetOutput3 = true;

//Normal PID
//Specify the links and initial tuning parameters
//double Kp=600, Ki=0, Kd=.00001;
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
double aggKp=1800, aggKi=1, aggKd=1;
double consKp=600, consKi=60, consKd=500;
double alidKp=1000, alidKi=.01, alidKd=.5;
double lidKp=1000, lidKi=40, lidKd=100;
double aggKpfan=400, aggKifan=.01, aggKdfan=.0001;
double consKpfan=100, consKifan=.1, consKdfan=.2;
//Specify the links and initial tuning parameters
PID heatPID1(&Input1, &Output1, &Setpoint1, consKp, consKi, consKd, P_ON_E, DIRECT);
PID heatPID2(&Input2, &Output2, &Setpoint2, consKp, consKi, consKd, P_ON_E, DIRECT);
PID heatPID3(&Input3, &Output3, &Setpoint3, consKp, consKi, consKd, P_ON_E, DIRECT);
PID lidPID(&Inputlid, &Outputlid, &Setpointlid, lidKp, lidKi, lidKd, P_ON_E, DIRECT);
PID fanPID(&Inputfan, &Outputfan, &Setpointfan, consKpfan, consKifan, consKdfan, P_ON_E, REVERSE);


boolean PCRon = false;
boolean blockOn = false;
boolean preheatLid = false;
boolean preheatBlock = false;

float blockTemp;

float preheatLidTemp;
float preheatBlockTemp;

boolean timerStarted = false;
float tempreturn;
float tempset;

float tempthreshold = 2;
float setLidTemp = 0;
float totalTime;
float initDenatureTemp;
float initDenatureTime;
float denatureTemp;
float denatureTime;
float annealTemp;
float annealTempLow;
float annealTempHigh;
float annealTime;
float extendTemp;
float extendTime;
float finalExtendTemp;
float finalExtendTime;
float cycleNum;
int cycleNumInt;
byte cycleCount = 1;
float heatLow = 0;
float heatHigh = 0;
float heatMid;
unsigned long timerSerial;
float initDenatureTimeSec;
float TDannealTimeSec;
float TDextendTimeSec;
float TDdenatureTimeSec;
float annealTimeSec;
float extendTimeSec;
float denatureTimeSec;
float finalExtendTimeSec;
float rampTimeMin;
float heatBlockTimeMin;
boolean denatureStep = true;
boolean annealStep = false;
boolean extendStep = false;
String cycleState = "Heating Lid";
unsigned long previousMillis;
unsigned long currentMillis;

byte programState = 0;
int  programType = 0;

float TDcycleNum;
float TDdenatureTemp;
float TDdenatureTime;
float TDStartTemp;
float TDEndTemp;
float TDLowStartTemp;
float TDHighStartTemp;
float TDLowEndTemp;
float TDHighEndTemp;
float TDannealTime;
float TDextendTemp;
float TDextendTime;
float startRampTemp;
float endRampTemp;
float lowstartRampTemp;
float highstartRampTemp;
float lowendRampTemp;
float highendRampTemp;
float rampTime;
float heatBlockTemp;
float lowheatBlockTemp;
float highheatBlockTemp;
float heatBlockTime;
String progState = "PCR Ready";
String data;
String programName;
String programNameSave;
String autostartprogramName = "";
String autostartName;
String autostartProgram;
boolean updatestart = false;
boolean autostart = false;
boolean quietfan = false;
String configdata;
String softAP_ssid;
String softAP_password;
String userpsw = "";
int chipIDint;
String propcrChip;
IPAddress apIP;
IPAddress netMsk(255, 255, 255, 0);
boolean apon = true;
String userssid;
String userpass = "";
boolean connectwifi = false;
boolean openmdns = true;
boolean APcatch = false;
boolean APtimernew = true;
unsigned long APtime;
boolean doubleReset = true;
unsigned long resetTime;
boolean resetOn = true;
String connected = "false";
String localIPaddress = "";
String chipIDstring;

const long serialinterval = 250;
unsigned long previousserialMillis = 0;

const long WSinterval = 1000;
unsigned long previousWSMillis = 0;

//===> functions <--------------------------------------------------------------

static AsyncClient * aClient = NULL;

void runAsyncClient(){
        //Serial.println("async client");
  if(aClient)//client already exists
    return;

  aClient = new AsyncClient();
  if(!aClient)//could not allocate client
    return;

  aClient->onError([](void * arg, AsyncClient * client, err_t error){
    //Serial.println("Connect Error");
    aClient = NULL;
    delete client;
  }, NULL);

  aClient->onConnect([](void * arg, AsyncClient * client){
    //Serial.println("Connected");
    aClient->onError(NULL, NULL);

    client->onDisconnect([](void * arg, AsyncClient * c){
      //Serial.println("Disconnected");
      aClient = NULL;
      delete c;
    }, NULL);

    client->onData([](void * arg, AsyncClient * c, void * data, size_t len){
      //Serial.print("\r\nData: ");
      //Serial.println(len);
      uint8_t * d = (uint8_t*)data;
      for(size_t i=0; i<len;i++)
        Serial.write(d[i]);
    }, NULL);

    //send the request
    client->write("GET / HTTP/1.0\r\nHost: www.propcr.com\r\n\r\n");
  }, NULL);

  if(!aClient->connect("www.propcr.com", 80)){
    //Serial.println("Connect Fail");
    AsyncClient * client = aClient;
    aClient = NULL;
    delete client;
  }
}

void setupAP(){
        //Serial.println("Configuring access point...");
        //Serial.println(ESP.getChipId());
        String chipID = String(ESP.getChipId()).substring(3,6);
        //Serial.println(chipID);
        chipIDint = chipID.toInt();
        while (chipIDint > 255 || chipIDint < 100) {
                if (chipIDint < 100) {
                  chipIDint = chipIDint + 500;
                }
                chipIDint = chipIDint - 155;
                //Serial.println(chipIDint);
        }
        IPAddress apIP(1, 1, 1, chipIDint);
        chipIDstring = String(chipIDint);
        softAP_ssid = "proPCR." + chipIDstring;
        propcrChip = "propcr" + chipIDstring;
        WiFi.hostname(propcrChip);
        //Serial.print("softAP_ssid: ");
        //Serial.println(softAP_ssid);
        if (userpsw != "") {
                softAP_password = userpsw;
        }else{
                softAP_password = "buildthefuture";
        }
        if (WiFi.status() != 3){
          WiFi.mode(WIFI_AP);
        }
        WiFi.softAPConfig(apIP, apIP, netMsk);
        WiFi.softAP(softAP_ssid.c_str(), softAP_password.c_str());
        delay(500); // Needed delay so IP address doesn't blank
        //Serial.print("AP IP address: ");
        //Serial.println(WiFi.softAPIP());

}

String toStringIp(IPAddress ip) {
        String res = "";
        for (int i = 0; i < 3; i++) {
                res += String((ip >> (8 * i)) & 0xFF) + ".";
        }
        res += String(((ip >> 8 * 3)) & 0xFF);
        return res;
}

void startWifi(){

        //Serial.println("function: startWifi()");

    //     int n = WiFi.scanNetworks();
    //     //Serial.print(n);
    // //Serial.println(" networks found");
    // for (int i = 0; i < n; ++i)
    // {
    //   // Print SSID and RSSI for each network found
    //   //Serial.print(i + 1);
    //   //Serial.print(": ");
    //   //Serial.print(WiFi.SSID(i));
    //   //Serial.print(" (");
    //   //Serial.print(WiFi.RSSI(i));
    //   //Serial.print(")");
    //   //Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE)?" ":"*");
    //   delay(10);
    // }

        WiFi.mode(WIFI_STA);
        //Serial.print("userssid: ");
        //Serial.println(userssid);
        //Serial.print("userpass: ");
        //Serial.println(userpass);
        if (userpass == "") {
                WiFi.begin(userssid.c_str());
        }else{
                WiFi.begin(userssid.c_str(), userpass.c_str());
        }
        delay(500);
        unsigned long startedAt = millis();
          while(millis() - startedAt < 10000)
          {
              delay(100);
              if (WiFi.status()==WL_CONNECTED) {
                APcatch = true;
                //Serial.println("wifi connected");
      			connected = "true";
            localIPaddress = toStringIp(WiFi.localIP());
            //Serial.print("local ip:");
            //Serial.println(WiFi.localIP());
            //Serial.println(localIPaddress);
      		}
          }

        //Serial.print("Wifi status: ");
        //Serial.println(WiFi.status());
        if (WiFi.status() == 3){
                  if (apon) {
                          WiFi.mode(WIFI_AP_STA);
                          delay(500);
                          //Serial.println("connected ap sta");

                  }
                        //Serial.print("connected localIP: ");
                        //Serial.println(WiFi.localIP());
        }
        if (apon) {
                setupAP();
        }
}





float readTherm(int therm){

        //uint8_t i;
        float average;
        /*
           // take N samples in a row, with a slight delay
           for (i=0; i< NUMSAMPLES; i++) {
           samples[i] = therm;
           delay(5);
           }

           // average all the samples out
           average = 0;
           for (i=0; i< NUMSAMPLES; i++) {
             average += samples[i];
           }
           average /= NUMSAMPLES;
           ////Serial.print("Average analog reading: ");
           ////Serial.println(average);
         */
        average = therm;
        // convert the value to resistance
        average = 25500 / average - 1;
        average = SERIESRESISTOR / average;
        ////Serial.print("Thermistor ");
        ////Serial.print(therm);
        ////Serial.print(" resistance conversion: ");
        ////Serial.println(average);


        steinhart = average / THERMISTORNOMINAL;    // (R/Ro)
        steinhart = log(steinhart);               // ln(R/Ro)
        steinhart /= BCOEFFICIENT;                // 1/B * ln(R/Ro)
        steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);    // + (1/To)
        steinhart = 1.0 / steinhart;              // Invert
        steinhart -= 273.15;                      // convert to C
        /*
           //Serial.print("Thermistor ");
           //Serial.print(therm);
           //Serial.print(" temp celcius: ");
           //Serial.println(steinhart);
         */
        return steinhart;
}

void readThermistors(){
        int16_t adc0, adc1, adc2, adc3;
        adc0 = ads.readADC_SingleEnded(0);
        adc1 = ads.readADC_SingleEnded(1);
        adc2 = ads.readADC_SingleEnded(2);
        adc3 = ads.readADC_SingleEnded(3);

        templid = readTherm(adc0);
        tempone = readTherm(adc1);
        temptwo = readTherm(adc2);
        tempthree = readTherm(adc3);
}

void resetPCR() {
        cycleState = "Heating Lid";
        isPaused = false;
        // programName = "";
        Setpointlid = 0;
        heatLow = 0;
        heatHigh = 0;
        programType = 1;
        setLidTemp = 0;
        initDenatureTemp = 0;
        initDenatureTime = 0;
        denatureTemp = 0;
        denatureTime = 0;
        annealTemp = 0;
        annealTempLow = 0;
        annealTempHigh = 0;
        annealTime = 0;
        extendTemp = 0;
        extendTime = 0;
        finalExtendTemp = 0;
        finalExtendTime = 0;
        cycleNum = 0;
        cycleNumInt = 0;
        cycleCount = 1;
        TDcycleNum = 0;
        TDdenatureTemp = 0;
        TDdenatureTime = 0;
        TDdenatureTimeSec = 0;
        TDannealTime = 0;
        TDannealTimeSec = 0;
        TDextendTemp = 0;
        TDextendTime = 0;
        TDextendTimeSec = 0;
        TDStartTemp = 0;
        TDEndTemp = 0;
        TDLowStartTemp = 0;
        TDHighStartTemp = 0;
        TDLowEndTemp = 0;
        TDHighEndTemp = 0;
        rampTime = 0;
        rampTimeMin = 0;
        startRampTemp = 0;
        endRampTemp = 0;
        lowstartRampTemp = 0;
        highstartRampTemp = 0;
        lowendRampTemp = 0;
        highendRampTemp = 0;
        heatBlockTime = 0;
        heatBlockTimeMin = 0;
        heatBlockTemp = 0;
        lowheatBlockTemp = 0;
        highheatBlockTemp = 0;
        PCRon = false;
        programState = 0;
        blockOn = false;
        timerStarted = false;
        totalTime = 0;
        denatureStep = true;
        annealStep = false;
        extendStep = false;
        preheatBlock = false;
        preheatLid = false;
}

void runPCR(float &low, float &high){

        switch (programState) {

        //Lid Wait
        case 0:
                cycleState = "Heating Lid";
                if (Inputlid >= setLidTemp - tempthreshold || setLidTemp == 0) {
                       
                        programState = 1;
                        blockOn = true;
                        cycleState = "Initial Denature";
                }
                break;

        //Initial Denature
        case 1:
                cycleState = "Initial Denature";
                low = initDenatureTemp;
                high = initDenatureTemp;
                totalTime = initDenatureTime;
                if (Input1 >= low - tempthreshold) {
                        Serial.print("Timer: ");
                        Serial.print(timerSerial);
                        Serial.print(" of ");
                        Serial.println(initDenatureTimeSec);
                        currentMillis = millis();
                        if (timerStarted == false) {
                                previousMillis = millis();
                                timerStarted = true;
                        }
                        if (timerStarted == true && currentMillis - previousMillis > initDenatureTime) {
                                timerStarted = false;
                                if (programType == 3 || programType == 4) {
                                  programState = 2;
                                  cycleState = "Touchdown Phase Denature Step";
                                  cycleNumInt = (int) TDcycleNum;
                                }else{
                                  programState = 3;
                                  cycleState = "Denature Step";
                                  cycleNumInt = (int) cycleNum;
                                }
                              }
                }
                // Serial.print("timerStarted: ");
                        // Serial.println(timerStarted);
                break;

        //Touchdown Cycles
        case 2:
        if (cycleCount <= TDcycleNum) {
                //Serial.print("Touchdown Cycle ");
                //Serial.print(cycleCount);
                //Serial.print(" of ");
                //Serial.println(TDcycleNum);

                //Denature step
                if (denatureStep == true) {
                  totalTime = TDdenatureTime;
                        //Serial.println("TD Denature Step");

                        low = TDdenatureTemp;
                        high = TDdenatureTemp;

                        if (Input1 >= low - tempthreshold && Input1 <= low + tempthreshold) {
                                //Serial.print("Timer: ");
                                //Serial.print(timerSerial);
                                //Serial.print(" of ");
                                //Serial.println(TDdenatureTimeSec);
                                currentMillis = millis();
                                if (timerStarted == false) {
                                        previousMillis = millis();
                                        timerStarted = true;
                                }
                        }
                        if (timerStarted == true && currentMillis - previousMillis > TDdenatureTime) {
                                denatureStep = false;
                                annealStep = true;
                                timerStarted = false;
                                cycleState = "Touchdown Phase Anneal Step";
                        }
                }

                //Anneal step
                if (annealStep == true) {
                    totalTime = TDannealTime;
                        //Serial.println("Touchdown Phase Anneal Step");

                        if (programType == 3) {
                          float tempStep;
                          float TDannealTemp;
                          tempStep = TDStartTemp - TDEndTemp;
                          tempStep = abs(tempStep);
                          tempStep /= TDcycleNum;
                          if (TDStartTemp > TDEndTemp) {
                            TDannealTemp = TDStartTemp;
                            TDannealTemp += tempStep;
                            tempStep *= cycleCount;
                            TDannealTemp -= tempStep;
                          }else{
                            TDannealTemp = TDStartTemp;
                            TDannealTemp -= tempStep;
                            tempStep *= cycleCount;
                            TDannealTemp += tempStep;
                          }

                          low = TDannealTemp;
                          high = TDannealTemp;
                        }
                        if (programType == 4) {
                          float lowtempStep;
                          float hightempStep;
                          float lowTDannealTemp;
                          float highTDannealTemp;
                          lowtempStep = TDLowStartTemp - TDLowEndTemp;
                          lowtempStep = abs(lowtempStep);
                          lowtempStep /= TDcycleNum;
                          hightempStep = TDHighStartTemp - TDHighEndTemp;
                          hightempStep = abs(hightempStep);
                          hightempStep /= TDcycleNum;
                          if (TDLowStartTemp > TDLowEndTemp) {
                            lowTDannealTemp = TDLowStartTemp;
                            lowTDannealTemp += lowtempStep;
                            lowtempStep *= cycleCount;
                            lowTDannealTemp -= lowtempStep;
                          }else{
                            lowTDannealTemp = TDLowStartTemp;
                            lowTDannealTemp -= lowtempStep;
                            lowtempStep *= cycleCount;
                            lowTDannealTemp += lowtempStep;
                          }
                          if (TDHighStartTemp > TDHighEndTemp) {
                            highTDannealTemp = TDHighStartTemp;
                            highTDannealTemp += hightempStep;
                            hightempStep *= cycleCount;
                            highTDannealTemp -= hightempStep;
                          }else{
                            highTDannealTemp = TDHighStartTemp;
                            highTDannealTemp -= hightempStep;
                            hightempStep *= cycleCount;
                            highTDannealTemp += hightempStep;
                          }
                        low = lowTDannealTemp;
                        high = highTDannealTemp;
                        }

                        if (Input1 >= low - tempthreshold && Input1 <= low + tempthreshold) {
                                //Serial.print("Timer: ");
                                //Serial.print(timerSerial);
                                //Serial.print(" of ");
                                //Serial.println(TDannealTimeSec);
                                currentMillis = millis();

                                if (timerStarted == false) {
                                        previousMillis = millis();
                                        timerStarted = true;
                                }
                        }
                        if (timerStarted == true && currentMillis - previousMillis > TDannealTime) {
                                //Serial.println("Timer Stopped");
                                annealStep = false;
                                extendStep = true;
                                timerStarted = false;
                                cycleState = "Touchdown Phase Extend Step";
                        }
                }
                //Extend step
                if (extendStep == true) {
                  totalTime = TDextendTime;
                        //Serial.println("TD Extend Step");

                        low = TDextendTemp;
                        high = TDextendTemp;

                        if (Input1 >= low - tempthreshold && Input1 <= low + tempthreshold) {
                                //Serial.print("Timer: ");
                                //Serial.print(timerSerial);
                                //Serial.print(" of ");
                                //Serial.println(TDextendTimeSec);
                                currentMillis = millis();

                                if (timerStarted == false) {
                                        previousMillis = millis();
                                        timerStarted = true;
                                }
                        }
                        if (timerStarted == true && currentMillis - previousMillis > TDextendTime) {
                                extendStep = false;
                                denatureStep = true;
                                timerStarted = false;
                                cycleState = "Touchdown Phase Denature Step";
                                cycleCount++;
                        }
                }
        }
        if (cycleCount >= TDcycleNum) {
                cycleCount = 1;
                programState = 3;
                cycleState = "Denature Step";
                cycleNumInt = (int) cycleNum;
        }
        break;

        case 3:
                if (cycleCount <= cycleNum) {
                        // //Serial.print("Cycle ");
                        // //Serial.print(cycleCount);
                        // //Serial.print(" of ");
                        // //Serial.println(cycleNum);

                        //Denature step
                        if (denatureStep == true) {
                          totalTime = denatureTime;
                                // //Serial.println("Denature Step");

                                low = denatureTemp;
                                high = denatureTemp;

                                if (Input1 >= low - tempthreshold && Input1 <= low + tempthreshold) {
                                        // //Serial.print("Timer: ");
                                        // //Serial.print(timerSerial);
                                        // //Serial.print(" of ");
                                        // //Serial.println(denatureTimeSec);
                                        currentMillis = millis();
                                        if (timerStarted == false) {
                                                previousMillis = millis();
                                                timerStarted = true;
                                        }
                                }
                                if (timerStarted == true && currentMillis - previousMillis > denatureTime) {
                                        denatureStep = false;
                                        annealStep = true;
                                        timerStarted = false;
                                        cycleState = "Anneal Step";
                                }
                        }

                        //Anneal step
                        if (annealStep == true) {
                          totalTime = annealTime;
                                // //Serial.println("Anneal Step");

                                if (programType == 1 || programType == 3) {
                                  low = annealTemp;
                                  high = annealTemp;
                                }
                                if (programType == 2 || programType == 4) {
                                  low = annealTempLow;
                                  high = annealTempHigh;
                                }


                                if (Input1 >= low - tempthreshold && Input1 <= low + tempthreshold) {
                                        // //Serial.print("Timer: ");
                                        // //Serial.print(timerSerial);
                                        // //Serial.print(" of ");
                                        // //Serial.println(annealTimeSec);
                                        currentMillis = millis();

                                        if (timerStarted == false) {
                                                previousMillis = millis();
                                                timerStarted = true;
                                        }
                                }
                                if (timerStarted == true && currentMillis - previousMillis > annealTime) {
                                        // //Serial.println("Timer Stopped");
                                        annealStep = false;
                                        extendStep = true;
                                        timerStarted = false;
                                        cycleState = "Extend Step";
                                }
                        }
                        //Extend step
                        if (extendStep == true) {
                          totalTime = extendTime;
                                // //Serial.println("Extend Step");

                                low = extendTemp;
                                high = extendTemp;

                                if (Input1 >= low - tempthreshold && Input1 <= low + tempthreshold) {
                                        // //Serial.print("Timer: ");
                                        // //Serial.print(timerSerial);
                                        // //Serial.print(" of ");
                                        // //Serial.println(extendTimeSec);
                                        currentMillis = millis();

                                        if (timerStarted == false) {
                                                previousMillis = millis();
                                                timerStarted = true;
                                        }
                                }
                                if (timerStarted == true && currentMillis - previousMillis > extendTime) {
                                        extendStep = false;
                                        denatureStep = true;
                                        timerStarted = false;
                                        cycleState = "Denature Step";
                                        cycleCount++;
                                }
                        }
                }
                if (cycleCount >= cycleNum) {
                        programState = 4;
                        cycleState = "Final Extend Step";
                }
                break;

        //Final extend
        case 4:
          totalTime = finalExtendTime;
                //Serial.println("Final Extend Step");

                low = finalExtendTemp;
                high = finalExtendTemp;

                if (Input1 >= low - tempthreshold && Input1 <= low + tempthreshold) {
                        //Serial.print("Timer: ");
                        //Serial.print(timerSerial);
                        //Serial.print(" of ");
                        //Serial.println(finalExtendTimeSec);
                        currentMillis = millis();
                        if (timerStarted == false) {
                                previousMillis = millis();
                                timerStarted = true;
                        }
                        if (timerStarted == true && currentMillis - previousMillis > finalExtendTime) {
                                timerStarted = false;
                                programState = 5;
                                cycleState = "Program Done!";
                        }
                }

                break;

        //Program done
        case 5:
                progState = "Finished";
                resetPCR();
                //Serial.println("Program done!");
                break;
        }
}

void runRamp(float &low, float &high){

        switch (programState) {

        //Lid Wait
        case 0:
                cycleState = "Heating Lid";
                if (Inputlid >= setLidTemp - tempthreshold || setLidTemp == 0) {
                        programState = 1;
                        blockOn = true;
                        cycleState = "Heating Block";
                }
                break;

        
        case 1:
                
                //Serial.println("Heating to Start Temp");
                if (programType == 5) {
                  low = startRampTemp;
                  high = startRampTemp;
                }
                if (programType == 6) {
                  low = lowstartRampTemp;
                  high = highstartRampTemp;
                }
                //Serial.print("Input3: ");
                //Serial.println(Input3);

                if (Input3 >= high - 1) {
                  //Serial.println("Running Ramp");

                                cycleState = "Running Ramp";
                                programState = 2;
                        }
                break;

        case 2:
        totalTime = rampTime;
        // //Serial.print("Timer: ");
        // //Serial.print(timerSerial);
        // //Serial.print(" of ");
        // //Serial.println(TDdenatureTimeSec);
        //Serial.print("Input3: ");
        //Serial.println(Input3);
        currentMillis = millis();
        if (timerStarted == false) {
                previousMillis = millis();
                timerStarted = true;
        }
        unsigned long elapsedMillis;
        elapsedMillis = currentMillis - previousMillis;
        //Serial.print("elapsedMillis: ");
        //Serial.println(elapsedMillis);

        if (programType == 5) {
          //Serial.println("prog5");
          float tempStep;
          tempStep = endRampTemp - startRampTemp;
          //Serial.print("startRampTemp - endRampTemp: ");
          //Serial.println(tempStep);
          //Serial.print("rampTimeMin: ");
          //Serial.println(rampTime);
          tempStep /= rampTime;
          //Serial.print("tempStep /= rampTime; ");
          //Serial.println(tempStep, 6);


          tempStep *= elapsedMillis;
          //Serial.print("tempStep *= elapsedMillis; ");
          //Serial.println(tempStep,8);
          tempStep += startRampTemp;
          //Serial.print("tempStep += startRampTemp;: ");
          //Serial.println(tempStep);
          low = tempStep;
          high = tempStep;
        }
        if (programType == 6) {
          float lowtempStep;
          float hightempStep;
          lowtempStep = lowendRampTemp - lowstartRampTemp;
          lowtempStep /= rampTime;
          lowtempStep *= elapsedMillis;
          lowtempStep += lowstartRampTemp;
          hightempStep = highendRampTemp - highstartRampTemp;
          hightempStep /= rampTime;
          hightempStep *= elapsedMillis;
          hightempStep += highstartRampTemp;
          low = lowtempStep;
          high = hightempStep;
        }
        if (timerStarted == true && currentMillis - previousMillis > rampTime) {
                programState = 3;
                timerStarted = false;
                cycleState = "Program Done!";
        }
        break;

        case 3:
                progState = "Finished";
                resetPCR();
                //Serial.println("Program done!");
                break;
        }
}

void runHeat(float &low, float &high){

        switch (programState) {

        //Lid Wait
        case 0:
                cycleState = "Heating Lid";
                if (Inputlid >= setLidTemp - tempthreshold || setLidTemp == 0) {
                        programState = 1;
                        blockOn = true;
                        cycleState = "Heating Block";
                }
                break;

        case 1:
                // Serial.println("Heating to Start Temp");
                if (programType == 7) {
                  low = heatBlockTemp;
                  high = heatBlockTemp;
                }
                if (programType == 8) {
                  low = lowheatBlockTemp;
                  high = highheatBlockTemp;
                }

                if (Input3 >= high - tempthreshold) {
                                cycleState = "Running Heat Block";
                                programState = 2;
                        }
                break;

        case 2:
                totalTime = heatBlockTime;
                if (programType == 7) {
                  low = heatBlockTemp;
                  high = heatBlockTemp;
                }
                if (programType == 8) {
                  low = lowheatBlockTemp;
                  high = highheatBlockTemp;
                }
                currentMillis = millis();
                if (timerStarted == false) {
                        previousMillis = millis();
                        timerStarted = true;
                }
                if (timerStarted == true && currentMillis - previousMillis > heatBlockTime) {
                        programState = 3;
                        timerStarted = false;
                        cycleState = "Program Done!";
                }
                break;

                case 3:
                        progState = "Finished";
                        resetPCR();
                        //Serial.println("Program done!");
                        break;
              }
            }



void thermocycler(){
        if (setLidTemp != 0) {
          Setpointlid = setLidTemp;
        }

        if (PCRon == true) {
                if(isPaused == false) {
                        if (programType == 1 || programType == 2 || programType == 3 || programType == 4) {

                                runPCR(heatLow, heatHigh);
                                totalTime /= 1000;

                                heatMid = heatLow + heatHigh;
                                heatMid /= 2;

                                Setpoint1 = heatLow;
                                Setpoint2 = heatMid;
                                Setpoint3 = heatHigh;
                                Setpointfan = heatLow;

                                unsigned long serialMillis = millis();
                                if (serialMillis - previousserialMillis >= serialinterval) {
                                previousserialMillis = serialMillis;
                                Serial.print("Setpoint Low: ");
                                Serial.println(heatLow);
                                Serial.print("Setpoint Mid: ");
                                Serial.println(heatMid);
                                Serial.print("Setpoint High: ");
                                Serial.println(heatHigh);
                                Serial.print("Setpoint Fan: ");
                                Serial.println(Setpointfan);
                                Serial.print("Temp 1: ");
                                Serial.println(tempone);
                                Serial.print("Temp 2: ");
                                Serial.println(temptwo);
                                Serial.print("Temp 3: ");
                                Serial.println(tempthree);
                                Serial.print("Temp Lid: ");
                                Serial.println(templid);
                                Serial.print("Output1: ");
                                Serial.println(Output1);
                                Serial.print("Output 2: ");
                                Serial.println(Output2);
                                Serial.print("Output 3: ");
                                Serial.println(Output3);
                                Serial.print("Output Lid: ");
                                Serial.println(Outputlid);
                                Serial.print("Output Fan: ");
                                Serial.println(Outputfan);
                                Serial.print("denatureStep: ");
                                Serial.println(denatureStep);
                                Serial.print("annealStep: ");
                                Serial.println(annealStep);
                                Serial.print("extendStep: ");
                                Serial.println(extendStep);
                                // Serial.print("Mode: ");
                                // Serial.println(heatPID1.GetMode());
                                Serial.println(" ");
                                }
                        }
                      if (programType == 5 || programType == 6){
                        runRamp(heatLow, heatHigh);
                        totalTime /= 1000;
                        heatMid = heatLow + heatHigh;
                        heatMid /= 2;

                        Setpoint1 = heatLow;
                        Setpoint2 = heatMid;
                        Setpoint3 = heatHigh;
                        Setpointfan = heatLow;
                        //Serial.print("Setpoint Low: ");
                        //Serial.println(heatLow);
                        //Serial.print("Setpoint Mid: ");
                        //Serial.println(heatMid);
                        //Serial.print("Setpoint High: ");
                        //Serial.println(heatHigh);
                        //Serial.print("Temp 1: ");
                        //Serial.println(tempone);
                        //Serial.print("Temp 2: ");
                        //Serial.println(temptwo);
                        //Serial.print("Temp 3: ");
                        //Serial.println(tempthree);
                        //Serial.print("Setpoint Fan: ");
                        //Serial.println(Setpointfan);
                        //Serial.print("Output1: ");
                        //Serial.println(Output1);
                        //Serial.print("Output 2: ");
                        //Serial.println(Output2);
                        //Serial.print("Output 3: ");
                        //Serial.println(Output3);
                        //Serial.print("Output Lid: ");
                        //Serial.println(Outputlid);
                        //Serial.print("Output Fan: ");
                        //Serial.println(Outputfan);


                      }
                      if (programType == 7 || programType == 8){
                        runHeat(heatLow, heatHigh);

                        unsigned long serialMillis = millis();
                if (serialMillis - previousserialMillis >= serialinterval) {
                        previousserialMillis = serialMillis;
                        Serial.print("Setpoint Low: ");
                        Serial.println(heatLow);
                        Serial.print("Setpoint Mid: ");
                        Serial.println(heatMid);
                        Serial.print("Setpoint High: ");
                        Serial.println(Setpoint3);
                        Serial.print("Temp 1: ");
                        Serial.println(tempone);
                        Serial.print("Temp 2: ");
                        Serial.println(temptwo);
                        Serial.print("Temp 3: ");
                        Serial.println(tempthree);
                        Serial.print("Setpoint Fan: ");
                        Serial.println(Setpointfan);
                        Serial.print("Output1: ");
                        Serial.println(Output1);
                        Serial.print("Output 2: ");
                        Serial.println(Output2);
                        Serial.print("Output 3: ");
                        Serial.println(Output3);
                        Serial.print("Output Lid: ");
                        Serial.println(Outputlid);
                        Serial.print("Output Fan: ");
                        Serial.println(Outputfan);
                }

                        totalTime /= 1000;
                        heatMid = heatLow + heatHigh;
                        heatMid /= 2;

                        Setpoint1 = heatLow;
                        Setpoint2 = heatMid;
                        Setpoint3 = heatHigh;
                        Setpointfan = heatLow;
                      }
                }else{
                        Setpoint1 = tempone;
                        Setpoint2 = temptwo;
                        Setpoint3 = tempthree;
                        Setpointlid = templid;
                }
        }
        timerSerial = currentMillis - previousMillis;
        timerSerial = timerSerial / 1000;

        Input = tempone + temptwo + tempthree;
        Input /= 3;
        blockTemp = Input;
        Input1 = tempone;
        Input2 = temptwo;
        Input3 = tempthree;
        Inputlid = templid;

        if (programType == 1 || programType == 3 || programType == 5 || programType == 7) {
                Inputfan = Input2;
        }

        if (programType == 2 || programType == 4 || programType == 6 || programType == 8) {
                Inputfan = Input1;         
        }

        double gap1 = Setpoint1-Input1; 
        if (gap1 > 8){ 
                heatPID1.SetMode(MANUAL);
                Output1 = 1023;
                resetOutput1=true;
                // heatPID1.SetTunings(aggKp, aggKi, aggKd, P_ON_E);
        }
        if (gap1 <= 8 && gap1 >= 2){
                // Output1 = 0;
                heatPID1.SetMode(AUTOMATIC);
                heatPID1.SetTunings(aggKp, aggKi, aggKd, P_ON_E);
                resetOutput1 = true;
        }else if (gap1 < 2 && gap1 >= -1){
                if (resetOutput1 == true)
                {
                       heatPID1.SetMode(MANUAL);
                       Output1 = 0;
                       resetOutput1 = false;
                }
                if (resetOutput1 == false){
                heatPID1.SetMode(AUTOMATIC);
                heatPID1.SetTunings(consKp, consKi, consKd, P_ON_E);
                }
        }else if (gap1 < -1){
                heatPID1.SetMode(MANUAL);
                Output1 = 0;
                resetOutput1=true;
        }
        
        double gap2 = Setpoint2-Input2; 
        if (gap2 > 10){ 
                heatPID2.SetMode(MANUAL);
                Output2 = 1023;
                resetOutput2=true;
                // heatPID1.SetTunings(aggKp, aggKi, aggKd, P_ON_E);
        }else if (gap2 <= 10 && gap2 >= 2){
                // Output2 = 0;
                heatPID2.SetMode(AUTOMATIC);
                heatPID2.SetTunings(aggKp, aggKi, aggKd, P_ON_E);
                resetOutput2 = true;
        }else if (gap2 < 2 && gap2 >= -1){
                if (resetOutput2 == true)
                {
                       heatPID2.SetMode(MANUAL);
                       Output2 = 0;
                       resetOutput2 = false;
                }
                if (resetOutput2 == false){
                heatPID2.SetMode(AUTOMATIC);
                heatPID2.SetTunings(consKp, consKi, consKd, P_ON_E);
                }
        }else if (gap2 < -1){
                heatPID2.SetMode(MANUAL);
                Output2 = 0;
                resetOutput2=true;
        }

        double gap3 = Setpoint3-Input3; 
        if (gap3 > 8){ 
                heatPID3.SetMode(MANUAL);
                Output3 = 1023;
                resetOutput3=true;
                // heatPID1.SetTunings(aggKp, aggKi, aggKd, P_ON_E);
        }else if (gap3 <= 8 && gap3 >= 2){
                // Output3 = 0;
                heatPID3.SetMode(AUTOMATIC);
                heatPID3.SetTunings(aggKp, aggKi, aggKd, P_ON_E);
                resetOutput3 = true;
        }else if (gap3 < 2 && gap3 >= -1){
                if (resetOutput3 == true)
                {
                       heatPID3.SetMode(MANUAL);
                       Output3 = 0;
                       resetOutput3 = false;
                }
                if (resetOutput3 == false){
                heatPID3.SetMode(AUTOMATIC);
                heatPID3.SetTunings(consKp, consKi, consKd, P_ON_E);
                }
        }else if (gap3 < -1){
                heatPID3.SetMode(MANUAL);
                Output3 = 0;
                resetOutput3=true;
        }

        double gaplid = abs(Setpointlid-Inputlid); 
        if (gaplid < 3){
                lidPID.SetTunings(lidKp, lidKi, lidKd, P_ON_E);
                
        }else{
                lidPID.SetTunings(alidKp, alidKi, alidKd, P_ON_E);
        }

        double gapfan = abs(Setpointfan-Inputfan);
        if (gapfan < 1){ 
                fanPID.SetTunings(consKpfan, consKifan, consKdfan);
        }else{
                fanPID.SetTunings(aggKpfan, aggKifan, aggKdfan);
        }


        if (PCRon == true) {
                lidPID.Compute();
                analogWrite(mosfetPinLid, Outputlid);
                if (blockOn == true) {
                        heatPID1.Compute();
                        heatPID2.Compute();
                        heatPID3.Compute();

                        analogWrite(mosfetPinOne, Output1);
                        analogWrite(mosfetPinTwo, Output2);
                        analogWrite(mosfetPinThree, Output3);
                        if(isPaused == false) {

                                fanPID.Compute();
                                // //Serial.print("Output Fan: ");
                                // //Serial.println(Outputfan);

                                if (programType == 1 || programType == 3 || programType == 5 || programType == 7) {
                                  if (Outputfan >= 300 && Outputfan < 1023) {
                                        analogWrite(fanPin, 1023);
                                        }else if (Outputfan > 0 && Outputfan < 300){
                                        analogWrite(fanPin, 0);
                                        }else{
                                        analogWrite(fanPin, Outputfan);
                                        }
                                }
                                if (programType == 2 || programType == 4 || programType == 6 || programType == 8) {
                                  if (abs(Setpoint3 - Setpoint1) >= 5 && (Setpoint3 - Input3)<2 )
                                        {
                                                Outputfan = 1023;
                                                analogWrite(fanPin, Outputfan);
                                        }else{      
                                  if (Outputfan >= 350 && Outputfan < 1023) {
                                        analogWrite(fanPin, 1023);
                                        }else if (Outputfan >= 0 && Outputfan < 350){
                                        analogWrite(fanPin, 0);
                                        }else{
                                        analogWrite(fanPin, Outputfan);
                                        }
                                }
                                }




                        }else{analogWrite(fanPin, 0);}
                }
        }
        else{
                analogWrite(mosfetPinOne, 0);
                analogWrite(mosfetPinTwo, 0);
                analogWrite(mosfetPinThree, 0);
                analogWrite(fanPin, 0);
                analogWrite(mosfetPinLid, 0);
        }

        if (preheatLid == true) {

                Setpointlid = preheatLidTemp;
                lidPID.Compute();
                analogWrite(mosfetPinLid, Outputlid);
        }
        // else{
        //         analogWrite(mosfetPinLid, 0);
        // }

        if (preheatBlock == true) {
                Setpoint1 = preheatBlockTemp;
                Setpoint2 = preheatBlockTemp;
                Setpoint3 = preheatBlockTemp;
                heatPID1.Compute();
                heatPID2.Compute();
                heatPID3.Compute();
                analogWrite(mosfetPinOne, Output1);
                analogWrite(mosfetPinTwo, Output2);
                analogWrite(mosfetPinThree, Output3);

                unsigned long serialMillis = millis();
                if (serialMillis - previousserialMillis >= serialinterval) {
                        previousserialMillis = serialMillis;
                        //Serial.print("Setpoint Low: ");
                        //Serial.println(Setpoint1);
                        //Serial.print("Setpoint Mid: ");
                        //Serial.println(Setpoint2);
                        //Serial.print("Setpoint High: ");
                        //Serial.println(Setpoint3);
                        //Serial.print("Temp 1: ");
                        //Serial.println(tempone);
                        //Serial.print("Temp 2: ");
                        //Serial.println(temptwo);
                        //Serial.print("Temp 3: ");
                        //Serial.println(tempthree);
                        //Serial.print("Output1: ");
                        //Serial.println(Output1);
                        //Serial.print("Output 2: ");
                        //Serial.println(Output2);
                        //Serial.print("Output 3: ");
                        //Serial.println(Output3);
                }
        }

        if (templid < -10) {
                analogWrite(mosfetPinLid, 0);
        }

}

void deleteProgram(){
        File file = SPIFFS.open("/programs.json", "r");
        size_t size = file.size();
        std::unique_ptr<char[]> buf (new char[size]);
        file.readBytes(buf.get(), size);

        StaticJsonBuffer<1000> jsonBuffer;
        JsonObject& root = jsonBuffer.parseObject(buf.get());
        file.close();

        if (root.success()) {
                //Serial.println("root success");
                root.remove(data);
                //root.printTo(Serial);
                File file = SPIFFS.open("/programs.json", "w");
                root.printTo(file);
                file.close();
                String deletealert = "{\"deletealert\":\"Deleted "+data+"\",\"deleted\":\""+data+"\"}";
                ws.textAll(deletealert);
        }else{
                String deletefail = "{\"deletealert\":\"Program could not be deleted\"}";
                ws.textAll(deletefail);
        }
        jsonBuffer.clear();
}

void resetFiles(){
        //Serial.println("files reset");
        String resetalert;
        File config = SPIFFS.open("/config.json", "w");

        config.print("{\"update\":\"false\",\"autostart\":\"false\",\"programname\":\"\",\"programdata\":[],\"userpsw\":\"false\",\"apon\":\"true\",\"help\":\"true\",\"theme\":\"true\",\"connectwifi\":\"false\",\"userssid\":\"\",\"connected\":\"false\",\"ip\":\"\",\"quietfan\":\"false\"}");
        config.close();
        File programfile = SPIFFS.open("/programs.json", "w");
        programfile.print("{\"General Regular\":[1,100,95,180,25,95,30,45,72,60,72,300,55,0],\"General Gradient\":[2,100,95,180,25,95,30,45,72,60,72,300,55,65],\"General Touchdown\":[3,100,95,180,20,95,30,45,72,60,72,300,55,0,10,95,30,45,72,60,65,55]}");
        programfile.close();
        File wififile = SPIFFS.open("/wifi.json", "w");
        wififile.print("{\"connect\":\"false\",\"userpsw\":\"\",\"userssid\":\"\",\"userpass\":\"\"}");
        wififile.close();
        resetalert = "{\"resetalert\":\"PCR Reset\"}";
        ws.textAll(resetalert);
}



void handleApon(){
  File file = SPIFFS.open("/config.json", "r");
  size_t size = file.size();
  std::unique_ptr<char[]> buf (new char[size]);
  file.readBytes(buf.get(), size);

  StaticJsonBuffer<600> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(buf.get());
  file.close();

  //Serial.println("json root: ");
  if (root.success()) {
          String aponalert;
          //Serial.println("root success");
          if (root["apon"] == "true") {
            root["apon"] = "false";
            aponalert = "{\"aponalert\":\"proPCR will not broadcast its own wifi network on start up\"}";
          }else if (root["apon"] == "false") {
            root["apon"] = "true";
            aponalert = "{\"aponalert\":\"proPCR will broadcast its own wifi network on start up\"}";
          }
          //root.printTo(Serial);
          File file = SPIFFS.open("/config.json", "w");
          root.printTo(file);
          file.close();
          ws.textAll(aponalert);
  }else{
          String savefail = "{\"aponalert\":\"Network settings could not be changed\"}";
          ws.textAll(savefail);
  }
  jsonBuffer.clear();
}

void handleHelp(){
  File file = SPIFFS.open("/config.json", "r");
  size_t size = file.size();
  std::unique_ptr<char[]> buf (new char[size]);
  file.readBytes(buf.get(), size);

  StaticJsonBuffer<600> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(buf.get());
  file.close();

  //Serial.println("json root: ");
  if (root.success()) {
          String helpalert;
          //Serial.println("root success");
          if (root["help"] == "true") {
            root["help"] = "false";
            helpalert = "{\"helpalert\":\"Info Boxes Turned Off\"}";
          }else if (root["help"] == "false") {
            root["help"] = "true";
            helpalert = "{\"helpalert\":\"Info Boxes Turned On\"}";
          }
          //root.printTo(Serial);
          File file = SPIFFS.open("/config.json", "w");
          root.printTo(file);
          file.close();
          ws.textAll(helpalert);
  }else{
          String savefail = "{\"helpalert\":\"Help settings could not be changed\"}";
          ws.textAll(savefail);
  }
  jsonBuffer.clear();
}

void handleTheme(){
  File file = SPIFFS.open("/config.json", "r");
  size_t size = file.size();
  std::unique_ptr<char[]> buf (new char[size]);
  file.readBytes(buf.get(), size);

  StaticJsonBuffer<600> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(buf.get());
  file.close();

  //Serial.println("json root: ");
  if (root.success()) {
          String themealert;
          //Serial.println("root success");
          if (root["theme"] == "true") {
            root["theme"] = "false";
            themealert = "{\"themealert\":\"Theme Switched To Dark\"}";
          }else if (root["theme"] == "false") {
            root["theme"] = "true";
            themealert = "{\"themealert\":\"Theme Switched To Light\"}";
          }
          //root.printTo(Serial);
          File file = SPIFFS.open("/config.json", "w");
          root.printTo(file);
          file.close();
          ws.textAll(themealert);
  }else{
          String savefail = "{\"themealert\":\"Theme settings could not be changed\"}";
          ws.textAll(savefail);
  }
  jsonBuffer.clear();
}

void loadConfig(){
        // if (!SPIFFS.exists("/config.json")) {
        //         //Serial.println("make file");
        //         File file = SPIFFS.open("/config.json", "w");
        //         file.print("{\"update\":\"false\",\"autostart\":\"false\",\"programname\":\"\",\"programdata\":[],\"userpsw\":\"false\",\"apon\":\"true\",\"help\":\"true\",\"theme\":\"true\",\"connectwifi\":\"false\",\"quietfan\":\"false\"}");
        //         file.close();
        // }
        File file = SPIFFS.open("/config.json", "r");
        //Serial.println("open file");
        size_t size = file.size();
        std::unique_ptr<char[]> buf (new char[size]);
        file.readBytes(buf.get(), size);

        StaticJsonBuffer<500> jsonBuffer;
        JsonObject& root = jsonBuffer.parseObject(buf.get());
        //root.printTo(Serial);
        file.close();
        if (root.success()) {
                //Serial.println("root sucess");
                if (root["autostart"] == "true") {
                        //Serial.println("autostart true");
                        autostartName = root.get<String>("programname");
                        
                        autostartProgram = root.get<String>("programdata");
                        //Serial.print("autostartProgram: ");
                        //Serial.println(autostartProgram);
                        
                        
                        autostart = true;
                }

                if (root["apon"] == "true") {
                        apon = true;
                }else{
                        apon = false;
                }

                if (root["update"] == "true") {
                        updatestart = true;
                }
        }
        jsonBuffer.clear();
        //Serial.println("load end");
}

void setWifi(){
  //Serial.println("setWifi()");
  File file = SPIFFS.open("/wifi.json", "r");
  size_t size = file.size();
  std::unique_ptr<char[]> buf (new char[size]);
  file.readBytes(buf.get(), size);
  StaticJsonBuffer<600> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(buf.get());
  file.close();
  //Serial.println("json root: ");
  if (root.success()) {
          //Serial.println("root success");
          if (root["connect"] == "true") {
                  connectwifi = true;

                  userssid = root.get<String>("userssid");
                  userpass = root.get<String>("userpass");
                  //Serial.println("wifi credentials set");
          }else{
            connectwifi = false;
            //Serial.println("wifi credentials not set");
          }
          if (root["userpsw"] != "") {
                  //Serial.println("set userpsw");
                  userpsw = root.get<String>("userpsw");
          }

  }else{
          //Serial.println("wifi.json not parsed");
  }
  jsonBuffer.clear();
}

void quietFanSave(){
        File file = SPIFFS.open("/config.json", "r");
        size_t size = file.size();
        std::unique_ptr<char[]> buf (new char[size]);
        file.readBytes(buf.get(), size);

        StaticJsonBuffer<500> jsonBuffer;
        JsonObject& root = jsonBuffer.parseObject(buf.get());
        file.close();
        root["quietfan"] = data;
}

void saveProgram(){
        File file = SPIFFS.open("/programs.json", "r");
        size_t size = file.size();
        std::unique_ptr<char[]> buf (new char[size]);
        file.readBytes(buf.get(), size);

        StaticJsonBuffer<1000> jsonBuffer;
        JsonObject& root = jsonBuffer.parseObject(buf.get());
        file.close();

        //Serial.println("json root: ");
        if (root.success()) {
                if (root.size() > 20) {
                        String saveoverload = "{\"savealert\":\"Too many programs saved, please delete one before saving\"}";
                        ws.textAll(saveoverload);
                        return;
                }
                //Serial.println("root success");
                //Serial.print("array before: ");
                //Serial.println(data);
                JsonArray& array = jsonBuffer.parseArray(data);
                //Serial.print("json array: ");
                // //Serial.println(array);
                root[programNameSave] = array;
                //root.printTo(Serial);
                File file = SPIFFS.open("/programs.json", "w");
                root.printTo(file);
                file.close();
                String savealert = "{\"savealert\":\"Saved "+programNameSave+"\"}";
                ws.textAll(savealert);
        }else{
                String savefail = "{\"savealert\":\"Program could not be saved\"}";
                ws.textAll(savefail);
        }
        jsonBuffer.clear();
}



void runAutoStart(){
        progState = "Running";
        //Serial.println("Running Autostart");
        programName = autostartName;
        //Serial.println(programName);
        StaticJsonBuffer<500> jsonBuffer;
        JsonArray& array = jsonBuffer.parseArray(autostartProgram);
        if (!array.success()) {
                //Serial.println("autostart parse failed");
                autostart = false;
                return;
        }
        //array.printTo(Serial);
                programType = array[0];
                //Serial.print("programType: ");
                //Serial.println(programType);
                setLidTemp = array[1];
                //Serial.print("Lid temp: ");
                //Serial.println(setLidTemp);
                if (programType == 1 || programType == 2 || programType == 3 || programType == 4) {
                  initDenatureTemp = array[2];
                  //Serial.print("initDenatureTemp: ");
                  //Serial.println(initDenatureTemp);
                  initDenatureTime = array.get<float>(3);
                  initDenatureTimeSec = initDenatureTime;
                  initDenatureTime *= 1000;
                  //Serial.print("initDenatureTime: ");
                  //Serial.println(initDenatureTime);
                  cycleNum = array.get<float>(4);
                  //Serial.print("cycleNum: ");
                  //Serial.println(cycleNum);
                  denatureTemp = array.get<float>(5);
                  //Serial.print("denatureTemp: ");
                  //Serial.println(denatureTemp);
                  denatureTime = array.get<float>(6);
                  denatureTimeSec = denatureTime;
                  denatureTime *= 1000;
                  //Serial.print("denatureTime: ");
                  //Serial.println(denatureTime);
                  annealTime = array.get<float>(7);
                  annealTimeSec = annealTime;
                  annealTime *= 1000;
                  //Serial.print("annealTime: ");
                  //Serial.println(annealTime);
                  extendTemp = array.get<float>(8);
                  extendTime = array.get<float>(9);
                  extendTimeSec = extendTime;
                  extendTime *= 1000;
                  finalExtendTemp = array.get<float>(10);
                  finalExtendTime = array.get<float>(11);
                  finalExtendTimeSec = finalExtendTime;
                  finalExtendTime *= 1000;
                  //Serial.print("finalExtendTime: ");
                  //Serial.println(finalExtendTime);
                }
                if (programType == 1 || programType == 3) {
                  annealTemp = array.get<float>(12);
                  //Serial.print("annealTemp: ");
                  //Serial.println(annealTemp);
                }
                if (programType == 2 || programType == 4) {
                  annealTempLow = array.get<float>(12);
                  //Serial.print("annealTempLow: ");
                  //Serial.println(annealTempLow);
                  annealTempHigh = array.get<float>(13);
                  //Serial.print("annealTempHigh: ");
                  //Serial.println(annealTempHigh);
                }
                if (programType == 3 || programType == 4) {
                  TDcycleNum = array.get<float>(14);
                  //Serial.print("TDcycleNum: ");
                  //Serial.println(TDcycleNum);
                  TDdenatureTemp = array.get<float>(15);
                  //Serial.print("TDdenatureTemp: ");
                  //Serial.println(TDdenatureTemp);
                  TDdenatureTime = array.get<float>(16);
                  //Serial.print("TDdenatureTime: ");
                  //Serial.println(TDdenatureTime);
                  TDdenatureTimeSec = TDdenatureTime;
                  TDdenatureTime *= 1000;
                  TDannealTime = array.get<float>(17);
                  //Serial.print("TDannealTime: ");
                  //Serial.println(TDannealTime);
                  TDannealTimeSec = TDannealTime;
                  TDannealTime *= 1000;
                  TDextendTemp = array.get<float>(18);
                  //Serial.print("TDextendTemp: ");
                  //Serial.println(TDextendTemp);
                  TDextendTime = array.get<float>(19);
                  //Serial.print("TDextendTime: ");
                  //Serial.println(TDextendTime);
                  TDextendTimeSec = TDextendTime;
                  TDextendTime *= 1000;
                }
                if (programType == 3){
                  TDStartTemp = array.get<float>(20);
                  //Serial.print("TDStartTemp: ");
                  //Serial.println(TDStartTemp);
                  TDEndTemp = array.get<float>(21);
                  //Serial.print("TDEndTemp: ");
                  //Serial.println(TDEndTemp);
                }
                if (programType == 4){
                  TDLowStartTemp = array.get<float>(20);
                  //Serial.print("TDLowStartTemp: ");
                  //Serial.println(TDLowStartTemp);
                  TDHighStartTemp = array.get<float>(21);
                  //Serial.print("TDHighStartTemp: ");
                  //Serial.println(TDHighStartTemp);
                  TDLowEndTemp = array.get<float>(22);
                  //Serial.print("TDLowEndTemp: ");
                  //Serial.println(TDLowEndTemp);
                  TDHighEndTemp = array.get<float>(23);
                  //Serial.print("TDHighEndTemp: ");
                  //Serial.println(TDHighEndTemp);
                }
                if (programType == 5 || programType == 6) {
                  rampTime = array.get<float>(2);
                  //Serial.print("rampTime: ");
                  //Serial.println(rampTime);
                  rampTimeMin = rampTime;
                  rampTime *= 60000;
                }
                if (programType == 5) {
                  startRampTemp = array.get<float>(3);
                  //Serial.print("startRampTemp: ");
                  //Serial.println(startRampTemp);
                  endRampTemp = array.get<float>(4);
                  //Serial.print("endRampTemp: ");
                  //Serial.println(endRampTemp);
                }
                if (programType == 6) {
                  lowstartRampTemp = array.get<float>(3);
                  //Serial.print("lowstartRampTemp: ");
                  //Serial.println(lowstartRampTemp);
                  highstartRampTemp = array.get<float>(4);
                  //Serial.print("highstartRampTemp: ");
                  //Serial.println(highstartRampTemp);
                  lowendRampTemp = array.get<float>(5);
                  //Serial.print("lowendRampTemp: ");
                  //Serial.println(lowendRampTemp);
                  highendRampTemp = array.get<float>(6);
                  //Serial.print("highendRampTemp: ");
                  //Serial.println(highendRampTemp);
                }
                if (programType == 7 || programType == 8) {
                  heatBlockTime = array.get<float>(2);
                  //Serial.print("heatBlockTime: ");
                  //Serial.println(heatBlockTime);
                  heatBlockTimeMin = heatBlockTime;
                  heatBlockTime *= 60000;
                }
                if (programType == 7){
                  heatBlockTemp = array.get<float>(3);
                  //Serial.print("heatBlockTemp: ");
                  //Serial.println(heatBlockTemp);
                }
                if (programType == 8){
                  lowheatBlockTemp = array.get<float>(3);
                  //Serial.print("lowheatBlockTemp: ");
                  //Serial.println(lowheatBlockTemp);
                  highheatBlockTemp = array.get<float>(4);
                  //Serial.print("highheatBlockTemp: ");
                  //Serial.println(highheatBlockTemp);
                }
        PCRon = true;
        //Serial.print("PCR true autostart");
        autostart = false;
        jsonBuffer.clear();
}

const long LEDinterval = 1000;
unsigned long previousLEDMillis = 0;
boolean ledblink = true;

void handleLED(){
    if (PCRon == true) {
      float something = millis()/2000.0;
      int ledstate = 510 + 510 * sin( something * 2.0 * PI  );
      analogWrite(LEDpin, ledstate);
    }
    if (PCRon == false && connectwifi && WiFi.status() != WL_CONNECTED){
      unsigned long LEDMillis = millis();
      if (LEDMillis - previousLEDMillis >= LEDinterval) {
        previousLEDMillis = LEDMillis;
        if (ledblink) {
         analogWrite(LEDpin, 1023);
         ledblink = false;
       }else{
         analogWrite(LEDpin, 0);
         ledblink = true;
       }
      }
    }
}


void onRequest(AsyncWebServerRequest *request){
        //Serial.println("async redirect");
        request->redirect("http://connect.propcr.com");
}

void handleAutostart(){
  File file = SPIFFS.open("/config.json", "r");
  size_t size = file.size();
  std::unique_ptr<char[]> buf (new char[size]);
  file.readBytes(buf.get(), size);

  StaticJsonBuffer<600> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(buf.get());
  file.close();

  //Serial.println("json root: ");
  if (root.success()) {
          String autostartalert;
          //Serial.println("root success");
          if (root["autostart"] == "true") {
            root["autostart"] = "false";
            root["programname"] = "";
            root["programdata"] = "[]";
            autostartalert = "{\"autostartalert\":\"Autostart Turned Off\"}";
          }else if (root["autostart"] == "false") {
           
                //Serial.print("array before: ");
                //Serial.println(data);
                
            root["autostart"] = "true";
            root["programname"] = autostartprogramName;
            root["programdata"] = data;
            autostartalert = "{\"autostartalert\":\""+autostartprogramName+" will autostart next next time this proPCR turns on\"}";
          }
          //root.printTo(Serial);
          File file = SPIFFS.open("/config.json", "w");
          root.printTo(file);
          file.close();
          ws.textAll(autostartalert);
  }else{
          String savefail = "{\"autostartalert\":\"Autostart could not be set\"}";
          ws.textAll(savefail);
  }
  jsonBuffer.clear();
}

void saveUserpsw(boolean userpsw){
  //Serial.println("saveUserpsw()");
  File file = SPIFFS.open("/wifi.json", "r");
  size_t size = file.size();
  std::unique_ptr<char[]> buf (new char[size]);
  file.readBytes(buf.get(), size);
  StaticJsonBuffer<600> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(buf.get());
  file.close();
  //Serial.println("json root wifi.json: ");
  //root.printTo(Serial);
  if (root.success()) {
          //Serial.println("root success");
          String userpswalert;
          if (userpsw) {
            root["userpsw"] = data;
            userpswalert = "{\"userpswalert\":\"proPCR wifi network password updated\"}";
          }else{
            root["userpsw"] = "";
            userpswalert = "{\"userpswalert\":\"proPCR wifi network password reset to default\"}";
          }

          //root.printTo(Serial);
          File file = SPIFFS.open("/wifi.json", "w");
          root.printTo(file);
          file.close();
          jsonBuffer.clear();
          ws.textAll(userpswalert);
  }else{
          String savefail = "{\"userpswalert\":\"Password could not be set\"}";
          ws.textAll(savefail);
  }
  jsonBuffer.clear();
}


void saveUserpswConfig(boolean userpsw){
  //Serial.println("saveUserpswConfig()");
  File configfile = SPIFFS.open("/config.json", "r");
  size_t configsize = configfile.size();
  std::unique_ptr<char[]> configbuf (new char[configsize]);
  configfile.readBytes(configbuf.get(), configsize);
  StaticJsonBuffer<600> jsonBuffer;
  JsonObject& config = jsonBuffer.parseObject(configbuf.get());
  configfile.close();
  //Serial.println("json root config.json: ");
  //config.printTo(Serial);
  if (config.success()) {
    if (userpsw) {
      config["userpsw"] = "true";
    }else{
      config["userpsw"] = "false";
    }
  }
  //config.printTo(Serial);
  File savefile = SPIFFS.open("/config.json", "w");
  config.printTo(savefile);
  savefile.close();
  jsonBuffer.clear();
}

void saveWifi(boolean connect){
  File file = SPIFFS.open("/wifi.json", "r");
  size_t size = file.size();
  std::unique_ptr<char[]> buf (new char[size]);
  file.readBytes(buf.get(), size);
  StaticJsonBuffer<600> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(buf.get());
  file.close();
  //Serial.println("json root: ");
  if (root.success()) {
          //Serial.println("root success");
          String connectalert;
          if (connect) {
            root["connect"] = "true";
            root["userssid"] = userssid;
            root["userpass"] = userpass;
            connectalert = "{\"connectalert\":\"On startup proPCR will connect to the network: "+userssid+"\"}";
          }else{
                  //Serial.println("saveWifi() success");
            root["connect"] = "false";
            root["userssid"] = "";
            root["userpass"] = "";
            connectalert = "{\"connectalert\":\"On startup proPCR will only broadcast the proPCR network and will not connect to and existing network\"}";
          }
          //root.printTo(Serial);
          File file = SPIFFS.open("/wifi.json", "w");
          root.printTo(file);
          file.close();
          ws.textAll(connectalert);
  }else{
          String savefail = "{\"connectalert\":\"Wifi connect could not be set\"}";
          ws.textAll(savefail);
  }
  jsonBuffer.clear();
  //Serial.println("saveWifi() end");
}

void saveWifiConfig(boolean connect){
  File file = SPIFFS.open("/config.json", "r");
  size_t size = file.size();
  std::unique_ptr<char[]> buf (new char[size]);
  file.readBytes(buf.get(), size);
  StaticJsonBuffer<1000> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(buf.get());
  file.close();
  Serial.println("json root: ");
  if (root.success()) {
          Serial.println("root success");
    if (connect) {
      root["connectwifi"] = "true";
      root["userssid"] = userssid;

    }else{
            //Serial.println("saveWifiConfig() success");
      root["connectwifi"] = "false";
      root["userssid"] = "";
    }
  }
  //config.printTo(Serial);
  File savefile = SPIFFS.open("/config.json", "w");
  root.printTo(savefile);
  savefile.close();
  //config.printTo(Serial);
  jsonBuffer.clear();
  //Serial.println("saveWifiConfig() end");
}

void updateFirmware(){
        //Serial.println("update function ");
        runAsyncClient();
        File file = SPIFFS.open("/config.json", "r");
        size_t size = file.size();
        std::unique_ptr<char[]> buf (new char[size]);
        file.readBytes(buf.get(), size);

        StaticJsonBuffer<600> jsonBuffer;
        JsonObject& root = jsonBuffer.parseObject(buf.get());
        file.close();

        //Serial.println("json root: ");
        if (root.success()) {
                //Serial.println("root success");
                root["update"] = "true";
                //root.printTo(Serial);
                File file = SPIFFS.open("/config.json", "w");
                root.printTo(file);
                file.close();
                ESP.restart();
        }
}



void handleWS(String msg){
        //Serial.println("Program: "+msg);
        //Serial.println("websocket recieved");
        byte separator=msg.indexOf('=');
        byte separator2=msg.indexOf(':');
        byte separator3=msg.indexOf('*');
        String var=msg.substring(0,separator);
        String val=msg.substring(separator+1);
        String command = msg.substring(0,separator2);
        //Serial.println(command);
        data = msg.substring(separator2+1,separator3);

        //Serial.print("data: ");
        //Serial.println(data);

        if (command == "run") {
                programName = "";
                progState = "Running";
                resetPCR();
                preheatLid = false;
                preheatBlock = false;
                programName = msg.substring(separator3+1);
                //Serial.print("programName: ");
                //Serial.println(programName);
                StaticJsonBuffer<500> jsonBuffer;
                JsonArray& array = jsonBuffer.parseArray(data);
                if (!array.success()) {
                        //Serial.println("parse failed");
                        return;
                }
                //array.printTo(Serial);
                programType = array[0];
                //Serial.print("programType: ");
                //Serial.println(programType);
                setLidTemp = array[1];
                //Serial.print("Lid temp: ");
                //Serial.println(setLidTemp);
                if (programType == 1 || programType == 2 || programType == 3 || programType == 4) {
                  initDenatureTemp = array[2];
                  //Serial.print("initDenatureTemp: ");
                  //Serial.println(initDenatureTemp);
                  initDenatureTime = array.get<float>(3);
                  initDenatureTimeSec = initDenatureTime;
                  initDenatureTime *= 1000;
                  //Serial.print("initDenatureTime: ");
                  //Serial.println(initDenatureTime);
                  cycleNum = array.get<float>(4);
                  //Serial.print("cycleNum: ");
                  //Serial.println(cycleNum);
                  denatureTemp = array.get<float>(5);
                  //Serial.print("denatureTemp: ");
                  //Serial.println(denatureTemp);
                  denatureTime = array.get<float>(6);
                  denatureTimeSec = denatureTime;
                  denatureTime *= 1000;
                  //Serial.print("denatureTime: ");
                  //Serial.println(denatureTime);
                  annealTime = array.get<float>(7);
                  annealTimeSec = annealTime;
                  annealTime *= 1000;
                  //Serial.print("annealTime: ");
                  //Serial.println(annealTime);
                  extendTemp = array.get<float>(8);
                  extendTime = array.get<float>(9);
                  extendTimeSec = extendTime;
                  extendTime *= 1000;
                  finalExtendTemp = array.get<float>(10);
                  finalExtendTime = array.get<float>(11);
                  finalExtendTimeSec = finalExtendTime;
                  finalExtendTime *= 1000;
                  //Serial.print("finalExtendTime: ");
                  //Serial.println(finalExtendTime);
                }
                if (programType == 1 || programType == 3) {
                  annealTemp = array.get<float>(12);
                  //Serial.print("annealTemp: ");
                  //Serial.println(annealTemp);
                }
                if (programType == 2 || programType == 4) {
                  annealTempLow = array.get<float>(12);
                  //Serial.print("annealTempLow: ");
                  //Serial.println(annealTempLow);
                  annealTempHigh = array.get<float>(13);
                  //Serial.print("annealTempHigh: ");
                  //Serial.println(annealTempHigh);
                }
                if (programType == 3 || programType == 4) {
                  TDcycleNum = array.get<float>(14);
                  //Serial.print("TDcycleNum: ");
                  //Serial.println(TDcycleNum);
                  TDdenatureTemp = array.get<float>(15);
                  //Serial.print("TDdenatureTemp: ");
                  //Serial.println(TDdenatureTemp);
                  TDdenatureTime = array.get<float>(16);
                  //Serial.print("TDdenatureTime: ");
                  //Serial.println(TDdenatureTime);
                  TDdenatureTimeSec = TDdenatureTime;
                  TDdenatureTime *= 1000;
                  TDannealTime = array.get<float>(17);
                  //Serial.print("TDannealTime: ");
                  //Serial.println(TDannealTime);
                  TDannealTimeSec = TDannealTime;
                  TDannealTime *= 1000;
                  TDextendTemp = array.get<float>(18);
                  //Serial.print("TDextendTemp: ");
                  //Serial.println(TDextendTemp);
                  TDextendTime = array.get<float>(19);
                  //Serial.print("TDextendTime: ");
                  //Serial.println(TDextendTime);
                  TDextendTimeSec = TDextendTime;
                  TDextendTime *= 1000;
                }
                if (programType == 3){
                  TDStartTemp = array.get<float>(20);
                  //Serial.print("TDStartTemp: ");
                  //Serial.println(TDStartTemp);
                  TDEndTemp = array.get<float>(21);
                  //Serial.print("TDEndTemp: ");
                  //Serial.println(TDEndTemp);
                }
                if (programType == 4){
                  TDLowStartTemp = array.get<float>(20);
                  //Serial.print("TDLowStartTemp: ");
                  //Serial.println(TDLowStartTemp);
                  TDHighStartTemp = array.get<float>(21);
                  //Serial.print("TDHighStartTemp: ");
                  //Serial.println(TDHighStartTemp);
                  TDLowEndTemp = array.get<float>(22);
                  //Serial.print("TDLowEndTemp: ");
                  //Serial.println(TDLowEndTemp);
                  TDHighEndTemp = array.get<float>(23);
                  //Serial.print("TDHighEndTemp: ");
                  //Serial.println(TDHighEndTemp);
                }
                if (programType == 5 || programType == 6) {
                  rampTime = array.get<float>(2);
                  //Serial.print("rampTime: ");
                  //Serial.println(rampTime);
                  rampTimeMin = rampTime;
                  rampTime *= 60000;
                }
                if (programType == 5) {
                  startRampTemp = array.get<float>(3);
                  //Serial.print("startRampTemp: ");
                  //Serial.println(startRampTemp);
                  endRampTemp = array.get<float>(4);
                  //Serial.print("endRampTemp: ");
                  //Serial.println(endRampTemp);
                }
                if (programType == 6) {
                  lowstartRampTemp = array.get<float>(3);
                  //Serial.print("lowstartRampTemp: ");
                  //Serial.println(lowstartRampTemp);
                  highstartRampTemp = array.get<float>(4);
                  //Serial.print("highstartRampTemp: ");
                  //Serial.println(highstartRampTemp);
                  lowendRampTemp = array.get<float>(5);
                  //Serial.print("lowendRampTemp: ");
                  //Serial.println(lowendRampTemp);
                  highendRampTemp = array.get<float>(6);
                  //Serial.print("highendRampTemp: ");
                  //Serial.println(highendRampTemp);
                }
                if (programType == 7 || programType == 8) {
                  heatBlockTime = array.get<float>(2);
                  //Serial.print("heatBlockTime: ");
                  //Serial.println(heatBlockTime);
                  heatBlockTimeMin = heatBlockTime;
                  heatBlockTime *= 60000;
                }
                if (programType == 7){
                  heatBlockTemp = array.get<float>(3);
                  //Serial.print("heatBlockTemp: ");
                  //Serial.println(heatBlockTemp);
                }
                if (programType == 8){
                  lowheatBlockTemp = array.get<float>(3);
                  //Serial.print("lowheatBlockTemp: ");
                  //Serial.println(lowheatBlockTemp);
                  highheatBlockTemp = array.get<float>(4);
                  //Serial.print("highheatBlockTemp: ");
                  //Serial.println(highheatBlockTemp);
                }
                //
                // setLidTemp = array[0];
                // //Serial.print("Lid temp: ");
                // //Serial.println(setLidTemp);
                // initDenatureTemp = array[1];
                // //Serial.print("initDenatureTemp: ");
                // //Serial.println(initDenatureTemp);
                // initDenatureTime = array.get<float>(2);
                // initDenatureTimeSec = initDenatureTime;
                // initDenatureTime *= 1000;
                // //Serial.print("initDenatureTime: ");
                // //Serial.println(initDenatureTime);
                // cycleNum = array.get<float>(3);
                // //Serial.print("cycleNum: ");
                // //Serial.println(cycleNum);
                // denatureTemp = array.get<float>(4);
                // //Serial.print("denatureTemp: ");
                // //Serial.println(denatureTemp);
                // denatureTime = array.get<float>(5);
                // denatureTimeSec = denatureTime;
                // denatureTime *= 1000;
                // //Serial.print("denatureTime: ");
                // //Serial.println(denatureTime);
                // programIsGradient = array.get<float>(6);
                // //Serial.print("programIsGradient: ");
                // //Serial.println(programIsGradient);
                // annealTemp = array.get<float>(7);
                // annealTempLow = array.get<float>(8);
                // annealTempHigh = array.get<float>(9);
                // annealTime = array.get<float>(10);
                // annealTimeSec = annealTime;
                // annealTime *= 1000;
                // //Serial.print("annealTime: ");
                // //Serial.println(annealTime);
                // extendTemp = array.get<float>(11);
                // extendTime = array.get<float>(12);
                // extendTimeSec = extendTime;
                // extendTime *= 1000;
                // finalExtendTemp = array.get<float>(13);
                // finalExtendTime = array.get<float>(14);
                // finalExtendTimeSec = finalExtendTime;
                // finalExtendTime *= 1000;
                // //Serial.print("finalExtendTime: ");
                // //Serial.println(finalExtendTime);
                PCRon = true;
                jsonBuffer.clear();
        }

        if (command == "save") {
                programNameSave = msg.substring(separator3+1);
                saveProgram();
        }
        if (command == "stop") {
                progState = "Stopped";
                resetPCR();
        }
        if (command == "delete") {
                deleteProgram();
        }
        if (command == "quietfan") {
                if (data == "true") {
                        quietfan = true;
                }
                if (data == "false") {
                        quietfan = false;
                }
        }
        if (command == "autostart") {
            autostartprogramName = msg.substring(separator3+1);
            handleAutostart();
        }
        
        if (command == "quietfansave") {
                quietFanSave();
        }

        if (command == "wifion") {
                userssid = msg.substring(separator2+1,separator3);
                userpass = msg.substring(separator3+1);
                saveWifi(true);
                saveWifiConfig(true);
        }

        if (command == "wifioff") {
                saveWifi(false);
                saveWifiConfig(false);
        }

        if (command == "userpswon") {
          saveUserpsw(true);
          saveUserpswConfig(true);
        }

        if (command == "userpswoff") {
          saveUserpsw(false);
          saveUserpswConfig(false);
        }

        if (command == "apon") {
          handleApon();
        }

        if (command == "help") {
          handleHelp();
        }

        if (command == "theme") {
          handleTheme();
        }
        if (command == "reset") {
          resetFiles();
        }
        if (command == "update1") {
          updateFirmware();
        }
       
        if(var=="isPaused") {
                if(val=="true"){
                        progState = "Paused";
                        isPaused=true;
                }
                if(val=="false"){
                        progState = "Running";
                        isPaused=false;
                }
        }
        if(var=="preheatLid") {
                if(val=="true") preheatLid=true;
                if(val=="false") preheatLid=false;
        }
        if(var=="preheatBlock") {
                if(val=="true") preheatBlock=true;
                if(val=="false") preheatBlock=false;
        }
        if(var=="preheatLidTemp") {
                preheatLidTemp = val.toFloat();
                //Serial.print("preheatLidTemp: ");
                //Serial.println(preheatLidTemp);
        }
        if(var=="preheatBlockTemp") {
                preheatBlockTemp = val.toFloat();
                //Serial.print("preheatBlockTemp: ");
                //Serial.println(preheatBlockTemp);
        }
}

void sendJSON(){
        if(millis()>wait001) {
                int Setpointlidint;
                int Setpointblockint;
                Setpointlidint = (int) Setpointlid;
                Setpointblockint = (int) Setpoint2;
                int totalTimeInt = (int) totalTime;
                String pauseSwitch="Pause";
                String preheatLidText = "Lid is Off";
                String preheatLidButton = "Start";
                String preheatBlockText = "Block is Off";
                String preheatBlockButton = "Start";
                if(isPaused==true) pauseSwitch="Unpause";
                if(preheatLid==true) preheatLidText="Lid is On";
                if(preheatLid==true) preheatLidButton="Stop";
                if(preheatBlock==true) preheatBlockText="Block is On";
                if(preheatBlock==true) preheatBlockButton="Stop";
                JSONtxt="{\"runtime\":\""+runTime+"\","+ 
                         "\"templid\":\""+(String)templid+"\","+
                         "\"connected\":\""+connected+"\","+
                         "\"chipid\":\""+chipIDstring+"\","+
                         "\"localIPaddress\":\""+localIPaddress+"\","+
                         "\"userssid\":\""+userssid+"\","+
                         "\"tempone\":\""+(String)tempone+"\","+
                         "\"temptwo\":\""+(String)temptwo+"\","+
                         "\"tempthree\":\""+(String)tempthree+"\","+
                         "\"timerStarted\":\""+(String)timerStarted+"\","+
                         "\"timer\":\""+(String)timerSerial+"\","+
                         "\"totalTime\":\""+(String)totalTimeInt+"\","+
                         "\"cycleCount\":\""+(String)cycleCount+"\","+
                         "\"blockTemp\":\""+(String)blockTemp+"\","+
                         "\"Setpointlid\":\""+(String)Setpointlidint+"\","+
                         "\"Setpointblock\":\""+(String)Setpointblockint+"\","+
                         "\"cycleState\":\""+cycleState+"\","+
                         "\"progState\":\""+progState+"\","+
                         "\"programName\":\""+programName+"\","+
                         "\"programType\":\""+(String)programType+"\","+
                         "\"pcrOn\":\""+PCRon+"\","+
                         "\"preheatBlockText\":\""+preheatBlockText+"\","+
                         "\"preheatBlockButton\":\""+preheatBlockButton+"\","+
                         "\"preheatLidText\":\""+preheatLidText+"\","+
                         "\"preheatLidButton\":\""+preheatLidButton+"\","+
                         "\"pauseSwitch\":\""+pauseSwitch+"\","+
                         "\"cycleNum\":\""+(String)cycleNumInt+"\"}";
                ws.textAll(JSONtxt);
                wait001=millis()+websockMillis;
        }
}

void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
        if(type == WS_EVT_CONNECT) {
                //Serial.printf("ws[%s][%u] connect\n", server->url(), client->id());


        } else if(type == WS_EVT_DISCONNECT) {
                //Serial.printf("ws[%s][%u] disconnect: %u\n", server->url(), client->id());
        } else if(type == WS_EVT_ERROR) {
                //Serial.printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);
        } else if(type == WS_EVT_PONG) {
                //Serial.printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len) ? (char*)data : "");
        } else if(type == WS_EVT_DATA) {
                AwsFrameInfo * info = (AwsFrameInfo*)arg;
                String msg = "";
                if(info->final && info->index == 0 && info->len == len) {
                        //the whole message is in a single frame and we got all of it's data
                        //Serial.printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT) ? "text" : "binary", info->len);

                        if(info->opcode == WS_TEXT) {
                                for(size_t i=0; i < info->len; i++) {
                                        msg += (char) data[i];

                                }
                                handleWS(msg);
                        }
                } else {
                        //message is comprised of multiple frames or the frame is split into multiple packets
                        if(info->index == 0) {
                                //if(info->num == 0)
                                        //Serial.printf("ws[%s][%u] %s-message start\n", server->url(), client->id(), (info->message_opcode == WS_TEXT) ? "text" : "binary");
                                //Serial.printf("ws[%s][%u] frame[%u] start[%llu]\n", server->url(), client->id(), info->num, info->len);
                        }

                        //Serial.printf("ws[%s][%u] frame[%u] %s[%llu - %llu]: ", server->url(), client->id(), info->num, (info->message_opcode == WS_TEXT) ? "text" : "binary", info->index, info->index + len);

                        if(info->opcode == WS_TEXT) {
                                for(size_t i=0; i < info->len; i++) {
                                        msg += (char) data[i];

                                }
                                handleWS(msg);
                        } else {
                                char buff[3];
                                for(size_t i=0; i < info->len; i++) {
                                        sprintf(buff, "%02x ", (uint8_t) data[i]);
                                        msg += buff;
                                }
                        }

                }
        }
}

void setup(){
        delay(3000);
        Serial.begin(115200);
        //Serial.setDebugOutput(true);
        //Serial.println("Setup Start");

        SPIFFS.begin();
        if (drd.detectDoubleReset()) {
          //Serial.println("Double reset detected");
          resetFiles();
        }


        loadConfig();
        setWifi();
        if (updatestart == true)
        { 
                //Serial.println("Update Start");
                File file = SPIFFS.open("/config.json", "r");
                size_t size = file.size();
                std::unique_ptr<char[]> buf (new char[size]);
                file.readBytes(buf.get(), size);
                StaticJsonBuffer<600> jsonBuffer;
                JsonObject& root = jsonBuffer.parseObject(buf.get());
                file.close();
                //Serial.println("json root: ");
                if (root.success()) {
                        //Serial.println("root success");
                        root["update"] = "false";
                        //root.printTo(Serial);
                        File file = SPIFFS.open("/config.json", "w");
                        root.printTo(file);
                        file.close();
                }
                //Serial.println();
                WiFi.mode(WIFI_STA);
                //Serial.print("userssid: ");
                //Serial.println(userssid);
                //Serial.print("userpass: ");
                //Serial.println(userpass);
                if (userpass == "") {
                        WiFi.begin(userssid.c_str());
                }else{
                        WiFi.begin(userssid.c_str(), userpass.c_str());
                }
                while (WiFi.status() != WL_CONNECTED) {
                        delay(1000);
                        //Serial.println("Connecting..");
                }
                WiFiClient client;
                ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);
                t_httpUpdate_return ret = ESPhttpUpdate.updateSpiffs(client, "http://propcr.com/spiffs.bin");
                //Serial.println(ret);
                if (ret == 0) {
                        //Serial.println("Update sketch...");
                        ret = ESPhttpUpdate.update(client, "http://propcr.com/firmware.bin");

                        // switch (ret) {
                        //         case HTTP_UPDATE_FAILED:
                        //         USE_//Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
                        //         break;

                        //         case HTTP_UPDATE_NO_UPDATES:
                        //         USE_//Serial.println("HTTP_UPDATE_NO_UPDATES");
                        //         break;

                        //         case HTTP_UPDATE_OK:
                        //         USE_//Serial.println("HTTP_UPDATE_OK");
                        //         break;
                        // }
                }


        }else{
        
        WiFi.disconnect(true);
        //WiFi.setAutoConnect(false);
        //WiFi.setAutoReconnect(false);
        if (connectwifi) {
                startWifi();
        }else{
                setupAP();
        }

        if (!connectwifi) {
                const char * mdnsName = propcrChip.c_str();
                if (!MDNS.begin(mdnsName)) {
                        //Serial.println("Error setting up MDNS responder!");
                } else {
                        //Serial.println("mDNS responder started");
                        // Add service to MDNS-SD
                        MDNS.addService("http", "tcp", 80);
                        MDNS.addService("ws", "tcp", 81);
                }
        }






        pinMode(mosfetPinOne, OUTPUT);
        pinMode(mosfetPinTwo, OUTPUT);
        pinMode(mosfetPinThree, OUTPUT);
        pinMode(mosfetPinLid, OUTPUT);
        pinMode(fanPin, OUTPUT);
        pinMode(LEDpin, OUTPUT);

        heatPID1.SetMode(AUTOMATIC);
        heatPID1.SetOutputLimits(0, 1023);
        heatPID2.SetMode(AUTOMATIC);
        heatPID2.SetOutputLimits(0, 1023);
        heatPID3.SetMode(AUTOMATIC);
        heatPID3.SetOutputLimits(0, 1023);
        lidPID.SetMode(AUTOMATIC);
        lidPID.SetOutputLimits(0, 1023);
        fanPID.SetMode(AUTOMATIC);
        fanPID.SetOutputLimits(0, 1023);
        ads.setGain(GAIN_ONE);
        ads.begin();

        // attach AsyncWebSocket
        ws.onEvent(onEvent);
        server.addHandler(&ws);

        dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
        dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

        server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
  request->redirect("/index.html");

});

// server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest * request) {
//   request->redirect("/index.html");

// });

// server.on("/fwlink", HTTP_GET, [](AsyncWebServerRequest * request) {
//   request->redirect("/index.html");

// });

server.onNotFound([](AsyncWebServerRequest * request) {
    request->redirect("/index.html");
  });
//
// // respond to GET requests on URL /heap
// server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
//         request->send(200, "text/plain", String(ESP.getFreeHeap()));
// });

server.on("/scan", HTTP_GET, [](AsyncWebServerRequest *request){
  String json = "[";
  int n = WiFi.scanComplete();
  if(n == -2){
    WiFi.scanNetworks(true);
  } else if(n){
    for (int i = 0; i < n; ++i){
      if(i) json += ",";
      json += "{";
      json += "\"rssi\":"+String(WiFi.RSSI(i));
      json += ",\"ssid\":\""+WiFi.SSID(i)+"\"";
      json += ",\"bssid\":\""+WiFi.BSSIDstr(i)+"\"";
      json += ",\"channel\":"+String(WiFi.channel(i));
      json += ",\"secure\":"+String(WiFi.encryptionType(i));
      json += ",\"hidden\":"+String(WiFi.isHidden(i)?"true":"false");
      json += "}";
    }
    WiFi.scanDelete();
    if(WiFi.scanComplete() == -2){
      WiFi.scanNetworks(true);
    }
  }
  json += "]";
  request->send(200, "application/json", json);
  json = String();
});
//
// server.on("/programs.json", HTTP_GET, [](AsyncWebServerRequest *request){
//         request->send(SPIFFS, "/programs.json", "text/json");
// });
//
// server.on("/config.json", HTTP_GET, [](AsyncWebServerRequest *request){
//         request->send(SPIFFS, "/config.json", "text/json");
// });
        // attach filesystem root at URL /fs

        // Catch-All Handlers
        // Any request that can not find a Handler that canHandle it
        // ends in the callbacks below.
      //  server.onNotFound(onRequest);

        server.begin();
       // WiFi.scanNetworks();
        }
}



void loop(){
      if (doubleReset) {
        if (resetOn) {
          resetTime = millis();
          resetOn = false;
        }

        if (millis() - resetTime < 60000) {
          // //Serial.println("drd.loop");
            drd.loop();
        }
        if (millis() - resetTime > 60001) {
          //Serial.println("drd.loop off");
            doubleReset = false;
        }
      }
        if (WiFi.status() != 3 && APcatch) {
                //Serial.print("WiFi.status() = ");
                //Serial.println(WiFi.status());
                //Serial.println("APcatch started");
                if (APtimernew) {
                        APtime = millis();
                        APtimernew = false;
                }

                if(millis() - APtime > 20000) {
                        //Serial.println("APcatch tripped");
                        connectwifi = false;
                        WiFi.mode(WIFI_AP);
                        delay(500);
                        setupAP();
                        const char * mdnsName = propcrChip.c_str();
                        if (!MDNS.begin(mdnsName)) {
                                //Serial.println("Error setting up MDNS responder!");
                        } else {
                                //Serial.println("mDNS responder started1");
                                // Add service to MDNS-SD
                                MDNS.addService("http", "tcp", 80);
                                MDNS.addService("ws", "tcp", 81);
                        }
                        APcatch = false;
                }
        }else{APtimernew = true;}


        if (connectwifi && WiFi.status() == WL_CONNECTED) {
                if (openmdns) {
                        openmdns = false;
                        const char * mdnsName = propcrChip.c_str();
                        if (!MDNS.begin(mdnsName)) {
                                //Serial.println("Error setting up MDNS responder!");
                        } else {
                                //Serial.println("mDNS responder started");
                                // Add service to MDNS-SD
                                MDNS.addService("http", "tcp", 80);
                                MDNS.addService("ws", "tcp", 81);
                        }
                }
        }
        // //Serial.println(autostart);
        if (autostart) {
                runAutoStart();
        }
        handleLED();
        dnsServer.processNextRequest();
        readThermistors();
        thermocycler();
        sendJSON();

unsigned long WSMillis = millis();
      if (WSMillis - previousWSMillis >= WSinterval) {
        previousWSMillis = WSMillis;
        ws.cleanupClients();
      }
}





