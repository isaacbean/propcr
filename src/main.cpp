#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
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
#define mosfetPinOne D5
#define mosfetPinTwo D6
#define mosfetPinThree D7
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
String webSite,javaScript,JSONtxt;
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

//Normal PID
//Specify the links and initial tuning parameters
//double Kp=600, Ki=0, Kd=.00001;
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
double aggKp=400, aggKi=.01, aggKd=.0001;
double consKp=2000, consKi=0.06, consKd=.001;
double aggKpfan=400, aggKifan=.01, aggKdfan=.0001;
double consKpfan=100, consKifan=.1, consKdfan=.2;
//Specify the links and initial tuning parameters
PID heatPID1(&Input1, &Output1, &Setpoint1, consKp, consKi, consKd, DIRECT);
PID heatPID2(&Input2, &Output2, &Setpoint2, consKp, consKi, consKd, DIRECT);
PID heatPID3(&Input3, &Output3, &Setpoint3, consKp, consKi, consKd, DIRECT);
PID lidPID(&Inputlid, &Outputlid, &Setpointlid, consKp, consKi, consKd, DIRECT);
PID fanPID(&Inputfan, &Outputfan, &Setpointfan, consKpfan, consKifan, consKdfan, REVERSE);

boolean PCRon = false;
boolean blockOn = false;
boolean preheatLid = false;
boolean preheatBlock = false;

float blockTemp;

float preheatLidTemp;
float preheatBlockTemp;

boolean programIsGradient;
boolean timerStarted = false;
float tempreturn;
float tempset;

float tempthreshold = 3;
float setLidTemp;
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
byte cycleCount = 1;
float heatLow = 0;
float heatHigh = 0;
float heatMid;
unsigned long timerSerial;
float initDenatureTimeSec;
float annealTimeSec;
float extendTimeSec;
float denatureTimeSec;
float finalExtendTimeSec;
boolean isGradient = false;
boolean denatureStep = true;
boolean annealStep = false;
boolean extendStep = false;
String cycleState = "Lid Heating";
byte programState = 0;
unsigned long previousMillis;
unsigned long currentMillis;

String data;
String programName;
String programNameSave;
String autostartName;
String autostartProgram;
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
String test = "test";
boolean openmdns = true;
boolean APcatch = false;
boolean APtimernew = true;
unsigned long APtime;
String connected = "false";
String localIPaddress = "";

//===> functions <--------------------------------------------------------------

void setupAP(){
        Serial.println("Configuring access point...");
        String chipID = String(ESP.getChipId()).substring(0,3);
        Serial.println(chipID);
        chipIDint = chipID.toInt();
        while (chipIDint > 255 || chipIDint < 100) {
                if (chipIDint < 100) {
                  chipIDint = chipIDint + 500;
                }
                chipIDint = chipIDint - 155;
                Serial.println(chipIDint);
        }
        IPAddress apIP(1, 1, 1, chipIDint);
        String chipIDstring = String(chipIDint);
        softAP_ssid = "proPCR." + chipIDstring;
        propcrChip = "propcr" + chipIDstring;
        WiFi.hostname(propcrChip);
        Serial.print("softAP_ssid: ");
        Serial.println(softAP_ssid);
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
        Serial.print("AP IP address: ");
        Serial.println(WiFi.softAPIP());

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

        Serial.println("function: startWifi()");
        int n = WiFi.scanNetworks();
        Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i)
    {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE)?" ":"*");
      delay(10);
    }
        WiFi.mode(WIFI_STA);
        Serial.print("userssid: ");
        Serial.println(userssid);
        Serial.print("userpass: ");
        Serial.println(userpass);
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
                Serial.println("wifi connected");
      			connected = "true";
            localIPaddress = toStringIp(WiFi.localIP());
            Serial.print("local ip:");
            Serial.println(WiFi.localIP());
            Serial.println(localIPaddress);
      		}
          }

        Serial.print("Wifi status: ");
        Serial.println(WiFi.status());
        if (WiFi.status() == 3){
                  if (apon) {
                          WiFi.mode(WIFI_AP_STA);
                          delay(500);
                          Serial.println("connected ap sta");

                  }
                        Serial.print("connected localIP: ");
                        Serial.println(WiFi.localIP());
        }
        if (apon) {
                setupAP();
        }
}


void wifiScan(){
  String json = "{\"wifiscan\"[";
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
  ws.textAll(json);
  json = String();
}

String getContentType(String filename) { // convert the file extension to the MIME type
        if (filename.endsWith(".html")) return "text/html";
        else if (filename.endsWith(".css")) return "text/css";
        else if (filename.endsWith(".js")) return "application/javascript";
        else if (filename.endsWith(".ico")) return "image/x-icon";
        else if (filename.endsWith(".gz")) return "application/x-gzip";
        else if (filename.endsWith(".json")) return "application/json";
        return "text/plain";
}

boolean isIp(String str) {
        for (int i = 0; i < str.length(); i++) {
                int c = str.charAt(i);
                if (c != '.' && (c < '0' || c > '9')) {
                        return false;
                }
        }
        return true;
}



// boolean captivePortal() {
//         if (!isIp(server.hostHeader()) && server.hostHeader() != (propcrChip+".local")) {
//                 Serial.println("Request redirected to captive portal");
//                 server.sendHeader("Location", String("http://") + toStringIp(server.client().localIP()), true);
//                 server.send ( 302, "text/plain", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
//                 server.client().stop(); // Stop is needed because we sent no content length
//                 return true;
//         }
//         return false;
// }

float readTherm(int therm){

        uint8_t i;
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
           //Serial.print("Average analog reading: ");
           //Serial.println(average);
         */
        average = therm;
        // convert the value to resistance
        average = 17830 / average - 1;
        average = SERIESRESISTOR / average;
        //Serial.print("Thermistor ");
        //Serial.print(therm);
        //Serial.print(" resistance conversion: ");
        //Serial.println(average);


        steinhart = average / THERMISTORNOMINAL;    // (R/Ro)
        steinhart = log(steinhart);               // ln(R/Ro)
        steinhart /= BCOEFFICIENT;                // 1/B * ln(R/Ro)
        steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);    // + (1/To)
        steinhart = 1.0 / steinhart;              // Invert
        steinhart -= 273.15;                      // convert to C
        /*
           Serial.print("Thermistor ");
           Serial.print(therm);
           Serial.print(" temp celcius: ");
           Serial.println(steinhart);
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
        cycleCount = 0;
        PCRon = false;
        programState = 0;
        blockOn = false;
        preheatBlock = false;
        preheatLid = false;
}



float regularPCR(){

        switch (programState) {

        //Lid Wait
        case 0:
                if (Inputlid >= setLidTemp - tempthreshold) {
                        programState = 1;
                        blockOn = true;
                        cycleState = "Initial Denature";
                        Serial.println("Initial Denature");
                }
                break;

        //Initial Denature
        case 1:
                Serial.println("Initial Denature");
                tempset = initDenatureTemp;
                if (Input >= tempset - tempthreshold) {
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
                                Serial.println("Timer Stopped");
                                timerStarted = false;
                                programState = 2;
                                cycleState = "Denature Step";
                        }
                }
                return tempset;

        //Cycles
        case 2:

                if (cycleCount < cycleNum) {
                        Serial.print("Cycle ");
                        Serial.print(cycleCount);
                        Serial.print(" of ");
                        Serial.println(cycleNum);
                        //Denature step
                        if (denatureStep == true) {
                                Serial.println("Denature Step");

                                tempset = denatureTemp;

                                if (Input >= tempset - tempthreshold && Input <= tempset + tempthreshold) {
                                        Serial.print("Timer: ");
                                        Serial.print(timerSerial);
                                        Serial.print(" of ");
                                        Serial.println(denatureTimeSec);
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
                                Serial.println("Anneal Step");
                                tempset = annealTemp;

                                if (Input >= tempset - tempthreshold && Input <= tempset + tempthreshold) {
                                        Serial.print("Timer: ");
                                        Serial.print(timerSerial);
                                        Serial.print(" of ");
                                        Serial.println(annealTimeSec);
                                        currentMillis = millis();

                                        if (timerStarted == false) {
                                                previousMillis = millis();
                                                timerStarted = true;
                                        }
                                }
                                if (timerStarted == true && currentMillis - previousMillis > annealTime) {
                                        Serial.println("Timer Stopped");
                                        annealStep = false;
                                        extendStep = true;
                                        timerStarted = false;
                                        cycleState = "Extend Step";
                                }
                        }
                        //Extend step
                        if (extendStep == true) {
                                Serial.println("Extend Step");
                                tempset = extendTemp;
                        }
                        if (Input >= tempset - tempthreshold && Input <= tempset + tempthreshold) {
                                Serial.print("Timer: ");
                                Serial.print(timerSerial);
                                Serial.print(" of ");
                                Serial.println(extendTimeSec);
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

                if (cycleCount == cycleNum) {
                        programState = 3;
                        cycleState = "Final Extend Step";
                }
                return tempset;

        //Final extend
        case 3:
                Serial.println("Final Extend Step");

                tempset = finalExtendTemp;

                if (Input >= tempset - tempthreshold && Input <= tempset + tempthreshold) {
                        Serial.print("Timer: ");
                        Serial.print(timerSerial);
                        Serial.print(" of ");
                        Serial.println(finalExtendTimeSec);
                        currentMillis = millis();
                        if (timerStarted == false) {
                                previousMillis = millis();
                                timerStarted = true;
                        }
                        if (timerStarted == true && currentMillis - previousMillis > finalExtendTime) {
                                timerStarted = false;
                                programState = 4;
                                cycleState = "Program Done!";
                        }
                }

                return tempset;

        //Program done
        case 4:
                resetPCR();
                Serial.println("Program done!");
                break;
        }
}

void gradientPCR(float &low, float &high){

        switch (programState) {

        //Lid Wait
        case 0:
                if (Inputlid >= setLidTemp - tempthreshold) {
                        programState = 1;
                        blockOn = true;
                        cycleState = "Initial Denature";
                }
                break;

        //Initial Denature
        case 1:
                Serial.println("Initial Denature");
                low = initDenatureTemp;
                high = initDenatureTemp;
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
                                programState = 2;
                                cycleState = "Denature Step";
                        }
                }
                break;

        //Cycles
        case 2:

                if (cycleCount <= cycleNum) {
                        Serial.print("Cycle ");
                        Serial.print(cycleCount);
                        Serial.print(" of ");
                        Serial.println(cycleNum);

                        //Denature step
                        if (denatureStep == true) {
                                Serial.println("Denature Step");

                                low = denatureTemp;
                                high = denatureTemp;

                                if (Input1 >= low - tempthreshold && Input1 <= low + tempthreshold) {
                                        Serial.print("Timer: ");
                                        Serial.print(timerSerial);
                                        Serial.print(" of ");
                                        Serial.println(denatureTimeSec);
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
                                Serial.println("Anneal Step");

                                low = annealTempLow;
                                high = annealTempHigh;

                                if (Input1 >= low - tempthreshold && Input1 <= low + tempthreshold) {
                                        Serial.print("Timer: ");
                                        Serial.print(timerSerial);
                                        Serial.print(" of ");
                                        Serial.println(annealTimeSec);
                                        currentMillis = millis();

                                        if (timerStarted == false) {
                                                previousMillis = millis();
                                                timerStarted = true;
                                        }
                                }
                                if (timerStarted == true && currentMillis - previousMillis > annealTime) {
                                        Serial.println("Timer Stopped");
                                        annealStep = false;
                                        extendStep = true;
                                        timerStarted = false;
                                        cycleState = "Extend Step";
                                }
                        }
                        //Extend step
                        if (extendStep == true) {
                                Serial.println("Extend Step");

                                low = extendTemp;
                                high = extendTemp;

                                if (Input1 >= low - tempthreshold && Input1 <= low + tempthreshold) {
                                        Serial.print("Timer: ");
                                        Serial.print(timerSerial);
                                        Serial.print(" of ");
                                        Serial.println(extendTimeSec);
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
                if (cycleCount == cycleNum) {
                        programState = 3;
                        cycleState = "Final Extend Step";
                }
                break;

        //Final extend
        case 3:
                Serial.println("Final Extend Step");

                low = finalExtendTemp;
                high = finalExtendTemp;

                if (Input1 >= low - tempthreshold && Input1 <= low + tempthreshold) {
                        Serial.print("Timer: ");
                        Serial.print(timerSerial);
                        Serial.print(" of ");
                        Serial.println(finalExtendTimeSec);
                        currentMillis = millis();
                        if (timerStarted == false) {
                                previousMillis = millis();
                                timerStarted = true;
                        }
                        if (timerStarted == true && currentMillis - previousMillis > finalExtendTime) {
                                timerStarted = false;
                                programState = 4;
                                cycleState = "Program Done!";
                        }
                }

                break;

        //Program done
        case 4:
                resetPCR();
                Serial.println("Program done!");
                break;
        }
}

void thermocycler(){
        Setpointlid = setLidTemp;
        if (PCRon == true) {
                Serial.print("PCR on");
                if(isPaused == false) {
                        if (programIsGradient == false) {

                                tempreturn = regularPCR();

                                Setpoint1 = tempreturn;
                                Setpoint2 = tempreturn;
                                Setpoint3 = tempreturn;
                                Setpointfan = tempreturn;

                                // Serial.print("Setpoint Lid: ");
                                // Serial.println(setLidTemp);
                                // Serial.print("Setpoint: ");
                                // Serial.println(tempreturn);
                                // Serial.print("Temp 1: ");
                                // Serial.println(tempone);
                                // Serial.print("Temp 2: ");
                                // Serial.println(temptwo);
                                // Serial.print("Temp 3: ");
                                // Serial.println(tempthree);
                                // Serial.print("Temp Lid: ");
                                // Serial.println(templid);

                        }

                        if (programIsGradient == true) {

                                gradientPCR(heatLow, heatHigh);

                                heatMid = heatLow + heatHigh;
                                heatMid /= 2;

                                Setpoint1 = heatLow;
                                Setpoint2 = heatMid;
                                Setpoint3 = heatHigh;
                                Setpointfan = heatLow;
                                // Serial.print("Setpoint Low: ");
                                // Serial.println(heatLow);
                                // Serial.print("Setpoint Mid: ");
                                // Serial.println(heatMid);
                                // Serial.print("Setpoint High: ");
                                // Serial.println(heatHigh);
                                // Serial.print("Temp 1: ");
                                // Serial.println(tempone);
                                // Serial.print("Temp 2: ");
                                // Serial.println(temptwo);
                                // Serial.print("Temp 3: ");
                                // Serial.println(tempthree);
                                // Serial.print("Temp Lid: ");
                                // Serial.println(templid);
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

        if (programIsGradient == false) {
                Inputfan = Input;
        }

        if (programIsGradient == true) {
                Inputfan = Input1;
        }


        double gap1 = abs(Setpoint1-Input1); //distance away from setpoint
        if (gap1 < 3)
        { //we're close to setpoint, use conservative tuning parameters
                heatPID1.SetTunings(consKp, consKi, consKd);
        }
        else
        {
                //we're far from setpoint, use aggressive tuning parameters
                heatPID1.SetTunings(aggKp, aggKi, aggKd);
        }

        double gap2 = abs(Setpoint2-Input2); //distance away from setpoint
        if (gap2 < 3)
        { //we're close to setpoint, use conservative tuning parameters
                heatPID2.SetTunings(consKp, consKi, consKd);
        }
        else
        {
                //we're far from setpoint, use aggressive tuning parameters
                heatPID2.SetTunings(aggKp, aggKi, aggKd);
        }

        double gap3 = abs(Setpoint3-Input3); //distance away from setpoint
        if (gap3 < 3) { //we're close to setpoint, use conservative tuning parameters
                heatPID3.SetTunings(consKp, consKi, consKd);
        }
        else{
                //we're far from setpoint, use aggressive tuning parameters
                heatPID3.SetTunings(aggKp, aggKi, aggKd);
        }

        double gaplid = abs(Setpointlid-Inputlid); //distance away from setpoint
        if (gaplid < 3)
        { //we're close to setpoint, use conservative tuning parameters
                lidPID.SetTunings(consKp, consKi, consKd);
        }
        else
        {
                //we're far from setpoint, use aggressive tuning parameters
                lidPID.SetTunings(aggKp, aggKi, aggKd);
        }

        double gapfan = abs(Setpointfan-Inputfan); //distance away from setpoint
        if (gapfan < 2)
        { //we're close to setpoint, use conservative tuning parameters
                fanPID.SetTunings(consKpfan, consKifan, consKdfan);
        }
        else
        {
                //we're far from setpoint, use aggressive tuning parameters
                fanPID.SetTunings(aggKpfan, aggKifan, aggKdfan);
        }

        if (Setpointfan > Input1) {
                if (quietfan) {
                        fanPID.SetOutputLimits(0, 800);
                }else{fanPID.SetOutputLimits(0, 1023);}

        }
        else {
                fanPID.SetOutputLimits(0, 0);
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
                                analogWrite(fanPin, Outputfan);
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
                Serial.println("root success");
                root.remove(data);
                root.printTo(Serial);
                File file = SPIFFS.open("/programs.json", "w");
                root.printTo(file);
                file.close();
                String deletealert = "{\"deletealert\":\"Deleted "+data+"\",\"deleted\":\""+data+"\"}";
                ws.textAll(deletealert);
        }else{
                String deletefail = "{\"deletealert\":\"Program could not be deleted\"}";
                ws.textAll(deletefail);
        }
}

void resetFiles(){
        File config = SPIFFS.open("/config.json", "w");
        config.print("{\"autostart\":\"false\",\"programname\":\"\",\"programdata\":[],\"userpsw\":\"\",\"apon\":\"true\",\"quietfan\":\"false\"}");
        config.close();
        File programfile = SPIFFS.open("/programs.json", "w");
        programfile.print("{}");
        programfile.close();
}

void handleApon(){
  File file = SPIFFS.open("/config.json", "r");
  size_t size = file.size();
  std::unique_ptr<char[]> buf (new char[size]);
  file.readBytes(buf.get(), size);

  StaticJsonBuffer<600> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(buf.get());
  file.close();

  Serial.println("json root: ");
  if (root.success()) {
          String aponalert;
          Serial.println("root success");
          if (root["apon"] == "true") {
            root["apon"] = "false";
            aponalert = "{\"aponalert\":\"proPCR will not broadcast its own wifi network on start up\"}";
          }else if (root["apon"] == "true") {
            root["apon"] = "true";
            aponalert = "{\"aponalert\":\"proPCR will broadcast its own wifi network on start up\"}";
          }
          root.printTo(Serial);
          File file = SPIFFS.open("/config.json", "w");
          root.printTo(file);
          file.close();
          ws.textAll(aponalert);
  }else{
          String savefail = "{\"aponalert\":\"Network settings could not be changed\"}";
          ws.textAll(savefail);
  }
}

void loadConfig(){
        if (!SPIFFS.exists("/config.json")) {
                Serial.println("make file");
                File file = SPIFFS.open("/programs.json", "w");
                file.print("{\"autostart\":\"false\",\"programname\":\"\",\"programdata\":[],\"userpsw\":\"false\",\"apon\":\"true\",\"connectwifi\":\"false\",\"quietfan\":\"false\"}");
                file.close();
        }
        File file = SPIFFS.open("/config.json", "r");
        Serial.println("open file");
        size_t size = file.size();
        std::unique_ptr<char[]> buf (new char[size]);
        file.readBytes(buf.get(), size);

        StaticJsonBuffer<500> jsonBuffer;
        JsonObject& root = jsonBuffer.parseObject(buf.get());
        root.printTo(Serial);
        file.close();
        if (root.success()) {
                if (root["autostart"] == "true") {
                        autostartName = root.get<String>("programname");
                        autostartProgram = "[";
                        autostartProgram += root.get<String>("programdata");
                        autostartProgram += "]";
                        autostart = true;
                }

                if (root["apon"] == "true") {
                        apon = true;
                }else{
                        apon = false;
                }

                if (root["quietfan"] == "true") {
                        quietfan = true;
                }
        }
        jsonBuffer.clear();
}

void setWifi(){
  Serial.println("setWifi()");
  File file = SPIFFS.open("/wifi.json", "r");
  size_t size = file.size();
  std::unique_ptr<char[]> buf (new char[size]);
  file.readBytes(buf.get(), size);
  StaticJsonBuffer<600> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(buf.get());
  file.close();
  Serial.println("json root: ");
  if (root.success()) {
          Serial.println("root success");
          if (root["connect"] == "true") {
                  connectwifi = true;

                  userssid = root.get<String>("userssid");
                  userpass = root.get<String>("userpass");
                  Serial.println("wifi credentials set");
          }else{
            connectwifi = false;
            Serial.println("wifi credentials not set");
          }
          if (root["userpsw"] != "") {
                  Serial.println("set userpsw");
                  userpsw = root.get<String>("userpsw");
          }

  }else{
          Serial.println("wifi.json not parsed");
  }
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

        Serial.println("json root: ");
        if (root.success()) {
                if (root.size() > 20) {
                        String saveoverload = "{\"savealert\":\"Too many programs saved, please delete one before saving\"}";
                        ws.textAll(saveoverload);
                        return;
                }
                Serial.println("root success");
                JsonArray& array = jsonBuffer.parseArray(data);
                root[programNameSave] = array;
                root.printTo(Serial);
                File file = SPIFFS.open("/programs.json", "w");
                root.printTo(file);
                file.close();
                String savealert = "{\"savealert\":\"Saved "+programNameSave+"\"}";
                ws.textAll(savealert);
        }else{
                String savefail = "{\"savealert\":\"Program could not be saved\"}";
                ws.textAll(savefail);
        }
}



void runAutoStart(){
        programName = autostartName;
        StaticJsonBuffer<500> jsonBuffer;
        JsonArray& array = jsonBuffer.parseArray(autostartProgram);
        if (!array.success()) {
                Serial.println("parse failed");
                autostart = false;
                return;
        }
        array.printTo(Serial);
        setLidTemp = array[0];
        Serial.print("Lid temp: ");
        Serial.println(setLidTemp);
        initDenatureTemp = array[1];
        Serial.print("initDenatureTemp: ");
        Serial.println(initDenatureTemp);
        initDenatureTime = array.get<float>(2);
        initDenatureTimeSec = initDenatureTime;
        initDenatureTime *= 1000;
        Serial.print("initDenatureTime: ");
        Serial.println(initDenatureTime);
        cycleNum = array.get<float>(3);
        Serial.print("cycleNum: ");
        Serial.println(cycleNum);
        denatureTemp = array.get<float>(4);
        Serial.print("denatureTemp: ");
        Serial.println(denatureTemp);
        denatureTime = array.get<float>(5);
        denatureTimeSec = denatureTime;
        denatureTime *= 1000;
        Serial.print("denatureTime: ");
        Serial.println(denatureTime);
        programIsGradient = array.get<float>(6);
        Serial.print("programIsGradient: ");
        Serial.println(programIsGradient);
        annealTemp = array.get<float>(7);
        annealTempLow = array.get<float>(8);
        annealTempHigh = array.get<float>(9);
        annealTime = array.get<float>(10);
        annealTimeSec = annealTime;
        annealTime *= 1000;
        Serial.print("annealTime: ");
        Serial.println(annealTime);
        extendTemp = array.get<float>(11);
        extendTime = array.get<float>(12);
        extendTimeSec = extendTime;
        extendTime *= 1000;
        finalExtendTemp = array.get<float>(13);
        finalExtendTime = array.get<float>(14);
        finalExtendTimeSec = finalExtendTime;
        finalExtendTime *= 1000;
        Serial.print("finalExtendTime: ");
        Serial.println(finalExtendTime);
        PCRon = true;
        autostart = false;
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

//flag to use from web update to reboot the ESP
bool shouldReboot = false;

void onRequest(AsyncWebServerRequest *request){
        request->redirect("/");
}

void handleAutostart(){
  File file = SPIFFS.open("/config.json", "r");
  size_t size = file.size();
  std::unique_ptr<char[]> buf (new char[size]);
  file.readBytes(buf.get(), size);

  StaticJsonBuffer<600> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(buf.get());
  file.close();

  Serial.println("json root: ");
  if (root.success()) {
          String autostartalert;
          Serial.println("root success");
          if (root["autostart"] == "true") {
            root["autostart"] = "false";
            autostartalert = "{\"autostartalert\":\"Autostart turned off\"}";
          }else if (root["autostart"] == "false") {
            root["autostart"] = "true";
            root["programname"] = programName;
            root["programdata"] = data;
            autostartalert = "{\"autostartalert\":\""+programName+" will autostart next power on\"}";
          }
          root.printTo(Serial);
          File file = SPIFFS.open("/config.json", "w");
          root.printTo(file);
          file.close();
          ws.textAll(autostartalert);
  }else{
          String savefail = "{\"autostartalert\":\"Autostart could not be set\"}";
          ws.textAll(savefail);
  }
}

void saveUserpsw(boolean userpsw){
  Serial.println("saveUserpsw()");
  File file = SPIFFS.open("/wifi.json", "r");
  size_t size = file.size();
  std::unique_ptr<char[]> buf (new char[size]);
  file.readBytes(buf.get(), size);
  StaticJsonBuffer<600> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(buf.get());
  file.close();
  Serial.println("json root wifi.json: ");
  root.printTo(Serial);
  if (root.success()) {
          Serial.println("root success");
          String userpswalert;
          if (userpsw) {
            root["userpsw"] = data;
            userpswalert = "{\"userpswalert\":\"proPCR wifi network password updated\"}";
          }else{
            root["userpsw"] = "";
            userpswalert = "{\"userpswalert\":\"proPCR wifi network password reset to default\"}";
          }

          root.printTo(Serial);
          File file = SPIFFS.open("/wifi.json", "w");
          root.printTo(file);
          file.close();
          jsonBuffer.clear();
          ws.textAll(userpswalert);
  }else{
          String savefail = "{\"userpswalert\":\"Password could not be set\"}";
          ws.textAll(savefail);
  }
}


void saveUserpswConfig(boolean userpsw){
  Serial.println("saveUserpswConfig()");
  File configfile = SPIFFS.open("/config.json", "r");
  size_t configsize = configfile.size();
  std::unique_ptr<char[]> configbuf (new char[configsize]);
  configfile.readBytes(configbuf.get(), configsize);
  StaticJsonBuffer<600> jb;
  JsonObject& config = jb.parseObject(configbuf.get());
  configfile.close();
  Serial.println("json root config.json: ");
  config.printTo(Serial);
  if (config.success()) {
    if (userpsw) {
      config["userpsw"] = "true";
    }else{
      config["userpsw"] = "false";
    }
  }
  config.printTo(Serial);
  File savefile = SPIFFS.open("/config.json", "w");
  config.printTo(savefile);
  savefile.close();
}

void saveWifi(boolean connect){
  File file = SPIFFS.open("/wifi.json", "r");
  size_t size = file.size();
  std::unique_ptr<char[]> buf (new char[size]);
  file.readBytes(buf.get(), size);
  StaticJsonBuffer<600> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(buf.get());
  file.close();
  Serial.println("json root: ");
  if (root.success()) {
          Serial.println("root success");
          String connectalert;
          if (connect) {
            root["connect"] = "true";
            root["userssid"] = userssid;
            root["userpass"] = userpass;
            connectalert = "{\"connectalert\":\"On startup proPCR will connect to the network: "+userssid+"\"}";
          }else{
            root["connect"] = "false";
            root["userssid"] = "";
            root["userpass"] = "";
            connectalert = "{\"connectalert\":\"On startup proPCR will only broadcast the proPCR network and will not connect to and existing network\"}";
          }
          root.printTo(Serial);
          File file = SPIFFS.open("/wifi.json", "w");
          root.printTo(file);
          file.close();
          ws.textAll(connectalert);
  }else{
          String savefail = "{\"connectalert\":\"Wifi connect could not be set\"}";
          ws.textAll(savefail);
  }
  jsonBuffer.clear();
}

void saveWifiConfig(boolean connect){
  File configfile = SPIFFS.open("/config.json", "r");
  size_t configsize = configfile.size();
  std::unique_ptr<char[]> configbuf (new char[configsize]);
  configfile.readBytes(configbuf.get(), configsize);
  StaticJsonBuffer<600> jb;
  JsonObject& config = jb.parseObject(configbuf.get());
  configfile.close();
  Serial.println("json root: ");
  if (config.success()) {
    if (connect) {
      config["connectwifi"] = "true";
      config["userssid"] = userssid;

    }else{
      config["connectwifi"] = "false";
      config["userssid"] = "";
    }
  }
  config.printTo(Serial);
  File savefile = SPIFFS.open("/config.json", "w");
  config.printTo(savefile);
  savefile.close();

}


void handleWS(String msg){
        Serial.println("Program: "+msg);
        Serial.println("websocket recieved");
        byte separator=msg.indexOf('=');
        byte separator2=msg.indexOf(':');
        byte separator3=msg.indexOf('*');
        String var=msg.substring(0,separator);
        String val=msg.substring(separator+1);
        String command = msg.substring(0,separator2);
        Serial.println(command);
        data = msg.substring(separator2+1,separator3);

        Serial.print("data: ");
        Serial.println(data);

        if (command == "run") {
                programName = msg.substring(separator3+1);
                Serial.print("programName: ");
                Serial.println(programName);
                StaticJsonBuffer<500> jsonBuffer;
                JsonArray& array = jsonBuffer.parseArray(data);
                if (!array.success()) {
                        Serial.println("parse failed");
                        return;
                }
                array.printTo(Serial);
                setLidTemp = array[0];
                Serial.print("Lid temp: ");
                Serial.println(setLidTemp);
                initDenatureTemp = array[1];
                Serial.print("initDenatureTemp: ");
                Serial.println(initDenatureTemp);
                initDenatureTime = array.get<float>(2);
                initDenatureTimeSec = initDenatureTime;
                initDenatureTime *= 1000;
                Serial.print("initDenatureTime: ");
                Serial.println(initDenatureTime);
                cycleNum = array.get<float>(3);
                Serial.print("cycleNum: ");
                Serial.println(cycleNum);
                denatureTemp = array.get<float>(4);
                Serial.print("denatureTemp: ");
                Serial.println(denatureTemp);
                denatureTime = array.get<float>(5);
                denatureTimeSec = denatureTime;
                denatureTime *= 1000;
                Serial.print("denatureTime: ");
                Serial.println(denatureTime);
                programIsGradient = array.get<float>(6);
                Serial.print("programIsGradient: ");
                Serial.println(programIsGradient);
                annealTemp = array.get<float>(7);
                annealTempLow = array.get<float>(8);
                annealTempHigh = array.get<float>(9);
                annealTime = array.get<float>(10);
                annealTimeSec = annealTime;
                annealTime *= 1000;
                Serial.print("annealTime: ");
                Serial.println(annealTime);
                extendTemp = array.get<float>(11);
                extendTime = array.get<float>(12);
                extendTimeSec = extendTime;
                extendTime *= 1000;
                finalExtendTemp = array.get<float>(13);
                finalExtendTime = array.get<float>(14);
                finalExtendTimeSec = finalExtendTime;
                finalExtendTime *= 1000;
                Serial.print("finalExtendTime: ");
                Serial.println(finalExtendTime);
                PCRon = true;
        }

        if (command == "save") {
                programNameSave = msg.substring(separator3+1);
                saveProgram();
        }
        if (command == "stop") {
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
            programName = msg.substring(separator3+1);
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

        if(var=="isPaused") {
                if(val=="true") isPaused=true;
                if(val=="false") isPaused=false;
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
                Serial.print("preheatLidTemp: ");
                Serial.println(preheatLidTemp);
        }
        if(var=="preheatBlockTemp") {
                preheatBlockTemp = val.toFloat();
                Serial.print("preheatBlockTemp: ");
                Serial.println(preheatBlockTemp);
        }
}

void sendJSON(){
        if(millis()>wait001) {
                int Setpointlidint;
                Setpointlidint = (int)Setpointlid;
                int Setpointblockint;
                Setpointblockint = (int)Setpoint2;
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
                JSONtxt="{\"runtime\":\""+runTime+"\","+  // JSON requires double quotes
                         "\"templid\":\""+(String)templid+"\","+
                         "\"connected\":\""+connected+"\","+
                         "\"localIPaddress\":\""+localIPaddress+"\","+
                         "\"userssid\":\""+userssid+"\","+
                         "\"tempone\":\""+(String)tempone+"\","+
                         "\"temptwo\":\""+(String)temptwo+"\","+
                         "\"tempthree\":\""+(String)tempthree+"\","+
                         "\"timer\":\""+(String)timerSerial+"\","+
                         "\"initDenatureTime\":\""+(String)initDenatureTimeSec+"\","+
                         "\"denatureTime\":\""+(String)denatureTimeSec+"\","+
                         "\"annealTime\":\""+(String)annealTimeSec+"\","+
                         "\"extendTime\":\""+(String)extendTimeSec+"\","+
                         "\"finalExtendTime\":\""+(String)finalExtendTimeSec+"\","+
                         "\"cycleCount\":\""+(String)cycleCount+"\","+
                         "\"blockTemp\":\""+(String)blockTemp+"\","+
                         "\"Setpointlid\":\""+(String)Setpointlidint+"\","+
                         "\"Setpointblock\":\""+(String)Setpointblockint+"\","+
                         "\"cycleState\":\""+cycleState+"\","+
                         "\"programName\":\""+programName+"\","+
                         "\"pcrOn\":\""+PCRon+"\","+
                         "\"isGradient\":\""+programIsGradient+"\","+
                         "\"preheatBlockText\":\""+preheatBlockText+"\","+
                         "\"preheatBlockButton\":\""+preheatBlockButton+"\","+
                         "\"preheatLidText\":\""+preheatLidText+"\","+
                         "\"preheatLidButton\":\""+preheatLidButton+"\","+
                         "\"pauseSwitch\":\""+pauseSwitch+"\","+
                         "\"cycleNum\":\""+(String)cycleNum+"\"}";
                ws.textAll(JSONtxt);
                wait001=millis()+websockMillis;
        }
}

void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
        if(type == WS_EVT_CONNECT) {
                Serial.printf("ws[%s][%u] connect\n", server->url(), client->id());


        } else if(type == WS_EVT_DISCONNECT) {
                Serial.printf("ws[%s][%u] disconnect: %u\n", server->url(), client->id());
        } else if(type == WS_EVT_ERROR) {
                Serial.printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);
        } else if(type == WS_EVT_PONG) {
                Serial.printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len) ? (char*)data : "");
        } else if(type == WS_EVT_DATA) {
                AwsFrameInfo * info = (AwsFrameInfo*)arg;
                String msg = "";
                if(info->final && info->index == 0 && info->len == len) {
                        //the whole message is in a single frame and we got all of it's data
                        Serial.printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT) ? "text" : "binary", info->len);

                        if(info->opcode == WS_TEXT) {
                                for(size_t i=0; i < info->len; i++) {
                                        msg += (char) data[i];

                                }
                                handleWS(msg);
                        }
                } else {
                        //message is comprised of multiple frames or the frame is split into multiple packets
                        if(info->index == 0) {
                                if(info->num == 0)
                                        Serial.printf("ws[%s][%u] %s-message start\n", server->url(), client->id(), (info->message_opcode == WS_TEXT) ? "text" : "binary");
                                Serial.printf("ws[%s][%u] frame[%u] start[%llu]\n", server->url(), client->id(), info->num, info->len);
                        }

                        Serial.printf("ws[%s][%u] frame[%u] %s[%llu - %llu]: ", server->url(), client->id(), info->num, (info->message_opcode == WS_TEXT) ? "text" : "binary", info->index, info->index + len);

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
        Serial.setDebugOutput(true);
        SPIFFS.begin();
        loadConfig();
        setWifi();
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
                        Serial.println("Error setting up MDNS responder!");
                } else {
                        Serial.println("mDNS responder started");
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

server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest * request) {
  request->redirect("/index.html");

});

server.on("/fwlink", HTTP_GET, [](AsyncWebServerRequest * request) {
  request->redirect("/index.html");

});
//
// // respond to GET requests on URL /heap
// server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
//         request->send(200, "text/plain", String(ESP.getFreeHeap()));
// });
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
}



void loop(){
        if (WiFi.status() != 3 && APcatch) {
                Serial.println("APcatch started");
                if (APtimernew) {
                        APtime = millis();
                        APtimernew = false;
                }

                if(millis() - APtime > 20000) {
                        Serial.println("APcatch tripped");
                        connectwifi = false;
                        WiFi.mode(WIFI_AP);
                        delay(500);
                        setupAP();
                        const char * mdnsName = propcrChip.c_str();
                        if (!MDNS.begin(mdnsName)) {
                                Serial.println("Error setting up MDNS responder!");
                        } else {
                                Serial.println("mDNS responder started");
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
                                Serial.println("Error setting up MDNS responder!");
                        } else {
                                Serial.println("mDNS responder started");
                                // Add service to MDNS-SD
                                MDNS.addService("http", "tcp", 80);
                                MDNS.addService("ws", "tcp", 81);
                        }
                }
        }
        if (autostart) {
                runAutoStart();
        }
        handleLED();
        dnsServer.processNextRequest();
        readThermistors();
        thermocycler();
        sendJSON();
}
