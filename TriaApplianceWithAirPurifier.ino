#include "DHT.h"
#include "MQ135.h"
#include "MATH.h"
#include <NeoSWSerial.h>
#include <string.h>

//arduino pin definition
//digital
#define DHTPIN 2
//analog
#define MQPIN 0

//sensor definitions
#define DHTTYPE DHT22
#define MDELAY 30000
#define TDELAY 500
//pins for relay activation
#define HEATERPIN 4
#define COOLERFAN 5
#define HUMIDIFIERPIN 7
#define AIRPURIFIERPIN 8

//sensor High/ Low limits
 float TH = 80.0;  // 80.0 <- 78.5 <-  80.0 <- 78.5
 float TL = 75.0; //74.0 <- 76.0 <- 75.0 <- 73.0  <- adjestment for temp sensitivity esha/ anjali
 float HH = 66.0; //66.0 <- 60.0 <- 60.0
 float HL = 50.0; //45.0 <- 41.0 <- 45.0 <- 40.0 <- 57.0 <-30.0
 float AH = 300.0; //300.0<-100<- 150.0 <- 10.0 <- 600.0 --change in library scale?
 float AL = 0.0;

 //status code and activation cd
 String alertCd="";
 int activationCd=-1;
 String sensorCd = "";
 int active = 1;

 //settings send status
 int sendSetting[3]={1,1,1};

//define bluetooth variables
const int rxPin = 10; //SoftwareSerial RX pin, connect to JY-MCY TX pin
const int txPin = 11; //SoftwareSerial TX pin, connect to JY-MCU RX pin
// level shifting to 3.3 volts may be needed
//initialize sensors
MQ135 gs = MQ135(MQPIN);
DHT dht(DHTPIN, DHTTYPE);

//define Software Serial with android bluetooth
NeoSWSerial triaSerial(rxPin, txPin); // RX, TX
char setbuff[50] = "";


//initialization and system setup
void setup() {
  Serial.begin(9600);
  delay(100);
  triaSerial.begin(9600);

  //initialize
  Serial.println("Tria Environment Alert System Initialization");

  //set pin mode
  pinMode(DHTPIN, INPUT);
  pinMode(MQPIN, INPUT);

  //set output pins
  pinMode(HEATERPIN, OUTPUT);
  pinMode(COOLERFAN, OUTPUT);
  pinMode(HUMIDIFIERPIN, OUTPUT);
  pinMode(AIRPURIFIERPIN, OUTPUT);

  //set output to off initially
  digitalWrite(HEATERPIN, HIGH);
  digitalWrite(COOLERFAN, HIGH);
  digitalWrite(HUMIDIFIERPIN, HIGH);
  digitalWrite(AIRPURIFIERPIN, HIGH);

  //bigin data collection
  dht.begin();
}

void loop() {
  //wait few seconds between measurments
  delay(MDELAY);
  //Serial.println("inside 30 sec loop");
  //read humidity
  float humidity = dht.readHumidity();
  //read temperature in celsius
  float temperature = dht.readTemperature();
  //read temperature in fahernheit
  float ftemperature = dht.readTemperature(true);

  //get calibration RZERO
  float rzero = gs.getRZero();
  //Serial.print("RZ: ");
  //Serial.println(rzero);
  

  //get ppm value
  float gsppm = gs.getPPM();
  //get temperature and humidity corrected ppm value
  float gsppmc = gs.getCorrectedPPM(temperature, humidity);


  //check if sensor data is valid for processing
  if (isnan(humidity) || isnan(temperature) || isnan(ftemperature)) {
    return;
  }
  //celsius heat index
  float heatindex = dht.computeHeatIndex(temperature, humidity, false);
  //faherenheit heat index
  float fheatindex = dht.computeHeatIndex(ftemperature, humidity);

  sensorCd = getSensorStatus(ftemperature, humidity, gsppmc);
  //Serial.print(sensorCd);

  String triaMsg = getTriaStatusMessage(sensorCd, ftemperature, humidity, gsppmc);
  String statusMsg = sensorCd + ":" + alertCd + ":" + String(activationCd);

  if(!statusMsg.equals("")){
        Serial.println("Tria Status: " +statusMsg);
    triaSerial.print(">" + statusMsg);
  }
  statusMsg="";

  delay(TDELAY);
 
  if (!triaMsg.equals("")) {
    Serial.println("Tria Values: " + triaMsg);
    triaSerial.print(triaMsg);
  }
  triaMsg = "";

  delay(TDELAY);
  
  if(sendSetting[0] == 1 || sendSetting[1] == 1 || sendSetting[2] == 1){
    if(sendSetting[0]  == 1){
       Serial.println("Tria Settings: " + String(TH) + ':' + String(TL) );
       triaSerial.print("R:T:" + String(TH) + ':' + String(TL));
       sendSetting[0] =0;
    } else if(sendSetting[1]  == 1){
       Serial.println("Tria Settings: " + String(HH) + ':' + String(HL) );
       triaSerial.print("R:H:" + String(HH) + ':' + String(HL));
        sendSetting[1] =0;
    } else if (sendSetting[2]  == 1){
       Serial.println("Tria Settings: " + String(AH) + ':' + String(AL) );
       triaSerial.print("R:A:" + String(AH) + ':' + String(AL));
       sendSetting[2]=0;
    }
    
  }
 

  byte n = triaSerial.available();
  {
    char* setparams[3];
    char* ptr;
    if (n != 0)
    {
      byte m = triaSerial.readBytesUntil('\n', setbuff, 50);
      Serial.println("Set Property request received: " + String(setbuff));
      byte index = 0;
      ptr = strtok(setbuff, ":");
      while(ptr != NULL){
        setparams[index] = ptr;
        index ++;
        ptr = strtok(NULL, ":");
      }
      String s = processChange(setparams[0], setparams[1], atof(setparams[2]));
      if (s = "S_OK"){
        Serial.println("Successfully changed the property!");
        Serial.print(setparams[1]);
        Serial.print(" to ");
        Serial.println(setparams[2]);
        Serial.println("Current Property values - TH: " + String(TH) + " TL: "+ String(TL) + " HH: "+ String(HH) + " HL: "+ String(HL) + " AH: "+ String(AH));
      }else{
        Serial.println("Invalid change property request received!");
      }
    }
  }
}

String processChange(String type, String deviceLevel, float value){
   if (type == "S"){
     if (deviceLevel == "TH"){
       TH = value;
     } else if (deviceLevel == "TL"){
       TL = value;
     } else if (deviceLevel == "HH"){
       HH = value;
     } else if (deviceLevel == "HL"){
       HL = value; 
     } else if (deviceLevel == "AH"){
       AH = value;
     } else {
      return "S_FAIL";
     }
   } else if( type == "C"){
    if (deviceLevel == "GS"){
      sendSetting[0]=1;sendSetting[1]=1;sendSetting[2]=1;
    }else if(deviceLevel == "S"){
      active = value;
    }else {
      return "S_FAIL";
    }
   }
   return "S_OK";
}

//get unified status message for user and control devices
String getTriaStatusMessage(String sensorCd, float ftemperature, float humidity, float airppm) {
 // String msg = "T:" + String(ftemperature) + ",H:" + String(humidity) + ",A:" + String(airppm) + ",C:" + sensorCd + ",M:";
 String msg = String(ftemperature) + ":" + String(humidity) + ":" + String(airppm);// + ":";
 //String alert;
 //int activationCd = -1;
  // 3st place is temperature, 2nd place is humidity & 1st place is air quality state
  if (sensorCd.equals("NNN")) {
    alertCd = "A"; //normal air quality, normal humidity, normal temperature
    activationCd = 0;
    activateDevice(0);
  }
  else if (sensorCd.equals("NNL")) {
    alertCd = "B";  // normal air quality, normal humidity, low temperature
    activationCd = 1;
    activateDevice(1);
  }
  else if (sensorCd.equals("NLN")) {
    alertCd = "C";  //normal air quality, low humidity, normal temperature
    activationCd = 2;
    activateDevice(2);
  }
  else if (sensorCd.equals("NLL")) {
    alertCd = "D";  //normal air quality, low humidity, low temperature
    activationCd = 3;
    activateDevice(3);
  }
  else if (sensorCd.equals("NNH")) {
    alertCd = "E";  //normal air quality, normal humidity, high temperature
    activationCd = 4;
    activateDevice(4);
  }
  else if (sensorCd.equals("NHN")) {
    alertCd = "E";  //normal air quality, high humidity, normal temperature 
    activationCd = 4;
    activateDevice(4);
  }
  else if (sensorCd.equals("HNN")) {
    alertCd = "E";  //high/ bad air quality, normal humidity, normal temperature
    activationCd = 5;
    activateDevice(5); // 5 <- 4
  }
  else if (sensorCd.equals("HHN")) {
    alertCd = "E";  // bad air quality, high humidity, notmal temperature
    activationCd = 6;
    activateDevice(6); // ideally de-humidifier
  }
  else if (sensorCd.equals("HNH")) {
    alertCd = "E";  // bad air quality, normal humidity, high temperature
    activationCd = 6;
    activateDevice(6);
  }
  else if (sensorCd.equals("NHH")) {
    alertCd = "E";  // normal/ low air quality, high temperature, high humidity
    activationCd = 4;
    activateDevice(4);
  }
  else if (sensorCd.equals("HHL")) {
    alertCd = "F";  //bad air quality, high humidity, low temperature
    activationCd = 6;
    activateDevice(6);
  }
  else if (sensorCd.equals("HLH")) {
    alertCd = "D";  // bad aq, low h, high t
    activationCd = 7;
    activateDevice(7);
  }
  else if (sensorCd.equals("HLL")) {
    alertCd = "I";  //bad air quality, low h, low t
    activationCd = 8;
    activateDevice(8);
  }
  else if (sensorCd.equals("NLH")) {
    alertCd = "J";  //low h, high t
    activationCd = 9;
    activateDevice(9);
  }
  else if (sensorCd.equals("NHL")) {
    alertCd = "B";  //high h, low t
    activationCd = 1;
    activateDevice(1);
  }
  else if (sensorCd.equals("HLN")) {
    alertCd = "K";  //bad aq, low h
    activationCd = 10;
    activateDevice(10);
  }
  else if (sensorCd.equals("HNL")) {
    alertCd = "L";  //bad aq, low t
    activationCd = 11;
    activateDevice(11);
  }
  else if (sensorCd.equals("HHH")) {
    alertCd = "G";  //bad air quality, high humidity, high temperature
    activationCd = 6;
    activateDevice(6);
  }
  return msg ;//+ String(activationCd);
}

// activate devices to crontrol temperature, humidity & air quality
void activateDevice(int activationCd) {

  switch (activationCd) {
    case 0:
      digitalWrite(HEATERPIN, HIGH);
      digitalWrite(HUMIDIFIERPIN, HIGH);
      digitalWrite(AIRPURIFIERPIN, HIGH);
      digitalWrite(COOLERFAN, HIGH);
      break;
    case 1:
      digitalWrite(HEATERPIN, LOW);
      digitalWrite(HUMIDIFIERPIN, HIGH);
      digitalWrite(AIRPURIFIERPIN, HIGH);
      digitalWrite(COOLERFAN, HIGH);
      break;
    case 2:
      digitalWrite(HEATERPIN, HIGH);
      digitalWrite(HUMIDIFIERPIN, LOW);
      digitalWrite(AIRPURIFIERPIN, HIGH);     
      digitalWrite(COOLERFAN, HIGH);
      break;
    case 3:
      digitalWrite(HEATERPIN, LOW);
      digitalWrite(HUMIDIFIERPIN, LOW);
      digitalWrite(AIRPURIFIERPIN, HIGH);      
      digitalWrite(COOLERFAN, HIGH);
      break;
    case 4:
      digitalWrite(HEATERPIN, HIGH);
      digitalWrite(HUMIDIFIERPIN, HIGH);
      digitalWrite(AIRPURIFIERPIN, HIGH);
      digitalWrite(COOLERFAN, LOW);
      break;
    case 5:
      digitalWrite(HEATERPIN, HIGH);
      digitalWrite(HUMIDIFIERPIN, HIGH);
      digitalWrite(AIRPURIFIERPIN, LOW);      
      digitalWrite(COOLERFAN, HIGH);
      break;
    case 6:
      digitalWrite(HEATERPIN, HIGH);
      digitalWrite(HUMIDIFIERPIN, HIGH);
      digitalWrite(AIRPURIFIERPIN, LOW); 
      digitalWrite(COOLERFAN, LOW);
      break;
    case 7:
      digitalWrite(HEATERPIN, HIGH);
      digitalWrite(HUMIDIFIERPIN, LOW);
      digitalWrite(AIRPURIFIERPIN, LOW); 
      digitalWrite(COOLERFAN, LOW);
      break;

    case 8:
      digitalWrite(HEATERPIN, LOW);
      digitalWrite(HUMIDIFIERPIN, LOW);
      digitalWrite(AIRPURIFIERPIN, LOW);      
      digitalWrite(COOLERFAN, HIGH);
      break;  

    case 9:
      digitalWrite(HEATERPIN, HIGH);
      digitalWrite(HUMIDIFIERPIN, LOW);
      digitalWrite(AIRPURIFIERPIN, HIGH);      
      digitalWrite(COOLERFAN, LOW);
      break;   

    case 10:
      digitalWrite(HEATERPIN, HIGH);
      digitalWrite(HUMIDIFIERPIN, LOW);
      digitalWrite(AIRPURIFIERPIN, LOW);      
      digitalWrite(COOLERFAN, HIGH);
      break;   
   case 11:
      digitalWrite(HEATERPIN, LOW);
      digitalWrite(HUMIDIFIERPIN, HIGH);
      digitalWrite(AIRPURIFIERPIN, LOW);      
      digitalWrite(COOLERFAN, HIGH);
      break;   
    default:;
  }
}



//function to determine sensor status
String getSensorStatus(float temperature, float humidity, float airppm) {
  String st, sh, sa, ss;

  //check temperature
  if (temperature <= TH && temperature >= TL) {
    st = "N";
  } else if (temperature > TH) {
    st = "H";
  } else {
    st = "L";
  }

  //check humidity
  if (humidity <= HH && humidity >= HL) {
    sh = "N";
  } else if (humidity > HH) {
    sh = "H";
  } else {
    sh = "L";
  }

  //check air quality
  if (airppm <= AH) {
    sa = "N";
  } else if (airppm > AH) {
    sa = "H";
  } else {
    sa = "N";
  }
  ss = sa + sh + st;

  return ss;
}
