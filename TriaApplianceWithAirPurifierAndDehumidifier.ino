#include "DHT.h"
#include "MQ135.h"
#include "MATH.h"
#include "ssd1306.h"

#include <NeoSWSerial.h>
//#include <SoftwareSerial.h>
#include <string.h>

  /*
   * arduino pin definition digital digital temperature, humidity sensor
   */
#define DHTPIN 2

  /*
   * arduino pin definition analog air quality sensor
   */
#define MQPIN 0

  /*
   * pins for relay activation
   */
#define HEATERPIN 4
#define COOLERFAN 5
#define HUMIDIFIERPIN 6 //7 
#define AIRPURIFIERPIN 7 //8
#define DEHUMIDIFIERPIN 8 //9

  /*
   * sensor definitions
   */
#define DHTTYPE DHT22

  /*
   * time milli seconds delay 
   */
#define MDELAY 30000
#define TDELAY 500

/*
 * Tria operation mode L - logic, R - Reinforcement learning, P - Paused
 */
 #define LOGICAL 0
 #define REINFORCE 1
 #define PAUSE 2

/*
 * logging levels
 */
 #define INFO 0
 #define WARN 1
 #define ERR 2

/*
 * sensor types
 */
 #define TM 0  // temperature
 #define HU 1  // humidity
 #define AQ 2  // air quatity
/*
 * Actuator states 
 */
 #define OFF 0  // Not active/ energized
 #define ON 1   // active / energized
/*
 * System statuses
 */
 #define S_OK 0
 #define S_FAIL 1
 

 /*
  * global delaration of operation mode and logging level for tria
  */
 int opsMode = REINFORCE; // 
 int logLevel = ERR;      // INFO, WARN, ERR
 
  /*
   * sensor High/ Low limit definitions
   */
 float TH = 80.0;  // 80.0 <- 78.5 <-  80.0 <- 78.5
 float TL = 75.0; //74.0 <- 76.0 <- 75.0 <- 73.0  <- adjestment for temp sensitivity esha/ anjali
 float HH = 66.0; //66.0 <- 60.0 <- 60.0
 float HL = 50.0; //45.0 <- 41.0 <- 45.0 <- 40.0 <- 57.0 <-30.0
 float AH = 400.0; //400<-300.0<-100<- 150.0 <- 10.0 <- 600.0 --change in library scale?
 float AL = 0.0;

  /*
   * status code and activation cd variables 
   */
 String alertCd="";
 int activationCd=0;
 int action = 0;
 String sensorCd = "";
 int active = 1;

  /*
   * settings send status
   */
 int sendSetting[3]={1,1,1};

 
  /*
   * port definitions for communicating with smart phone devices vis BlueTooth LE
   */
const int rxPin = 10; //SoftwareSerial RX pin, connect to JY-MCY TX pin
const int txPin = 11; //11; //SoftwareSerial TX pin, connect to JY-MCU RX pin

  /*
   * port definitions for communicating with reinforcement learning A2C model
   * const int rxPinModel = 8;  //SoftwareSerial RX interface to model prediction service
   * const int txPinModel = 9;  //SoftwareSerial TX interface to model prediction service
   * 
   */

  /*
   * define Software Serial with android/ios bluetooth
   */
NeoSWSerial triaSerial(rxPin, txPin); // RX, TX


 /*
  * define Software Serial with Reinforement Learning A2C model
  * AltSoftSerial rlSerial(rxPinModel, txPinModel); // RX, TX 
  */


  /*
   * initialize sensors level shifting to 3.3 volts may be needed
   */
MQ135 gs = MQ135(MQPIN);
DHT dht(DHTPIN, DHTTYPE);

  /*
   * initialize buffer variable to communicate with smaprt phone devices 
   */  
char setbuff[50] = "";

/*****************************************************************/
/*    setup                                                      */
/*    desc: initialization and system setup                      */
/*****************************************************************/
void setup() {

  /*
   * start Serial interface for debug text 
   */
  Serial.begin(9600);
  
  /*
   * wait few milli seconds between measurments
   */
  delay(100);

  /*
   * wait few seconds between measurments
   * rlSerial.begin(9600);
   */

  triaSerial.begin(9600);
 
 /*
  * display initialization begin
  */
    ssd1306_128x64_i2c_init();
    ssd1306_fillScreen(0x00);
    ssd1306_setFixedFont(ssd1306xled_font6x8);
    ssd1306_printFixed (0, 0, "      TRIA Status    ", STYLE_NORMAL);
    ssd1306_drawLine(0,12, 127,12);
    ssd1306_printFixed (0, 16,"T:00.0 [00-00]F      ", STYLE_NORMAL);
    ssd1306_printFixed (0, 24,"H:00.0 [00-00]rH     ", STYLE_NORMAL);
    ssd1306_printFixed (0, 32,"A:00.0 [0-000]ppm    ", STYLE_NORMAL);
    ssd1306_drawLine(0,46, 127,46);
    ssd1306_drawRect(10,55, 20,60);    // heater                                  
    ssd1306_drawRect(35,55, 45,60);    // cooler - fan
    ssd1306_drawRect(60,55, 70,60);    // humidifier
    ssd1306_drawRect(85,55, 95,60);    // air purifier
    ssd1306_drawRect(110,55, 120,60);  // de-humidifier

    updateTriaStatusDisplay(opsMode);

  /*
   * debug message about tria system initialization
   */
  log("Tria Environment Control System -- Initialization",INFO);

  /*
   * set arduino pin mode
   */
  pinMode(DHTPIN, INPUT);
  pinMode(MQPIN, INPUT);

  /*
   * set arduino relay output pin mode
   */
  pinMode(HEATERPIN, OUTPUT);
  pinMode(COOLERFAN, OUTPUT);
  pinMode(HUMIDIFIERPIN, OUTPUT);
  pinMode(AIRPURIFIERPIN, OUTPUT);
  pinMode(DEHUMIDIFIERPIN,OUTPUT);

  /*
   * set arduino output to off initially
   */
  digitalWrite(HEATERPIN, HIGH);
  digitalWrite(COOLERFAN, HIGH);
  digitalWrite(HUMIDIFIERPIN, HIGH);
  digitalWrite(AIRPURIFIERPIN, HIGH);
  digitalWrite(DEHUMIDIFIERPIN, HIGH);
  updateActuatorDisplay(OFF,OFF,OFF,OFF,OFF);

  /*
   * bigin data collection for temp and humidity
   */
  dht.begin();
}


/*****************************************************************/
/*    loop                                                       */
/*    desc: main arduino contoller loop                          */
/*****************************************************************/
void loop() {

  /*
   * wait few seconds between measurments
   */
  delay(MDELAY);

  /*
   * read humidity
   */
  float humidity = dht.readHumidity();

   /*
   * read temperature in celsius
   */
  float temperature = dht.readTemperature();

   /*
   * read temperature in fahernheit
   */
  float ftemperature = dht.readTemperature(true);

   /*
   * get calibration RZERO
   */
  float rzero = gs.getRZero();

   /*
   * get air quality ppm value
   */
  float gsppm = gs.getPPM();

   /*
   * get temperature and humidity corrected ppm value
   */
  float gsppmc = gs.getCorrectedPPM(temperature, humidity);


   /*
   * check if sensor data is valid for processing
   */
  if (isnan(humidity) || isnan(temperature) || isnan(ftemperature)) {
    return;
  }

   /*
   * celsius heat index
   */
  float heatindex = dht.computeHeatIndex(temperature, humidity, false);

   /*
   * faherenheit heat index
   */
  float fheatindex = dht.computeHeatIndex(ftemperature, humidity);

   /*
   * decided to use fheatindex (webBulb temp) inplace of (dry buld)
   */
  ftemperature = fheatindex;

  /*
   * update display values
   */
    updateSensorDisplay(TM,ftemperature,TL,TH);
    updateSensorDisplay(HU,humidity,HL,HH);
    updateSensorDisplay(AQ,gsppmc,-1,AH);

  /*  
   *   send observations to reinforcement model to get predictive action
   */
   String rlMsg = String(ftemperature) + ":" + String(humidity) + ":" + String(gsppmc);
  if (!rlMsg.equals("")) {
    log("RL Values: " + rlMsg, INFO);
    Serial.println(rlMsg);
  }
  rlMsg = "";
   /*
   * get sensor status code
   */
  sensorCd = getSensorStatus( ftemperature, humidity, gsppmc);

  /*
   * generate / compose tria device status message using sensor code, alert code and activation code
   * necessary to display sensor values on BLE device - ios phone
   */
  String triaMsg = String(ftemperature) + ":" + String(humidity) + ":" + String(gsppmc);
  /*
   * create status message...alertCd is not neccessary!?
   * nice to have except activationCd to show current status of actuators
   */
  String statusMsg = sensorCd + ":" + alertCd + ":" + String(activationCd);


   /*
   * check status message code
   */
  if(!statusMsg.equals("")){
        log("Tria Status: " +statusMsg, INFO);
    triaSerial.print(">" + statusMsg);
  }
  statusMsg="";
  
   /*
   * add time delay 
   */
  delay(TDELAY);

   /*
   * check tria message is not null
   */
  if (!triaMsg.equals("")) {
    log("Tria Values: " + triaMsg, INFO);
    triaSerial.println(triaMsg);
  }
  triaMsg = "";

   /*
   * add time delay 
   */
  delay(TDELAY);

  /*
   * check device setting values
   */
  if(sendSetting[0] == 1 || sendSetting[1] == 1 || sendSetting[2] == 1){
    if(sendSetting[0]  == 1){
       log("Tria Settings: " + String(TH) + ':' + String(TL) , INFO);
       triaSerial.print("R:T:" + String(TH) + ':' + String(TL));
       sendSetting[0] =0;
    } else if(sendSetting[1]  == 1){
       log("Tria Settings: " + String(HH) + ':' + String(HL) , INFO);
       triaSerial.print("R:H:" + String(HH) + ':' + String(HL));
        sendSetting[1] =0;
    } else if (sendSetting[2]  == 1){
       log("Tria Settings: " + String(AH) + ':' + String(AL) , INFO);
       triaSerial.print("R:A:" + String(AH) + ':' + String(AL));
       sendSetting[2]=0;
    }
  }
 
   /*
   * receive messages from BlueTooth LE 
   */
  byte n = triaSerial.available();
  {
    char* setparams[3];
    char* ptr;
    if (n != 0)
    {
      byte m = triaSerial.readBytesUntil('\n', setbuff, 50);
      //Serial.println("Set Property request received: " + String(setbuff));
      log("Set Property request received: " + String(setbuff), INFO);
      byte index = 0;
      ptr = strtok(setbuff, ":");
      while(ptr != NULL){
        setparams[index] = ptr;
        index ++;
        ptr = strtok(NULL, ":");
      }
  /*
   * process change
   */
      int state = processChange(setparams[0], setparams[1], atof(setparams[2]));
      if (state == S_OK){
        log("Successfully changed the property!", INFO);
        log(setparams[1], INFO);
        log(" to ", INFO);
        log(setparams[2], INFO);
        log("Current Property values - TH: " + String(TH) + " TL: "+ String(TL) + " HH: "+ String(HH) + " HL: "+ String(HL) + " AH: "+ String(AH), INFO);
      }else{
        log("Invalid change property request received!", ERR);
      }
    }
  }
/*
 * receive message from reinforcement / RPI model
 */
 char ch[5];
 int nb;
 if (Serial.available() > 0) {
    nb = Serial.readBytesUntil('\n', ch, 5);
    ch[nb] = '\0';
    action = atoi(ch);   
  }

  /*
   * actuators logic based on operation mode
   * L - logic
   * R - reinforment learning model
   * P - pass put all actuators in off mode
   */
 if (opsMode == LOGICAL){ 
     execLogicActuators(sensorCd);
 } else if (opsMode == REINFORCE){
     execRLActuators(action);
 } else {
     execPauseActuators();
 }
}

/**************************************************************************/
/*    ProcessChange                                                       */
/*    desc: change on device high low settings for sensor from smartphone */
/**************************************************************************/
int processChange(String type, String deviceLevel, float value){
   if (type.equals("S")){
     if (deviceLevel.equals("TH")){
       TH = value;
     } else if (deviceLevel.equals("TL")){
       TL = value;
     } else if (deviceLevel.equals("HH")){
       HH = value;
     } else if (deviceLevel.equals("HL")){
       HL = value; 
     } else if (deviceLevel.equals("AH")){
       AH = value;
     } else {
      return "S_FAIL";
     }
   } else if( type.equals("C")){
    if (deviceLevel.equals("GS")){
      sendSetting[0]=1;sendSetting[1]=1;sendSetting[2]=1;
    }else if(deviceLevel.equals("S")){
      active = value;
    }else {
      return S_FAIL;
    }
   }
   return S_OK;
}
/**************************************************************************
 * processActionReceived 
 * desc: new action prediction received from Reinforcement Model in RPI
 **************************************************************************/
void execPauseActuators(){
  activationCd =0;
  updateActuatorState(activationCd);
}

/**************************************************************************
 * processActionReceived 
 * desc: new action prediction received from Reinforcement Model in RPI
 **************************************************************************/
void execRLActuators(int action){
if (action == 0){
      alertCd = "A";
      activationCd =0;
      updateActuatorState(activationCd);
      
}else if (action == 1){
      alertCd = "Z";
      activationCd =13;
      updateActuatorState(activationCd);
  
}else if (action ==2){
      alertCd = "E";
      activationCd =4;
      updateActuatorState(activationCd);
      
}else if (action == 3){
      alertCd = "E";
      activationCd =5;
      updateActuatorState(activationCd);
      
}else if (action == 4){
      alertCd = "C";
      activationCd =2;
      updateActuatorState(activationCd);
      
}else if (action == 5){
      alertCd = "B";
      activationCd =1;
      updateActuatorState(activationCd);
  
}else{
  //Serial.print("Invalid action received: " + action);
  } 
}
/*******************************************************************/
/*    getTriaStatusMessage                                         */
/*    desc:get unified status message for user and control devices */
/******************************************************************/
String execLogicActuators(String sensorCd) {
   /*
   * create actionable inferences from tria device status 'sensorCd'
   * 3rd place is temperature, 2nd place is humidity & 1st place is air quality state
   */
  if (sensorCd.equals("NNN")) {
    alertCd = "A"; //normal air quality, normal humidity, normal temperature
    activationCd = 0;
    updateActuatorState(activationCd);
  }
  else if (sensorCd.equals("NNL")) {
    alertCd = "B";  // normal air quality, normal humidity, low temperature
    activationCd = 1;
    updateActuatorState(activationCd);
  }
  else if (sensorCd.equals("NLN")) {
    alertCd = "C";  //normal air quality, low humidity, normal temperature
    activationCd = 2;
    updateActuatorState(activationCd);
  }
  else if (sensorCd.equals("NLL")) {
    alertCd = "D";  //normal air quality, low humidity, low temperature
    activationCd = 3;
    updateActuatorState(activationCd);
  }
  else if (sensorCd.equals("NNH")) {
    alertCd = "E";  //normal air quality, normal humidity, high temperature
    activationCd = 4;
    updateActuatorState(activationCd);
  }
  else if (sensorCd.equals("NHN")) {
    alertCd = "Z";  //normal air quality, high humidity, normal temperature ***
    activationCd = 13;
    updateActuatorState(activationCd);
  }
  else if (sensorCd.equals("HNN")) {
    alertCd = "E";  // bad air quality, normal humidity, normal temperature
    activationCd = 5;
    updateActuatorState(activationCd); // 5 <- 4
  }
  else if (sensorCd.equals("HHN")) {
    alertCd = "Z";  // bad air quality, high humidity, normal temperature ***
    activationCd = 15;
    updateActuatorState(activationCd); // ideally de-humidifier
  }
  else if (sensorCd.equals("HNH")) {
    alertCd = "E";  // bad air quality, normal humidity, high temperature
    activationCd = 6;
    updateActuatorState(activationCd);
  }
  else if (sensorCd.equals("NHH")) {
    alertCd = "Z";  // normal air quality, high temperature, high humidity
    activationCd = 14;
    updateActuatorState(activationCd);
  }
  else if (sensorCd.equals("HHL")) {
    alertCd = "Z";  //bad air quality, high humidity, low temperature ***
    activationCd = 16;
    updateActuatorState(activationCd);
  }
  else if (sensorCd.equals("HLH")) {
    alertCd = "D";  // bad air quality, low humidity, high temperature
    activationCd = 7;
    updateActuatorState(activationCd);
  }
  else if (sensorCd.equals("HLL")) {
    alertCd = "I";  // bad air quality, low humidity, low temperature
    activationCd = 8;
    updateActuatorState(activationCd);
  }
  else if (sensorCd.equals("NLH")) {
    alertCd = "J";  //normal air quality, low humidity, high temperature
    activationCd = 9;
    updateActuatorState(activationCd);
  }
  else if (sensorCd.equals("NHL")) {
    alertCd = "Z";  //normal air quality, high humidity, low temperature***
    activationCd = 12;
    updateActuatorState(activationCd);
  }
  else if (sensorCd.equals("HLN")) {
    alertCd = "K";  //bad air quality, low humidity, normal temperature
    activationCd = 10;
    updateActuatorState(activationCd);
  }
  else if (sensorCd.equals("HNL")) {
    alertCd = "L";  //bad air quality, normal humidity, low temperature
    activationCd = 11;
    updateActuatorState(activationCd);
  }
  else if (sensorCd.equals("HHH")) {
    alertCd = "Z";  //bad air quality, high humidity, high temperature ***
    activationCd = 17;
    updateActuatorState(activationCd);
  }
}

/****************************************************************************/
/*    activateDevice                                                        */
/*    desc:activate devices to crontrol temperature, humidity & air quality */
/****************************************************************************/
void updateActuatorState(int activationCd) {

  switch (activationCd) {
    case 0:
      digitalWrite(HEATERPIN, HIGH);
      digitalWrite(HUMIDIFIERPIN, HIGH); 
      digitalWrite(AIRPURIFIERPIN, HIGH);
      digitalWrite(COOLERFAN, HIGH);
      digitalWrite(DEHUMIDIFIERPIN, HIGH);
      updateActuatorDisplay(OFF,OFF,OFF,OFF,OFF);
      break;
    case 1:
      digitalWrite(HEATERPIN, LOW);
      digitalWrite(HUMIDIFIERPIN, HIGH);
      digitalWrite(AIRPURIFIERPIN, HIGH);
      digitalWrite(COOLERFAN, HIGH);
      digitalWrite(DEHUMIDIFIERPIN, HIGH);
      updateActuatorDisplay(ON,OFF,OFF,OFF,OFF);
      break;
    case 2:
      digitalWrite(HEATERPIN, HIGH);
      digitalWrite(HUMIDIFIERPIN, LOW);
      digitalWrite(AIRPURIFIERPIN, HIGH);    
      digitalWrite(COOLERFAN, HIGH);
      digitalWrite(DEHUMIDIFIERPIN, HIGH);
      updateActuatorDisplay(OFF,ON,OFF,OFF,OFF);
      break;
    case 3:
      digitalWrite(HEATERPIN, LOW);
      digitalWrite(HUMIDIFIERPIN, LOW);
      digitalWrite(AIRPURIFIERPIN, HIGH);     
      digitalWrite(COOLERFAN, HIGH);
      digitalWrite(DEHUMIDIFIERPIN, HIGH);
      updateActuatorDisplay(ON,ON,OFF,OFF,OFF);
      break;
    case 4:
      digitalWrite(HEATERPIN, HIGH);
      digitalWrite(HUMIDIFIERPIN, HIGH);
      digitalWrite(AIRPURIFIERPIN, HIGH);
      digitalWrite(COOLERFAN, LOW);
      digitalWrite(DEHUMIDIFIERPIN, HIGH);
      updateActuatorDisplay(OFF,OFF,OFF,ON,OFF);
      break;
    case 5:
      digitalWrite(HEATERPIN, HIGH);
      digitalWrite(HUMIDIFIERPIN, HIGH);
      digitalWrite(AIRPURIFIERPIN, LOW);      
      digitalWrite(COOLERFAN, HIGH);
      digitalWrite(DEHUMIDIFIERPIN, HIGH);
      updateActuatorDisplay(OFF,OFF,ON,OFF,OFF);
      break;
    case 6:
      digitalWrite(HEATERPIN, HIGH);
      digitalWrite(HUMIDIFIERPIN, HIGH);
      digitalWrite(AIRPURIFIERPIN, LOW); 
      digitalWrite(COOLERFAN, LOW);
      digitalWrite(DEHUMIDIFIERPIN, HIGH);
      updateActuatorDisplay(OFF,OFF,ON,ON,OFF);
      break;
    case 7:
      digitalWrite(HEATERPIN, HIGH);
      digitalWrite(HUMIDIFIERPIN, LOW);
      digitalWrite(AIRPURIFIERPIN, LOW);  
      digitalWrite(COOLERFAN, LOW);
      digitalWrite(DEHUMIDIFIERPIN, HIGH);
      updateActuatorDisplay(OFF,ON,ON,ON,OFF);
      break;

    case 8:
      digitalWrite(HEATERPIN, LOW);
      digitalWrite(HUMIDIFIERPIN, LOW);
      digitalWrite(AIRPURIFIERPIN, LOW);      
      digitalWrite(COOLERFAN, HIGH);
      digitalWrite(DEHUMIDIFIERPIN, HIGH);
      updateActuatorDisplay(ON,ON,ON,OFF,OFF);
      break;  

    case 9:
      digitalWrite(HEATERPIN, HIGH);
      digitalWrite(HUMIDIFIERPIN, LOW);
      digitalWrite(AIRPURIFIERPIN, HIGH);     
      digitalWrite(COOLERFAN, LOW);
      digitalWrite(DEHUMIDIFIERPIN, HIGH);
      updateActuatorDisplay(OFF,ON,OFF,ON,OFF);
      break;   

    case 10:
      digitalWrite(HEATERPIN, HIGH);
      digitalWrite(HUMIDIFIERPIN, LOW);
      digitalWrite(AIRPURIFIERPIN, LOW);      
      digitalWrite(COOLERFAN, HIGH);
      digitalWrite(DEHUMIDIFIERPIN, HIGH);
      updateActuatorDisplay(OFF,ON,ON,OFF,OFF);
      break; 
        
   case 11:
      digitalWrite(HEATERPIN, LOW);
      digitalWrite(HUMIDIFIERPIN, HIGH);
      digitalWrite(AIRPURIFIERPIN, LOW);      
      digitalWrite(COOLERFAN, HIGH);
      digitalWrite(DEHUMIDIFIERPIN, HIGH);
      updateActuatorDisplay(ON,OFF,ON,OFF,OFF);
      break;   

   case 12:
      digitalWrite(HEATERPIN, LOW);
      digitalWrite(HUMIDIFIERPIN, HIGH);
      digitalWrite(AIRPURIFIERPIN, HIGH);     
      digitalWrite(COOLERFAN, HIGH);
      digitalWrite(DEHUMIDIFIERPIN, LOW);
      updateActuatorDisplay(ON,OFF,OFF,OFF,ON);
      break;   

   case 13:
      digitalWrite(HEATERPIN, HIGH);
      digitalWrite(HUMIDIFIERPIN, HIGH);
      digitalWrite(AIRPURIFIERPIN, HIGH);       
      digitalWrite(COOLERFAN, HIGH);
      digitalWrite(DEHUMIDIFIERPIN, LOW);
      updateActuatorDisplay(OFF,OFF,OFF,OFF,ON);
      break;   

   case 14:
      digitalWrite(HEATERPIN, HIGH);
      digitalWrite(HUMIDIFIERPIN, HIGH);
      digitalWrite(AIRPURIFIERPIN, HIGH);      
      digitalWrite(COOLERFAN, LOW);
      digitalWrite(DEHUMIDIFIERPIN, LOW);
      updateActuatorDisplay(OFF,OFF,OFF,ON,ON);
      break;   

   case 15:
      digitalWrite(HEATERPIN, HIGH);
      digitalWrite(HUMIDIFIERPIN, HIGH);
      digitalWrite(AIRPURIFIERPIN, LOW);      
      digitalWrite(COOLERFAN, HIGH);
      digitalWrite(DEHUMIDIFIERPIN, LOW);
      updateActuatorDisplay(OFF,OFF,ON,OFF,ON);
      break;   

   case 16:
      digitalWrite(HEATERPIN, LOW);
      digitalWrite(HUMIDIFIERPIN, HIGH);
      digitalWrite(AIRPURIFIERPIN, LOW);     
      digitalWrite(COOLERFAN, HIGH);
      digitalWrite(DEHUMIDIFIERPIN, LOW);
      updateActuatorDisplay(ON,OFF,ON,OFF,ON);
      break;

   case 17:
      digitalWrite(HEATERPIN, HIGH);
      digitalWrite(HUMIDIFIERPIN, HIGH);
      digitalWrite(AIRPURIFIERPIN, LOW);     
      digitalWrite(COOLERFAN, LOW);
      digitalWrite(DEHUMIDIFIERPIN, LOW);
      updateActuatorDisplay(OFF,OFF,ON,ON,ON);
      break;   
    default:;
  }
}


/****************************************************************************/
/*    getSensorStatus                                                       */
/*    desc:function to determine sensor status cd                           */
/****************************************************************************/
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

void updateTriaStatusDisplay(int operationModeCd){
  const int BUF_MAX = 22;
  char buf[BUF_MAX];

  strcpy_P(buf, (const char*) F("TRIA |Mode "));
  
  if(operationModeCd == LOGICAL){
     strcat_P(buf, (const char*)F("Logical..")); 
     
  } else if (operationModeCd == REINFORCE){
    strcat_P(buf, (const char*) F("Reinforce")); 
    
  } else {
    strcat_P(buf, (const char*) F("Paused...")); 
  }
  ssd1306_printFixed (0, 0, buf, STYLE_NORMAL);
}

void updateSensorDisplay(int type, float value, float lo, float hi){
  
  const int BUF_MAX = 22;
  char buf[BUF_MAX];
  const int VAL_MAX = 4;
  char val[VAL_MAX];
  
  if (type == TM) {
  strcpy_P(buf, (const char*) F("T:"));
  } else if (type == HU){
    strcpy_P(buf, (const char*) F("H:"));
  } else{
    strcpy_P(buf, (const char*) F("A:"));
  }
  
  dtostrf(value, 3, 1, val);
  strcat(buf, val);
  strcat_P(buf, (const char*) F(" ["));
  if (lo != -1){
  dtostrf(lo, 3, 1, val);
  strcat(buf, val);
  strcat_P(buf, (const char*) F("-"));
  }
  dtostrf(hi, 3, 1, val);
  strcat(buf, val);
  //strcat_P(buf, (const char*) F("] F"));  
 
  if (type == TM){   
     strcat_P(buf, (const char*) F("] F")); 
     ssd1306_printFixed (0, 16,buf, STYLE_NORMAL);
     
  } else if (type == HU){
    strcat_P(buf, (const char*) F("] rH")); 
    ssd1306_printFixed (0, 24,buf, STYLE_NORMAL);

  } else{
    strcat_P(buf, (const char*) F("] ppm"));
    ssd1306_printFixed (0, 32,"", STYLE_NORMAL);
    ssd1306_printFixed (0, 32,buf, STYLE_NORMAL);  
  }
}

void updateActuatorDisplay(int Ht, int Fn, int Hu, int Ap, int Dh ){
  log(String(Ht) +':'+String(Fn)+':'+String(Hu)+':'+String(Ap)+':'+String(Dh), INFO);
  if (Ht == ON) {ssd1306_fillRect(10,55, 20,60);} else { ssd1306_drawRect(10,55, 20,60); }
  if (Fn == ON) {ssd1306_fillRect(35,55, 45,60);} else {ssd1306_drawRect(35,55, 45,60);}
  if (Hu == ON) {ssd1306_fillRect(60,55, 70,60);} else {ssd1306_drawRect(60,55, 70,60);}
  if (Ap == ON) {ssd1306_fillRect(85,55, 95,60);} else {ssd1306_drawRect(85,55, 95,60);}
  if (Dh == ON) {ssd1306_fillRect(110,55, 120,60);} else {ssd1306_drawRect(110,55, 120,60);}
}

void log(String sMsg, int type ){
  if (logLevel == type){
     String dMsg = ">" + String(type) +" : " + sMsg;
     Serial.println(dMsg);
  }
}
