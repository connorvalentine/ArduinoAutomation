  /*
  Features: Using blynk to automate Aquaponics rig
  1) Feed at certain times throughout the day
  1a) spins a stepper motor a certain # of steps
  1b) notifies me fish were fed at the given time
  2) water parameters updated every interval Turbidity, pH, temperature, TDS
  
/////////////////////////////////////////////////////
  Tasks:
  -) sync light buttons with virutal pin now
  -) add fish tank color light control (can we use the remote IR sensors from amazon?)
  -) test water sensor to see if its reliable over time. Don't want pump turning off randomly
  -) add pump control
  -) add flow meter for the pump?
  
/////////////////////////////////////////////////////
Turb: Pin Description 5V from board, not wall module
      Outpin to voltage splitter [need to shift 0-4.2 V output to 0-3.3V]
      using R1 = 2000 R2 = 7230. Outpin- [2kR- pinA2 - 7330Rchain]-ground 
      7320 == 
      
Motor:  Nema 17 stepper motor and TB6600 stepper motor driver
        Stepper Motor Driver TB6600
        12V 1 amp power source to the VCC (+) and Grnd (-) pins
        B-: red wire to motor
        B+: green wire to motor
        A-: blue wire to motor
        A+: yello wire to motor
        Dir-: Grnd
        Dir+: D8 (might need to switch to D14, hard to tell if I swapped D8 and D14)
        Pul-: Grnd
        Pul+: D14

Widge Virtual Pin #s
        V0    Terminal (must be v0)
        V1    Current Time
        V2    Current Date
        V3    H20 Temp
        V4    pH
        V5    TDS
        V6    Turbidity
        V7    Food Per Meal
        V8    Feed Fish Button command
        V9    Meals Today (numfeedings)
        V10   BreakFast Time (meal1)
        V11   Lunch Time (meal2)
        V12   Dinner Time (meal3)
        V13   Modnight Snack Time (meal4)
        V14   Grow Light timing
        V15   Fish Tank Light Timing
        V16   Grow light button sync
        V17   Fish Tank Light button sync
        V18   Water Sensor alert 
        V19   Number of Disconnections
        V20   Uptime counter since last disconnection
*/

////////////////////////////////////////////////////
/////// Initialization
////////////////////////////////////////////////////
//#define BLYNK_DEBUG
#define BLYNK_PRINT Serial /* Comment this out to disable prints and save space */

// List Libraries to include
#include <SPI.h>
#include <WiFiNINA.h>
#include <utility/wifi_drv.h> //controlling LED on board
#include <ArduinoOTA.h>
#include <BlynkSimpleWiFiNINA.h>
#include <TimeAlarms.h>
#include <WidgetRTC.h>
#include <OneWire.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "secrets.h" 

// define sensor pins 
const byte TDSSensorPin = A1,// TDS pin
           pHSensorPin = A0,   // pH Pin
           TurbSensorPin = A5, // turbidity sensor
           growLightPin = 1, // relay control for grow lights
           fishLightPin = 3, // fish tank light on or off
           thermocouplePin = 4, // temperature probe pin
           floodSensorPin = 9, //water floor sensor pin
           dirPin = 14, // stepper motor direction pin
           stepPin = 8; // stepper motor # of steps/control pin
           
AccelStepper stepper(1, stepPin, dirPin); // initialize the AccelStepper library for 2 pin control

/////// Wifi Settings /////// sensitive data in the Secret tab aka secrets.h
char ssid[] = SECRET_SSID;      // your network SSID (name)
char pass[] = SECRET_PASS;   // your network password
char auth[] = "bQHBh_KW__jSB9KQiVlnvOWePJQxBnJ1";
char server[] = "blynk-cloud.com";  // URL for Blynk Cloud Server
const short int port = 8080;

// initialize Blynk timer, time, and terminal widget
BlynkTimer timer;
WidgetRTC rtc; // real time clock widget needed
WidgetTerminal terminal(V0); // make sure it is same in the app V0

BLYNK_CONNECTED() { // this runs when blynk starts a new connection
  Blynk.syncAll();
  rtc.begin();// Synchronize time on connection
}

////////////////////////////////////////////////////
/////// functions for light control
////////////////////////////////////////////////////
// This is used in a few functions - easy way to report what
// time it is to the terminal widget.
String getTimeString(){
  String currentTime;
  if (hour() < 13){
    if (minute() <10){currentTime = String(hour()) + ":" +"0"+ minute() + " AM";}
    else {currentTime = String(hour()) + ":" + minute() + " AM";}
  }
  else{
    if (minute() <10){currentTime = String(hour()-12) + ":" +"0"+ minute() + " PM"; }
    else {currentTime = String(hour()-12) + ":" + minute() + " PM";}
  }
  return currentTime;
}

// grow light on and off times
AlarmId alarmGrowOn;
AlarmId alarmGrowOff;
BLYNK_WRITE(V14) {
  // clear old alarms
  Alarm.free(alarmGrowOn);
  Alarm.free(alarmGrowOff);
  
  TimeInputParam t(param);// Process start time
  if (t.hasStartTime()){
    alarmGrowOn = Alarm.alarmRepeat(t.getStartHour(),t.getStartMinute(),t.getStartSecond(), growLightsOn);
    String timeOn = String("Grow Lights Turn on at: ") + String(t.getStartHour()) + String(" hrs") + String(t.getStartMinute()) + String(" mins");
    terminal.println(timeOn);
    terminal.flush();
  }  
  if (t.hasStopTime()){
    alarmGrowOff = Alarm.alarmRepeat(t.getStopHour(),t.getStopMinute(),t.getStopSecond(), growLightsOff); 
    String timeOff = String("Grow Lights Turn off at: ") + String(t.getStopHour()) + String(" hrs") + String(t.getStopMinute()) + String(" mins");
    terminal.println(timeOff);
    terminal.flush();
  }
}

// fish tank light on and off times
AlarmId alarmFishOn;
AlarmId alarmFishOff;
bool growLightsOnFlag = false;
bool fishLightsOnFlag = false;

BLYNK_WRITE(V15) {
  // clear old alarms
  Alarm.free(alarmFishOn);
  Alarm.free(alarmFishOff);
  
  TimeInputParam t(param);// Process start time
  if (t.hasStartTime()){
    alarmFishOn = Alarm.alarmRepeat(t.getStartHour(),t.getStartMinute(),t.getStartSecond(), fishLightsOn);
    String timeOn = String("Fish Lights Turn on at: ") + String(t.getStartHour()) + String(" hrs") + String(t.getStartMinute()) + String(" mins");
    terminal.println(timeOn);
    terminal.flush();
  }
  if (t.hasStopTime()){
   alarmFishOff = Alarm.alarmRepeat(t.getStopHour(),t.getStopMinute(),t.getStopSecond(), fishLightsOff); 
   String timeOn = String("Fish Lights Turn off at: ") + String(t.getStopHour()) + String(" hrs") + String(t.getStopMinute()) + String(" mins");
   terminal.println(timeOn);
   terminal.flush();
  }
  //terminal.println(String("Total Alarms ") + String(Alarm.count()));
  //terminal.flush();
}

// functions that the alarms call to turn on/off thelights
void growLightsOn(){
  growLightsOnFlag = true;
  digitalWrite(growLightPin, HIGH);
  Blynk.virtualWrite(V16, 1);// sync light button state 
  // put times into the terminal 
  String timeString = getTimeString();
  String lightsOnString = String("Grow Lights turned on at ") + timeString;
  terminal.println(lightsOnString);
  terminal.flush();
}
void growLightsOff(){
  growLightsOnFlag = false;
  Blynk.virtualWrite(V16, 0);// sync light button state 
  // put times into the terminal 
  String timeString = getTimeString();
  String lightsOffString = String("Grow Lights turned off at ") + timeString;
  terminal.println(lightsOffString);
  terminal.flush();
}
void fishLightsOn(){
  fishLightsOnFlag = true;
  Blynk.virtualWrite(V17, 1); // sync light button state 
  // put times into the terminal 
  String timeString = getTimeString();
  String lightsFishOnString = String("Fish Lights turned on at ") + timeString;
  terminal.println(lightsFishOnString);
  terminal.flush();
}
void fishLightsOff(){
  fishLightsOnFlag = false;
  Blynk.virtualWrite(V17, 0); // sync light button state 
  // put times into the terminal 
  String timeString = getTimeString();
  String lightsFishOffString = String("Fish Lights turned off at ") + timeString;
  terminal.println(lightsFishOffString);
  terminal.flush();
}

void lightsMaintain(){
  if (growLightsOnFlag == true){digitalWrite(growLightPin, HIGH);}
  if (growLightsOnFlag == false){digitalWrite(growLightPin, LOW);}
  if (fishLightsOnFlag == true){digitalWrite(fishLightPin, HIGH);}
  if (fishLightsOnFlag == false){digitalWrite(fishLightPin, LOW);} 
}

////// lights on/off buttons
BLYNK_WRITE(V16){
  short int growLightsButtonValue = param.asInt(); // assigning incoming value from pin V8 to a variable
  if (growLightsButtonValue == 1){ growLightsOnFlag = true;}      // turn fish lights on
  if (growLightsButtonValue == 0){ growLightsOnFlag = false;}      // turn fish lights off 
}
BLYNK_WRITE(V17) {
  short int fishLightsButtonValue = param.asInt(); // assigning incoming value from pin V8 to a variable
  if (fishLightsButtonValue == 1){ fishLightsOnFlag = true;}      // turn fish lights on
  if (fishLightsButtonValue == 0){ fishLightsOnFlag = false;}      // turn fish lights off 
}
////////////////////////////////////////////////////
/////// functions for feeding 
////////////////////////////////////////////////////
////// Function that is activated to deliver meal at a given time 
short int numfeedings;
short int stepsPerMeal = 100;
void mealTime(){
    // Stepper motor defaults // stepper.setSpeed(500); // can set negative to go reverse direction
    stepper.setMaxSpeed(400);
    stepper.setAcceleration(200);
    stepper.moveTo(stepsPerMeal);  // Set the target position
    stepper.runToPosition(); // Go to target position with set dx and dx^2
    stepper.setCurrentPosition(0); // Reset the position to 0

    // put times of feeding into the terminal 
    String timeString = getTimeString();
    String mealTimeString = String("Fish fed at ") + timeString;
    terminal.println(mealTimeString );
    terminal.flush();
    
    numfeedings = numfeedings + 1;
    Blynk.virtualWrite(V9, numfeedings);
}

BLYNK_WRITE(V9) {
  numfeedings = param.asInt();
}

// Initialize default times for feeding
int time1; //= 32400; // 9 am
int time2; //= 50400; // 2 pm
int time3; //= 68400; // 7 pm
int time4; //= 82800; // 11 pm
AlarmId alarmMeal1;
AlarmId alarmMeal2;
AlarmId alarmMeal3;
AlarmId alarmMeal4;

// Function calls to change default feeding times 
BLYNK_WRITE(V10) {
  Alarm.free(alarmMeal1);
  time1 = param[0].asLong();
  alarmMeal1 = Alarm.alarmRepeat(hour(time1),minute(time1),second(time1),mealTime);
  }
BLYNK_WRITE(V11) {
  Alarm.free(alarmMeal2);
  time2 = param[0].asLong();
  alarmMeal2 = Alarm.alarmRepeat(hour(time2),minute(time2),second(time2),mealTime);
}
BLYNK_WRITE(V12) {
  Alarm.free(alarmMeal3);
  time3 = param[0].asLong();
  alarmMeal3 = Alarm.alarmRepeat(hour(time3),minute(time3),second(time3),mealTime);
}
BLYNK_WRITE(V13) {
  Alarm.free(alarmMeal4);
  time4 = param[0].asLong();
  alarmMeal4 = Alarm.alarmRepeat(hour(time4),minute(time4),second(time4),mealTime);
}

////// Function for input widget: set feed amount

BLYNK_WRITE(V7){
  short int pinValue = param.asInt(); // assigning incoming value from pin V7 to a variable
  // now write function to process received value
  stepsPerMeal = 50 + pinValue*10;
}

////// mealTime when app button is pressed
BLYNK_WRITE(V8){
  short int pinValue = param.asInt(); // assigning incoming value from pin V8 to a variable
  if (pinValue > 1) {
    mealTime();     // initialize stepper motor at 60 rpm
  } 
}

/////////////// Sensor FUNCTIONS/////////////////
// Sensor Functions
/////////////////////////////////////////////////

///// function: measure temperature
float temperature = 0;
OneWire ds(thermocouplePin);  // on digital pin 1
void measureTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius
  byte data[12];
  byte addr[8];
  if ( !ds.search(addr)) {
      ds.reset_search();} //no more sensors on chain, reset search
  if ( OneWire::crc8( addr, 7) != addr[7]) {
  //Serial.println("CRC is not valid!");
  }
  
  if ( addr[0] != 0x10 && addr[0] != 0x28) {
  //Serial.print("Device is not recognized");
  }
  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end
  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad
  for (short int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  ds.reset_search();
  byte MSB = data[1];
  byte LSB = data[0];
  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  temperature = (TemperatureSum*9/5) + 32; //C to F 
  Blynk.virtualWrite(V3, temperature);
}

///// function: measure tds
float tds = 0;  
const int numTDSReadings = 15;       // number of readings that we will average for each datapoint
float tdsArray[numTDSReadings];      // the readings from the analog input
int tdsIndex = 0;                 // the index of the current reading

void measureTDS() {
    short int TDS_sensor = analogRead(TDSSensorPin);
    float averageVoltage = TDS_sensor* 3.3 / 1023.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient=1.0+0.02*(((temperature-32)*5/9)-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
    tds=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
    tdsArray[tdsIndex] = tds; // save tds measurements to array
    
    // calculate the average of the array  
    float tdsSum = 0;
    byte i;
    for (i=0; i< numTDSReadings; i++)
    {
        tdsSum += tdsArray[i];
    }
    // add to the index, so we can overwrite the old values when we hit the end of the array
    ++tdsIndex;
    if (tdsIndex == numTDSReadings){
      tdsIndex = 0;
    }

  // push measurement to Blynk
  float tdsMeasurement = tdsSum/numTDSReadings;    
  Blynk.virtualWrite(V5, tdsMeasurement);
}

///// function: measure pH
float pH = 0;
const int numPHReadings = 15;       // number of readings that we will average for each datapoint
float pHArray[numPHReadings];      // the readings from the analog input
int pHIndex = 0;                 // the index of the current reading

void measurepH() {
  // first we take one pH reading
  short int PH_sensor = analogRead(pHSensorPin);
  float averageVoltagePH = PH_sensor*(3.3/1024);   // Convert the analog reading (readings are 0 - 1023) to (0-3.3 voltage)
  pH = -5.91*averageVoltagePH + 15.4; // calibration values - need to make a calibration linear regression 
  pHArray[pHIndex] = pH; // save pH measurements to array

  // calculate the average of the array  
  float pHSum = 0;
  byte i;
  for (i=0; i< numPHReadings; i++)
  {
      pHSum += pHArray[i];
  }
  // add to the index, so we can overwrite the old values when we hit the end of the array
  ++pHIndex;
  if (pHIndex == numPHReadings){
    pHIndex = 0;
  }

  // push measurement to Blynk
  float pHMeasurement = pHSum/numPHReadings;
  Blynk.virtualWrite(V4, pHMeasurement);
}

////// function: measure turbidity 
float turbidity = 0;
const int numTurbReadings = 15;       // number of readings that we will average for each datapoint
float turbArray[numTurbReadings];      // the readings from the analog input
int turbIndex = 0;                 // the index of the current reading

void measureTurb(){
    short int turbid_sensor = analogRead(TurbSensorPin);// turbidity sensor on A1 // Convert the analog reading (0 - 1023) to (0-3.3 voltage)
    float tv = turbid_sensor * (4.2/1024); // scale by 4.2 v (max for calibration standards from DFRobot... would normally do 3.3)
    turbidity = 4000*(1-(pow(2.3,(tv-4.2)))); // change so voltage is a turbidity measurement 
    turbArray[turbIndex] = turbidity; // save pH measurements to array

    // calculate the average of the array  
    float turbSum = 0;
    byte i;
    for (i=0; i< numTurbReadings; i++)
    {
        turbSum += turbArray[i];
    }
    // add to the index, so we can overwrite the old values when we hit the end of the array
    ++turbIndex;
    if (turbIndex == numTurbReadings){
      turbIndex = 0;
    }

    float turbMeasurement = turbSum/numTurbReadings;
    Blynk.virtualWrite(V6, turbMeasurement);
}

////// function: check for water on floor using water sensor
// top of water sensor is not waterproof. May need a work around
int waterVal;// this will change if water is detected
void checkForWater(){
    waterVal = digitalRead(floodSensorPin);
    SerialUSB.println(waterVal);
    if(waterVal==LOW){
      // if the sensor is not touching water, keep the pump on
    }
    else{ 
      // if the sensor is touching water, turn off pump, and notify
      Blynk.notify(String("FLOOD ALERT"));
      terminal.println(String("FLOOD ALERT"));
      terminal.flush();
   }
    Blynk.virtualWrite(V18, waterVal);
}
/////////////////////////////////////////////////////
/////////////// UPDATE FUNCTIONS///////////////////
// These are run at a regular intervals in void setup()
// timer.setInterval(interval,function_name)
/////////////////////////////////////////////////////
//

///// Reset function - can press button in Blynk and this will reset the arduino
void(* resetFunc) (void) = 0; //declare reset function @ address 0
BLYNK_WRITE(V21){
  short int pinValue = param.asInt(); 
  if (pinValue == 1) {
    resetFunc();  //call reset   
}
}


///// function: updates the time and date widgets on blynk app.
void updateTime(){
  // send time
  String timeString = getTimeString();
  Blynk.virtualWrite(V1, timeString);
  // send Date 
   String yr = String(year());
   yr = String(yr[2]) + String(yr[3]);
   String currentDate = String(month()) + "/" + day() + "/"  + yr;
   Blynk.virtualWrite(V2, currentDate); 
}

///// function: flagreset() sets the mealcount to 0 at midnight
void flagreset(){
  String yr = String(year());
  yr = String(yr[2]) + String(yr[3]);
  String currentDate = String(month()) + "/" + day() + "/"  + yr;
  terminal.println(String("***********") + currentDate + String("***********"));
  terminal.flush();
  numfeedings = 0;
  Blynk.virtualWrite(V9, numfeedings);
}

///// function in void loop() to reconnect to wifi. activated when the wifi signal is lost.
int DeviceLED = 2;
int ReCnctFlag = 0; // Reconnection Flag
int ReCnctCount = 0;

void UpTime() {
  Blynk.virtualWrite(V20, millis() / 60000);  // Send UpTime minutes to App
  Serial.print("UpTime: ");
  Serial.println(millis() / 1000);  // Send UpTime seconds to Serial
  // turn the onboard LED green to show it is connected
  WiFiDrv::analogWrite(25, 255);
  WiFiDrv::analogWrite(26, 0);
  WiFiDrv::analogWrite(27, 0);
}

////////////////////////////////////////////////////
////////////// VOID SETUP FUNCTION//////////////////
////////////////////////////////////////////////////
void setup() {
  WiFiDrv::pinMode(25, OUTPUT); //define green pin
  WiFiDrv::pinMode(26, OUTPUT); //define red pin
  WiFiDrv::pinMode(27, OUTPUT); //define blue pin
  // blynk start up
  Blynk.config(auth, server, port); 
  Blynk.begin(auth,ssid, pass,"blynk-cloud.com", 8080);
  Blynk.notify("Ladybird's Guardian has Awoken");
  terminal.clear();
  terminal.println(("-----------Ladybird's Gaurdian------------"));
  terminal.flush();
  
  // Update the sensors at odd intervals. This prevents crashes
  timer.setInterval(30069L, updateTime);
  timer.setInterval(1010L, measureTDS);
  timer.setInterval(1060L, measureTurb);
  timer.setInterval(1110L, measurepH);
  timer.setInterval(1160L, measureTemp);
  timer.setInterval(1210L, checkForWater);
  timer.setInterval(3310L, lightsMaintain);
  timer.setInterval(1170L,UpTime);

  // Everynight at 12:05 am meal count set to 0;
  Alarm.alarmRepeat(0,5,1,flagreset); 

  // Sync with Blynk every 900 milliseconds
  setSyncInterval(900);

  // set pin modes 
  pinMode(growLightPin, OUTPUT);
  pinMode(fishLightPin, OUTPUT); 
  pinMode(floodSensorPin, INPUT);

//  ArduinoOTA.setHostname("Loss of connection test");  // For OTA
//  ArduinoOTA.begin();  // For OTA
  // start the WiFi OTA library with internal (flash) based storage
//  ArduinoOTA.begin(WiFi.localIP(), "Arduino", "ladybird", InternalStorage);
}

void loop(){
    ArduinoOTA.poll(); // check for WiFi OTA updates
    timer.run(); //run timer  
    Alarm.delay(1000); //delay each alarm by 1 second after it is activated (helps with crashes)

    if (Blynk.connected()) {  // If connected run as normal
    Blynk.run();
  } else if (ReCnctFlag == 0) {  // If NOT connected and not already trying to reconnect, set timer to try to reconnect in 30 seconds
    ReCnctFlag = 1;  // Set reconnection Flag
    // turn the onboard LED red to show it is connected
    WiFiDrv::analogWrite(25, 0);
    WiFiDrv::analogWrite(26, 255);
    WiFiDrv::analogWrite(27, 0);
    Serial.println("Starting reconnection timer in 10 seconds...");
    timer.setTimeout(10000L, []() {  // Lambda Reconnection Timer Function
      ReCnctFlag = 0;  // Reset reconnection Flag
      ReCnctCount++;  // Increment reconnection Counter
      Serial.print("Attempting reconnection #");
      Serial.println(ReCnctCount);
      Blynk.connect();  // Try to reconnect to the server
    });  // END Timer Function
  }
}
