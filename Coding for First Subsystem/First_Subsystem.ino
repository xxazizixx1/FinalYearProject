/* First Subsystem By Azizi Bin Mohamad Tambi (1919661) */
//Library
#define BLYNK_PRINT Serial
#include "Adafruit_VL53L0X.h"
#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#include <BlynkSimpleEsp8266.h>

//Firebase Auth FYP
#define FIREBASE_HOST "smartmonitoring-1e0d2-default-rtdb.asia-southeast1.firebasedatabase.app"                    //Your Firebase Project URL goes here without "http:" , "\" and "/"
#define FIREBASE_AUTH "X38IBhJhvxhHZlCX2wuNuyzYFjf9zl0YZDmIqJL8" //Your Firebase Database Secret goes here
#define WIFI_SSID "Cocomelon"                                               //WiFi SSID to which you want NodeMCU to connect
#define WIFI_PASSWORD "azizi123"

// Declare the Firebase Data object in the global scope
FirebaseData firebaseData;

//Address for TOF
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x29
#define SHT_LOX1 0 //Pin 7 nodemcu
#define SHT_LOX2 2 //Pin 6 nodemcu
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

//Pin Assignment
#define trig1 14
#define echo1 12
#define trig2 13
#define echo2 15

//Variables
#define Sound_Velocity 0.034
long duration;
double tof1, tof2, us1, us2, fsr_1, fsr_2, fsr_3;

//Iteration for filter
const int iteration = 20; //Do iteration for 20 times
float RunningAverageBuffer[iteration];
int NextRunningAverage;

//Iteration for FSR
const int iteration2 = 20;
float RunningAverageBufferFSR[iteration2];
int NextRunningAverage2;

//BLYNK AUTH TOKEN (Project setting, nut icon)
char auth[] = "UPzZ5qT34BBNmniTzLfPlDaOI0IV7nGA";
BlynkTimer timer;

//Wifi + Password
// char ssid[] = "GreenTea";
// char pass[] = "azizi1999";
char ssid[] = "WAIBWJ";
char pass[] = "ezuddin97";

void setup() {
  Serial.begin(115200);

  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  
  //FIREBASE SETUP
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);         //try to connect with wifi
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Serial.print("IP Address is : ");
  Serial.println(WiFi.localIP());             //print local IP address
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  delay(1000); 
  
  //BLYNK SETUP
  Blynk.begin(auth, ssid, pass, IPAddress(139,59,224,74), 8080);
  timer.setInterval(1000L, allVal); //Set 1s interval to upload data (<=10data)

  //Intialize TOF
  while (! Serial) {
    delay(1); 
    }
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
  //(SHT_LOX2, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));
  
  
  Serial.println(F("Starting..."));
  setID_XSHUT();

}

////////////////////////////////////////FX for ULTRASONIC SENSORS//////////////////////////////
void Ultrasonic1() //UNCOMMENT WHEN SENSOR ARRIVE
{
  // Clears the trigPin
  digitalWrite(trig1, LOW); //Choose desired trigger pin
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trig1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig1, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echo1, HIGH);  //choose desired echo pin
  
  // Calculate the distance
  float distanceCm = duration * Sound_Velocity/2;

  //Upload Firebase
  Firebase.setInt(firebaseData, "/System_1/Ultrasonic/US1(cm) ", distanceCm);

  // Prints the distance on the Serial Monitor
  Serial.println();
  Serial.println("///////SYSTEM 1///////");
  Serial.print("Distance(cm) Ultrasonic 1:"); 
  Serial.println(distanceCm);
  
  //Write to Blynk (V0)
  Blynk.virtualWrite(V0, distanceCm);
  
  delay(1000);
  us1 = distanceCm;
  //return distanceCm;
}

void Ultrasonic2()
{
  // Clears the trigPin
  digitalWrite(trig2, LOW); //Choose desired trigger pin
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trig2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig2, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration2 = pulseIn(echo2, HIGH);  //choose desired echo pin
  
  // Calculate the distance
  float distanceCm2 = duration2 * Sound_Velocity/2;

  //Upload Firebase
  Firebase.setInt(firebaseData, "/System_1/Ultrasonic/US2(cm)/", distanceCm2);
  
  // Prints the distance on the Serial Monitor
  Serial.print("Distance(cm) Ultrasonic 2: "); 
  Serial.println(distanceCm2);

  //Write to Blynk (V1)
  Blynk.virtualWrite(V1, distanceCm2);
  
  delay(1000);
  us2 = distanceCm2;
  //return distanceCm2;
}

////////////////////////////////////////FX for TOF//////////////////////////////

void setID_XSHUT()
{

 // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
} 

void TimeOfFlight1()
{
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  if(measure1.RangeStatus != 4) {     // if not out of range
    Serial.print("Distance TOF1(cm): "); Serial.println(measure1.RangeMilliMeter/10);
  } else {
    Serial.print(F("Out of range"));
  }
  
  //Upload Firebase
  Firebase.setInt(firebaseData, "/System_1/Time Of Flight/TOF_1(cm) ", measure1.RangeMilliMeter/10);
  
  //Write value to V2 Blynk
  Blynk.virtualWrite(V2, measure1.RangeMilliMeter/10); 
  
  tof1 = measure1.RangeMilliMeter/10;
  //return measure1.RangeMilliMeter/10; //Return value in CM
}

void TimeOfFlight2()
{
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  if(measure2.RangeStatus != 4) {     // if not out of range
    Serial.print("Distance TOF2(cm): "); Serial.println(measure2.RangeMilliMeter/10);
  } else {
    Serial.print(F("Out of range"));
  }
  
  //Upload Firebase
  Firebase.setInt(firebaseData, "/System_1/Time Of Flight/TOF_2(cm) ", measure2.RangeMilliMeter/10);
  
  //Write value to V2 Blynk
  Blynk.virtualWrite(V3, measure2.RangeMilliMeter/10); 
  
  tof2 = measure2.RangeMilliMeter/10;
  //return measure2.RangeMilliMeter/10; //Return value in CM
} 

/////////////////////////////////////////////////////APPLY FILTER+SENSOR FUSION////////////////////
void fusionFilterDistance()
{
  //float data = (TimeOfFlight1()+TimeOfFlight2()+Ultrasonic1()+Ultrasonic2())/4;
  float data = (tof1+tof2+us1+us2)/4;

  RunningAverageBuffer[NextRunningAverage++] = data;

  if(NextRunningAverage >= iteration)
  {
    NextRunningAverage = 0;
  }

  float RunningAverageData = 0;
  for(int i=0; i<iteration; ++i)
  {
    RunningAverageData += RunningAverageBuffer[i];
  }
  RunningAverageData /= iteration;
  
  Serial.println("Sensor fusion distance(cm): "); 
  Serial.print(data);

  //Upload Firebase
  Firebase.setInt(firebaseData, "/Sensor_Fusion/System1(cm)/", data);
  
  //Write value to V2 Blynk
  Blynk.virtualWrite(V4, data);  
  delay(1000);

  //Level of rubbish bin
  //Max distance is 0-50cm (from top view)
  // 2 = 1023
  // Bits = 21.29*data - 41.58
  float rubbish_level = 1023-((21.29*data)-41.58);
  Blynk.virtualWrite(V20, rubbish_level);
  delay(1000);
}

void fsr1()
{
  int fsr1;
    //Get Data from FSR 1
  if (Firebase.getInt(firebaseData, "/System_2/FSR/FSR_1(g)")) {

      if (firebaseData.dataTypeEnum() == fb_esp_rtdb_data_type_integer) {
      //Serial.println(fbdo.to<int>());
      fsr1 = firebaseData.intData();
      Blynk.virtualWrite(V5, fsr1);
      Serial.println("///////SYSTEM 2///////"); 
      Serial.print("FSR1 (g): ");
      Serial.println(fsr1);
      fsr_1 = fsr1;
    }
  }
  fsr1 = firebaseData.intData();
  Blynk.virtualWrite(V5, fsr1);
  Serial.println("///////SYSTEM 2///////"); 
  Serial.print("FSR1 (g): ");
  Serial.println(fsr1);
  fsr_1 = fsr1;
  
}
void fsr2()
{
  int fsr2;
  //Get Data from FSR 2
  if (Firebase.getInt(firebaseData, "/System_2/FSR/FSR_2(g)")) {

      if (firebaseData.dataTypeEnum() == fb_esp_rtdb_data_type_integer) {
      //Serial.println(fbdo.to<int>());
      fsr2 = firebaseData.intData();
      Blynk.virtualWrite(V6, fsr2);
      Serial.print("FSR2 (g): ");
      Serial.println(fsr2);
      fsr_2 = fsr2;
    }
  }
  fsr2 = firebaseData.intData();
  Blynk.virtualWrite(V6, fsr2);
  Serial.print("FSR2 (g): ");
  Serial.println(fsr2);
  fsr_2 = fsr2;

}
void fsr3()
{
  int fsr3;

  //Get Data from FSR 3
  if (Firebase.getInt(firebaseData, "/System_2/FSR/FSR_3(g)")) {

      if (firebaseData.dataTypeEnum() == fb_esp_rtdb_data_type_integer) {
      //Serial.println(fbdo.to<int>());
      fsr3 = firebaseData.intData();
      Blynk.virtualWrite(V7, fsr3);
      Serial.print("FSR3 (g): ");
      Serial.println(fsr3);
      fsr_3 = fsr3;
    }
  }
  fsr3 = firebaseData.intData();
  Blynk.virtualWrite(V7, fsr3);
  Serial.print("FSR3 (g): ");
  Serial.println(fsr3);
  fsr_3 = fsr3;

}

void fusionFSR()
{
  fsr1();
  fsr2();
  fsr3();
  float dataFSR = (fsr_1 + fsr_2 + fsr_3)/3;
  Serial.println();
  Serial.print("Sensor fusion FSR(g): "); 
  Serial.println(dataFSR);

  //Upload Firebase
  Firebase.setInt(firebaseData, "/Sensor_Fusion/System2(g)/", dataFSR);
  RunningAverageBufferFSR[NextRunningAverage2++] = dataFSR;

  if(NextRunningAverage2 >= iteration2)
  {
    NextRunningAverage2 = 0;
  }

  float RunningAverageFSR = 0;
  for(int j=0; j<iteration2; ++j)
  {
    RunningAverageFSR += RunningAverageBufferFSR[j];
  }
  RunningAverageFSR /= iteration2;


  
  //Write value to V2 Blynk
  Blynk.virtualWrite(V8, dataFSR);  
  delay(1000);

}

void prediction()
{
  float prediction_distance, prediction_mass;
  //Get Data from distance prediction
  if (Firebase.getInt(firebaseData, "/Prediction/Output/Distance (cm)")) {

      if (firebaseData.dataTypeEnum() == fb_esp_rtdb_data_type_integer) {
      //Serial.println(fbdo.to<int>());
      prediction_distance = firebaseData.intData();
      Blynk.virtualWrite(V30, prediction_distance);
      Serial.println("///////Prediction///////"); 
      Serial.print("Prediction_Distance(cm): ");
      Serial.println(prediction_distance);
    }
      prediction_distance = firebaseData.intData();
      Blynk.virtualWrite(V30, prediction_distance);
      Serial.println("///////Prediction///////"); 
      Serial.print("Prediction_Distance(cm): ");
      Serial.println(prediction_distance);
  }
  //Max distance is 0-50cm (from top view)
  float rubbish_level2 = 1023-((21.29*prediction_distance)-41.58);
  Blynk.virtualWrite(V21, rubbish_level2);


  //Get Data from mass prediction
  if (Firebase.getInt(firebaseData, "/Prediction/Output/Mass (g)")) {

      if (firebaseData.dataTypeEnum() == fb_esp_rtdb_data_type_integer) {
      //Serial.println(fbdo.to<int>());
      prediction_mass = firebaseData.intData();
      Blynk.virtualWrite(V31, prediction_mass);
      Serial.print("Prediction_Mass(g): ");
      Serial.println(prediction_mass);
    }
      prediction_mass = firebaseData.intData();
      Blynk.virtualWrite(V31, prediction_mass);
      Serial.print("Prediction_Mass(g): ");
      Serial.println(prediction_mass);
  }  
}

/////////////////////////////////////////////////////Upload all to BLYNK + FIREBASE///////////////////////////
void allVal()
{

  Ultrasonic1(); //Ultrasonic 1 (V0)
  Ultrasonic2(); //Ultrasonic 2 (V1)
  TimeOfFlight1();  //Time of flight 1 (V2)
  TimeOfFlight2(); //Time of flight 2 (V3)
  fusionFilterDistance(); //Sensor Fusion (V4)
  fusionFSR();
  prediction();
  //FSR 1 (V5)
  //FSR 2 (V6)
  //FSR 3 (V7)
  //Sensor Fusion FSR (V8)
}

void loop() {
  Blynk.run();
  timer.run();
}
