#include <Arduino.h>
#include <WiFi.h>
#include <NewPing.h>
#include <FirebaseESP32.h>


#define FIREBASE_HOST "esp32-flutter-project.firebaseio.com"
#define FIREBASE_AUTH "sMuaV86cJhMXpk3htAaXfk3xkveaZ6CcqrL9tuW3"


//Define firebase Data object
FirebaseData firebaseData;


#define TRIGGER_PIN 12 //Ultrasonic Sensor HC-SR04
#define ECHO_PIN 13
#define MAX_DISTANCE 400
#define Buzzer 2

// NewPing setup of pins and maximum distance
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 

String path="/ESP32_Device";

int freq=2000;
int channel=0;
uint resolution1=8;


const char* ssid = "SHAW-BDF72C";
const char* password = "Saavi2019";


void iniFirebase();

void setup() {
  
  Serial.begin(9600);
  Serial.setDebugOutput(true);
  Serial.println("Hello...");
  
  ledcSetup(channel,freq,resolution1);
  ledcAttachPin(2,0);
  


  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  
  iniFirebase();
}


void iniFirebase()
{
  Firebase.begin(FIREBASE_HOST,FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

  //set database read timeout to 1 minute
  Firebase.setReadTimeout(firebaseData,1000*60);
  Firebase.setwriteSizeLimit(firebaseData,"tiny");
}

void loop() {
  //put your main code here, to run repeatedly:
  
  delay(1000);
   double distance = sonar.ping_cm();
   if(distance > 0){
   Serial.print(distance);
   Serial.println("cm");
   }

   Firebase.setDouble(firebaseData,path+"/Distance/Data",distance);

 //ledcWrite(channel, 125);
    if (distance > 0 && distance <400){
    ledcWrite(0, 125);
    ledcWriteTone(0,5000);
    delay(1000);
  }
  else {
     ledcWriteTone(0,0);
  }
}
