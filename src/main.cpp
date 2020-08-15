#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <NewPing.h>
#include <FirebaseESP32.h>

// Select camera model

#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"

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


const char* ssid = "TELUS1005";
const char* password = "nfnb33pgv2";

void startCameraServer();
void iniWifi();

void setup() {
  
  Serial.begin(9600);
  Serial.setDebugOutput(true);
  Serial.println("Hello...");
  
  ledcSetup(channel,freq,resolution1);
  ledcAttachPin(2,0);
  

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
  
  iniWifi();
}


void iniWifi()
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
    if (distance > 0 && distance <100){
    ledcWrite(0, 125);
    ledcWriteTone(0,2000);
    delay(1000);
  }
  else if (distance >100 && distance <200){
    //ledcWrite(0, 125);
    ledcWriteTone(0,2500);
    delay(1000);
  }
  else if (distance >200 && distance <300){
    //ledcWrite(0, 125);
    ledcWriteTone(0,3000);
    delay(1000);
  }
  else if (distance >300 && distance <400){
    //ledcWrite(0, 125);
    ledcWriteTone(0,5000);
    delay(1000);
  }
  else {
     ledcWriteTone(0,0);
  }
}
