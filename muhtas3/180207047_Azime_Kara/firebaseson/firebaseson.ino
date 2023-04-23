#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>

//#include <Servo.h>
//Servo myservo;
//
//int pos = 0;

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "HUAWEI Mate 20 lite"
#define WIFI_PASSWORD "eac6a6c8217f"

// Insert Firebase project API Key
#define API_KEY "AIzaSyCFMn8DVa6L7OXC1tbvKaSJm5opNNDdEp8"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "muhtas3-smarthomesystems-default-rtdb.europe-west1.firebasedatabase.app/" 

//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
int count = 0;
bool signupOK = false;

int sicaklik=0;
int lamba=0;
int motor=0;
int nem=0;
int fan=0;
int gelenVeri[3];
int i=0;

int buton1 = 5;
int buton2 = 4;
int buton3 = 0;
int buton4 = 14;

int buton5 = 12;
int buton6 = 13;
int buton7 = 9;



void setup(){
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  pinMode(buton1,INPUT);
  pinMode(buton2,INPUT);
  pinMode(buton3,INPUT);
  pinMode(buton4,INPUT);

  pinMode(buton5,OUTPUT);
  pinMode(buton6,OUTPUT);

 //  myservo.attach(9);
}

void loop(){

  if (Firebase.RTDB.getInt(&fbdo, "/fan"))
  {
    if (fbdo.dataType() == "int") 
    {
      fan = fbdo.intData();
      Serial.println(fan);
    }

    if(fan==1)
    {
      digitalWrite(buton5,HIGH);
      Serial.println("fan açık");
    }
    else
    {
      digitalWrite(buton5,LOW);
      Serial.println("fan kapalı");
    }
  }

  if (Firebase.RTDB.getInt(&fbdo, "/lamba"))
  {
    if (fbdo.dataType() == "int") 
    {
      lamba = fbdo.intData();
      Serial.println(lamba);
    }

    if(lamba==1)
    {
      digitalWrite(buton6,HIGH);
      Serial.println("lamba açık");
    }
    else
    {
      digitalWrite(buton6,LOW);
      Serial.println("lamba kapalı");
    }
  }

  if (Firebase.RTDB.getInt(&fbdo, "/motor"))
  {
    if (fbdo.dataType() == "int") 
    {
      motor = fbdo.intData();
      Serial.println(motor);
    }

    if(motor==1)
    {
      digitalWrite(buton7,HIGH);
      Serial.println("motor açık");
    }
    else
    {
      digitalWrite(buton7,LOW);
      Serial.println("motor kapalı");
    }
  
  if(Serial.available())
  {   
    sicaklik = Serial.read();
    if(sicaklik !=0)
    {
      Firebase.RTDB.setInt(&fbdo, "sicaklik",sicaklik);
    }
    
    Serial.println(sicaklik);  
  }

  bool ldr = digitalRead(buton1); 
  if(ldr == true)
  {
    Firebase.RTDB.setInt(&fbdo, "ldr",1);
  }
  else
  {
    Firebase.RTDB.setInt(&fbdo, "ldr",0);
  }

  bool yagmur = digitalRead(buton2);
  if(yagmur == true)
  {
    Firebase.RTDB.setInt(&fbdo, "yagmur",1);
  }
  else
  {
    Firebase.RTDB.setInt(&fbdo, "yagmur",0);
  }
  
  bool pir = digitalRead(buton3); 
  if(pir == true)
  {
    Firebase.RTDB.setInt(&fbdo, "pir",1);
  }
  else
  {
    Firebase.RTDB.setInt(&fbdo, "pir",0);
  }
  
  bool touch = digitalRead(buton4); 
  if(touch == true)
  {
    Firebase.RTDB.setInt(&fbdo, "touch",1);
  }
  else
  {
    Firebase.RTDB.setInt(&fbdo, "touch",0);
  } 
 }
}
