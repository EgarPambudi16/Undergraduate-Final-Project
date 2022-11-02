#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include <string.h>

Adafruit_AMG88xx amg;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

int scl_pin = 6;
int sda_pin = 7;

const char * ssid = "Phantom";
const char * pass = "zmkt5857";

const char * host = "192.168.109.223";
const int port = 8080;

char str[512];
char str_buff[10];

//create UDP instance
WiFiUDP udp;

void setup(){
    Wire.setPins(sda_pin, scl_pin);
    Wire.begin();
    
    Serial.begin(115200);
    Serial.println(F("AMG88xx test"));

    bool status;
    
    // default settings
    status = amg.begin();
    if (!status) {
        Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
        while (1);
    }
    
    Serial.println("-- Pixels Test --");
    Serial.println();
    delay(100); // let sensor boot up

    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED){
      delay(500);
      Serial.println("...");
    }

    Serial.print("WiFi connected with IP: ");
    Serial.println(WiFi.localIP());

  
  //Connect to the WiFi network
  WiFi.begin(ssid, pass);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop(){
  WiFiClient client;
  amg.readPixels(pixels);

  if (!client.connect(host, port)) {
    Serial.println("Connection to host failed");
    delay(1000);
    return;
  }
  Serial.println("Connected to server successful");
  str[0] = '\0';
  for(int i=AMG88xx_PIXEL_ARRAY_SIZE; i>=1; i--){
      sprintf(str_buff, "%.2f", pixels[i-1]);
      Serial.print(str_buff);
      Serial.print(", ");
      strcat(str, str_buff);
      strcat(str, ";");
      if((i-1)%8 == 0){
        Serial.println();
      }
    }
  client.print(str);
  delay(200);
}
