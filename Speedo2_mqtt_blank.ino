// A speedo for the cat wheel.

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h> 
#include <Wire.h>

const int hallPin = 3;  // Pin for the magentic Hall sensor
const int led = 15;      // Pin for an LED 
const float wheelCircumference = 2.98;  // Circumference of the wheel in meters
volatile int cnt = 0;     // Counts the number of wheel rotations
volatile unsigned long revolutions = 0;
unsigned long lastTime = 0;  // Time to calculate speed
float distance = 0.0;       // Total distance covered
float speed = 0.0;          // Speed in meters per second
volatile unsigned long lastTriggerTime = 0; // Last interrupt trigger time
const unsigned long debounceDelay = 100;    // Debounce delay in milliseconds

#define RESET_PIN 0

//set up the Wifi and MQTT
const char* ssid = "Wifi-name";
const char* password = "Wifi-password";
const char* mqttserver = "192.168.0.XX" //IP Address of your MQTT broker;
const int mqttPort = 1883; // I am using HomeAssistant
const char* mqttUser = "homeassistant";
const char* mqttPassword = "MQTT-password";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
const char* resetTopic = "catwheel/reset"; 

void count() {
  unsigned long currentTime = millis();
  if (currentTime - lastTriggerTime > debounceDelay) {
    cnt++;               // Increment the count every time the Hall sensor is triggered
    revolutions++;       // Increment the total revolutions count
    lastTriggerTime = currentTime;  // Update the last trigger time
  } 
}

void setup() {
  Serial.begin(115200);     
  setup_wifi();
  client.setServer(mqttserver, 1883);
  client.setCallback(mqttCallback);
  pinMode(hallPin, INPUT);   // Set Hall magnet pin as input
  pinMode(led, OUTPUT);      // Set LED pin as output (optional)
  attachInterrupt(digitalPinToInterrupt(hallPin), count, FALLING); // Trigger count on falling edge
}

void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

//Subscribe to the resetTopic, so that the counts can all be reset via an MQTT
void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      client.subscribe(resetTopic);
      Serial.println("Subscribed to reset topic");
    } else {
      delay(5000);
    }
  }
}

//Subscribe to the resetTopic, so that the counts can all be reset via an MQTT
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);

  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Message: ");
  Serial.println(message);

  if (message == "reset") {
    resetValues();
  }
}

void loop() {
  // Every 5 seconds, calculate speed and distance
  unsigned long currentTime = millis();
   
  if (currentTime - lastTime >= 5000) {  
    float timeInSeconds = (currentTime - lastTime) / 5000.0;
   

    // Calculate speed in meters per second
    speed = (cnt * wheelCircumference) / timeInSeconds;

    // Calculate total distance
    distance += cnt * wheelCircumference;
    
    // Go to the Write subroutine to output the results
    Write();

    // Reset the count and update the last time
    cnt = 0;
    lastTime = currentTime;
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  }
}

// activated when an MQTT is recieved on "reset"
void resetValues() {
  wheelRevolutions = 0;
  speed = 0;
  distance = 0;
  lastMsg = 0;
  Serial.println("All values have been reset");
}

//Writes the results and publishes them on MQTT topic catwheel/data, and to the serial monitor
void Write() {
  if (!client.connected()) {
    reconnect();
  }
    StaticJsonDocument<256> doc;
  doc["revolutions"] = revolutions;
  doc["current_speed"] = speed;
  doc["distance"] = distance;

  char jsonBuffer[256];
  serializeJson(doc, jsonBuffer);
  client.publish("catwheel/data", jsonBuffer);
  // Debug information for Serial Monitor
   Serial.println("Published MQTT Data:");
   Serial.print("Revolutions: ");
   Serial.println(revolutions);
   Serial.print("Current Speed: ");
   Serial.println(speed);
   Serial.print("Distance: ");
   Serial.println(distance);
   Serial.println(" -- ");
 }
