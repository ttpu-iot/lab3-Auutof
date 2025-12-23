/*************************************************
 * LAB 3 – Exercise 2
 * ESP32 + MQTT LED Control
 * Ready for Wokwi
 *************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

// LED pins
#define RED_LED_PIN     26
#define GREEN_LED_PIN   27
#define BLUE_LED_PIN    14
#define YELLOW_LED_PIN  12

// WiFi credentials
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// MQTT credentials
const char* mqtt_broker = "mqtt.iotserver.uz";
const int   mqtt_port   = 1883;
const char* mqtt_user   = "userTTPU";
const char* mqtt_pass   = "mqttpass";

// MQTT topics
const char* TOPIC_RED    = "ttpu/iot/Jaloliddin/led/red";
const char* TOPIC_GREEN  = "ttpu/iot/Jaloliddin/led/green";
const char* TOPIC_BLUE   = "ttpu/iot/Jaloliddin/led/blue";
const char* TOPIC_YELLOW = "ttpu/iot/Jaloliddin/led/yellow";

// WiFi and MQTT objects
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Connect to WiFi
void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// Connect to MQTT broker
void connectMQTT() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT broker...");
    String clientId = "esp32-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("MQTT connected!");
      mqttClient.subscribe(TOPIC_RED);
      mqttClient.subscribe(TOPIC_GREEN);
      mqttClient.subscribe(TOPIC_BLUE);
      mqttClient.subscribe(TOPIC_YELLOW);
    } else {
      Serial.print("MQTT connect failed, rc=");
      Serial.println(mqttClient.state());
      Serial.println("Retrying in 2 seconds...");
      delay(2000);
    }
  }
}

// Determine LED pin from topic
int getLedPin(const String& topic) {
  if (topic.endsWith("/red")) return RED_LED_PIN;
  if (topic.endsWith("/green")) return GREEN_LED_PIN;
  if (topic.endsWith("/blue")) return BLUE_LED_PIN;
  if (topic.endsWith("/yellow")) return YELLOW_LED_PIN;
  return -1;
}

// Callback when message arrives
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) message += (char)payload[i];
  message.trim(); // remove spaces or newlines

  Serial.print("[MQTT] Message on topic: ");
  Serial.println(topic);
  Serial.print("[MQTT] Payload: ");
  Serial.println(message);

  int ledPin = getLedPin(String(topic));
  if (ledPin == -1) return;

  // Try JSON parsing first
  String state = "";
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, message);
  if (!error && doc.containsKey("state")) {
    state = doc["state"].as<String>();
  } else {
    // If JSON fails, treat payload as plain "ON"/"OFF"
    state = message;
  }

  // Set LED
  if (state == "ON") digitalWrite(ledPin, HIGH);
  else if (state == "OFF") digitalWrite(ledPin, LOW);
  else {
    Serial.println("[WARN] Invalid payload, ignoring");
    return;
  }

  Serial.print("[LED] LED on pin ");
  Serial.print(ledPin);
  Serial.print(" set to ");
  Serial.println(state);
}

// Setup
void setup() {
  Serial.begin(115200);
  delay(500);

  // Initialize LEDs
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);

  connectWiFi();
  mqttClient.setServer(mqtt_broker, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  connectMQTT();
}

// Main loop
void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqttClient.connected()) connectMQTT();
  mqttClient.loop();
  delay(10); 
}
