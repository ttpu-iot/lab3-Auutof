/*************************************************
 * LAB 3 – Exercise 2
 * ESP32 + MQTT LED Control + Sensors + LCD
 *************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

// Pin definitions
#define LIGHT_PIN      33
#define BUTTON_PIN     25
#define RED_LED_PIN    26
#define GREEN_LED_PIN  27
#define BLUE_LED_PIN   14
#define YELLOW_LED_PIN 12

// LCD object
hd44780_I2Cexp lcd;
const int LCD_COLS = 16;
const int LCD_ROWS = 2;

// WiFi and MQTT credentials
const char* ssid           = "Wokwi-GUEST";
const char* password       = "";
const char* mqtt_broker    = "mqtt.iotserver.uz";
const int   mqtt_port      = 1883;
const char* mqtt_username  = "userTTPU";
const char* mqtt_password  = "mqttpass";

// MQTT topics
const char* topic_light   = "ttpu/iot/Jaloliddin/sensors/light";
const char* topic_button  = "ttpu/iot/Jaloliddin/events/button";
const char* topic_led_red    = "ttpu/iot/Jaloliddin/led/red";
const char* topic_led_green  = "ttpu/iot/Jaloliddin/led/green";
const char* topic_led_blue   = "ttpu/iot/Jaloliddin/led/blue";
const char* topic_led_yellow = "ttpu/iot/Jaloliddin/led/yellow";

// Button & publishing variables
unsigned long lastPublishTime   = 0;
const long publishInterval      = 5000;
int lastButtonState             = LOW;
unsigned long lastDebounceTime  = 0;
const unsigned long debounceDelay = 50;

// WiFi and MQTT clients
WiFiClient espClient;
PubSubClient mqtt_client(espClient);

// ---------------- WiFi connection ----------------
void connectWifi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// ---------------- MQTT connection ----------------
void connectMQTT() {
  while (!mqtt_client.connected()) {
    Serial.println("Connecting to MQTT broker...");
    String clientId = "esp32-" + String(random(0xffff), HEX);
    if (mqtt_client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("MQTT connected!");
      mqtt_client.subscribe(topic_led_red);
      mqtt_client.subscribe(topic_led_green);
      mqtt_client.subscribe(topic_led_blue);
      mqtt_client.subscribe(topic_led_yellow);
    } else {
      Serial.print("MQTT connect failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" -> retry in 2 seconds");
      delay(2000);
    }
  }
}

// ---------------- MQTT callback ----------------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) message += (char)payload[i];
  message.trim();

  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, message);
  String state = "";
  if (!error && doc.containsKey("state")) {
    state = doc["state"].as<String>();
  } else {
    state = message; // fallback for plain "ON"/"OFF"
  }

  // Serial output
  Serial.print("MQTT Message on topic: ");
  Serial.println(topic);
  Serial.print("Payload: ");
  Serial.println(message);

  // LCD display
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Topic:");
  lcd.print(String(topic).substring(20)); // show LED color
  lcd.setCursor(0, 1);
  lcd.print("State:");
  lcd.print(state);

  // Control LEDs
  if (String(topic).endsWith("red"))    digitalWrite(RED_LED_PIN, state == "ON" ? HIGH : LOW);
  if (String(topic).endsWith("green"))  digitalWrite(GREEN_LED_PIN, state == "ON" ? HIGH : LOW);
  if (String(topic).endsWith("blue"))   digitalWrite(BLUE_LED_PIN, state == "ON" ? HIGH : LOW);
  if (String(topic).endsWith("yellow")) digitalWrite(YELLOW_LED_PIN, state == "ON" ? HIGH : LOW);
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);

  pinMode(LIGHT_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);

  // Initialize LCD
  if (lcd.begin(LCD_COLS, LCD_ROWS)) {
    Serial.println("LCD initialization failed!");
    while (1);
  }
  lcd.print("MQTT Ready!");
  delay(2000);
  lcd.clear();

  connectWifi();
  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setCallback(mqttCallback);
  connectMQTT();
}

// ---------------- Loop ----------------
void loop() {
  // --- Button event (debounced) ---
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState) lastDebounceTime = millis();
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != lastButtonState) {
      lastButtonState = reading;
      String eventType = (reading == HIGH) ? "PRESSED" : "RELEASED";
      String payload = "{\"event\": \"" + eventType + "\", \"timestamp\": " + String(time(NULL)) + "}";
      mqtt_client.publish(topic_button, payload.c_str());
    }
  }

  // --- Light sensor publishing ---
  if (millis() - lastPublishTime >= publishInterval) {
    int lightValue = analogRead(LIGHT_PIN);
    String payload = "{\"light\": " + String(lightValue) + ", \"timestamp\": " + String(time(NULL)) + "}";
    mqtt_client.publish(topic_light, payload.c_str());
    lastPublishTime = millis();
  }

  // --- Connection checks ---
  if (WiFi.status() != WL_CONNECTED) connectWifi();
  if (!mqtt_client.connected()) connectMQTT();
  mqtt_client.loop();
  delay(10);
}
