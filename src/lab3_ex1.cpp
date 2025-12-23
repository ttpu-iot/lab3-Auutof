// - RED LED - `D26`
// - Green LED - `D27`
// - Blue LED - `D14`
// - Yellow LED - `D12`

// - Button (Active high) - `D25`
// - Light sensor (analog) - `D33`

// - LCD I2C - SDA: `D21`
// - LCD I2C - SCL: `D22`

/**************************************
 * LAB 3 - EXERCISE 1: 
 * 
 * I want to publish message to mqtt every 5 second
 **************************************/

#include "Arduino.h"
#include "WiFi.h"
#include "PubSubClient.h"
#include <ArduinoJson.h>



// Your code here - global declarations
const int LIGHT_SENSOR_PIN = 33; // Pin for light sensor
const int BUTTON_PIN = 25;       // Pin for button


// WiFi credentials
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// MQTT Broker settings
const char* mqtt_broker = "mqtt.iotserver.uz";  // Free public MQTT broker
const int mqtt_port = 1883;
const char* mqtt_username = "userTTPU";  // username given in the telegram group
const char* mqtt_password = "mqttpass";  // password given in the telegram group

const char* mqtt_topic_sensor = "ttpu/iot/Jaloliddin/sensors/light"; // Topic for light sensor data
const char* mqtt_topic_button = "ttpu/iot/Jaloliddin/events/button"; // Topic for button press events


WiFiClient espClient;
PubSubClient mqtt_client(espClient);

// Function to connect to WiFi
void connectWiFi();
// Function to connect/reconnect to MQTT broker
void connectMQTT();

/*************************
 * SETUP
 */
void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n===== Lab 3 - Exercise 1 =====");

    pinMode(BUTTON_PIN, INPUT);

      // Connect to WiFi
    
    connectWiFi();
    
    // Setup MQTT
    mqtt_client.setServer(mqtt_broker, mqtt_port);

    // Connect to MQTT broker
    connectMQTT();
}


/*************************
 * LOOP
 */ 
void loop() 
{

        // Check WiFi connection
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected! Reconnecting...");
        connectWiFi();
    }
    
    // Check MQTT connection
    if (!mqtt_client.connected()) {
        Serial.println("MQTT disconnected! Reconnecting...");
        connectMQTT();
    }
    
    // Process incoming MQTT messages
    mqtt_client.loop();
    static unsigned long lastSensorReadTime = 0;
    const unsigned sensorReadInterval = 5000; 

    unsigned long currentTime = millis();
    if (currentTime - lastSensorReadTime >= sensorReadInterval) {
        lastSensorReadTime = currentTime;

        int sensorValue = analogRead(LIGHT_SENSOR_PIN); 
        Serial.print("Light Sensor Value: ");
        Serial.println(sensorValue);

        //Publish to MQTT 
        
        JsonDocument doc;
        doc["light"] = sensorValue;
        doc["timestamp"] = millis();

        char sensorMessage[256];
        serializeJson(doc, sensorMessage);

        if (mqtt_client.publish(mqtt_topic_sensor, sensorMessage))
        {
            Serial.println("Sensor value published MQTT!");
        } 
        else {
            Serial.println("Failed to publish sensor value to MQTT!");
        }
        
        
    }

    //DETECT BUTTON PRESS
    static int lastButtonState = LOW;
    int currentButtonState = digitalRead(BUTTON_PIN);
    if (currentButtonState != lastButtonState){
        lastButtonState = currentButtonState;
        String buttonStr = "";

        if (currentButtonState == HIGH) {
            buttonStr = "PRESSED";
            Serial.println("Button Pressed");
        } 
        else {
            buttonStr = "Released";
            Serial.println("Button Released");
        }

        JsonDocument doc;
        doc["event"] = buttonStr;
        doc["timestamp"] = millis();

        char btnEventMsg[256];
        serializeJson(doc, btnEventMsg);

        // Publish button event to MQTT
        if (mqtt_client.publish(mqtt_topic_button, btnEventMsg)) {
            Serial.println("Button event published to MQTT!");
        } else {
            Serial.println("Failed to publish button event to MQTT!");
        }

        delay(50); // For debouncing purpose
    }
}

/*************************
 * FUNCTIONS
 */
// Function to connect to WiFi
void connectWiFi(void) {
  Serial.println("\nConnecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

//--------------------------
// Function to connect/reconnect to MQTT broker
void connectMQTT(void) {
  while (!mqtt_client.connected()) {
    Serial.println("Connecting to MQTT broker...");
    
    String client_id = "esp32-client-" + String(WiFi.macAddress());
    
    if (mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT broker!");
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.println(mqtt_client.state());
      Serial.println("Retrying in 5 seconds...");
      delay(5000);
    }
  }
}