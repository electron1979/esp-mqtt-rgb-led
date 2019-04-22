/*
 * ESP8266 MQTT Lights for Home Assistant.
 *
 * Created DIY lights for Home Assistant using MQTT and JSON.
 * This project supports single-color, RGB, and RGBW lights.
 *
 * Copy the included `config-sample.h` file to `config.h` and update
 * accordingly for your setup.
 *
 * See https://github.com/corbanmailloux/esp-mqtt-rgb-led for more information.
 */

 /*
  * Sample Payloads:
  *   On          - {"state":"ON"}
  *   Off         - {"state":"OFF"}
  *   Flash       - {"flash":3}
  *   On & Flash  - {"state":"ON","flash":3}
  *   
  * Add to config.h:
  *   #define CONFIG_PIN_WHITE   12 // For RGB(W)
  *   #define PIR1  13
  *   #define PIR2  14
 */

// Set configuration options for LED type, pins, WiFi, and MQTT in the following file:
#include "config.h"

// https://github.com/bblanchon/ArduinoJson
///////// ******************           WAS USING Benoit Blanchon v5.13.4 **************************************************************************
///////// ******************           NOW TRY v5.13.5 **************************************************************************
#include <ArduinoJson.h>

///////// ******************           WAS USING esp8266 by Community v2.4.2
///////// ******************           STILL USE v2.4.2
#include <ESP8266WiFi.h>

// http://pubsubclient.knolleary.net/
#include <PubSubClient.h>

int status = WL_IDLE_STATUS;             // the Wifi radio's status
byte wiFiMAC[6];                         // the MAC address of your Wifi device

const int BUFFER_SIZE = JSON_OBJECT_SIZE(20);

uint8_t  change_pir1 = 0;
uint8_t  change_pir2 = 0;

bool stateOn = true;  // power-on stste

// Globals for flash
bool flash = false;
bool startFlash = false;
unsigned long flashLength = 0;
unsigned long flashStartTime = 0;

WiFiClient espClient;
PubSubClient client(espClient);


/////////// SET-UP ///////////////////////////////////////////////////////////////////////////////////////
void setup() {
  
  pinMode(CONFIG_PIN_WHITE, OUTPUT);
  pinMode(BUILTIN_LED, OUTPUT);
  if (!CONFIG_INVERT_LED_LOGIC) {
    digitalWrite(CONFIG_PIN_WHITE, stateOn);
    digitalWrite(BUILTIN_LED, !stateOn);
  }
  else {
    digitalWrite(CONFIG_PIN_WHITE, !stateOn);
    digitalWrite(BUILTIN_LED, stateOn);
}
  
  if (CONFIG_DEBUG) {
    Serial.begin(115200);
  }
  Serial.println('\n');
  delay(2000);

  WiFi.macAddress(wiFiMAC);
  Serial.print("MAC: ");
  Serial.print(wiFiMAC[0],HEX);
  Serial.print(":");
  Serial.print(wiFiMAC[1],HEX);
  Serial.print(":");
  Serial.print(wiFiMAC[2],HEX);
  Serial.print(":");
  Serial.print(wiFiMAC[3],HEX);
  Serial.print(":");
  Serial.print(wiFiMAC[4],HEX);
  Serial.print(":");
  Serial.println(wiFiMAC[5],HEX);
  
  setup_wifi();
  client.setServer(CONFIG_MQTT_HOST, CONFIG_MQTT_PORT);
  client.setCallback(callback);

  /// PIR ///
  pinMode(PIR1, INPUT);
  digitalWrite(PIR1, LOW);       // turn on pulldown resistors
  pinMode(PIR2, INPUT);
  digitalWrite(PIR2, LOW);       // turn on pulldown resistors

  if (!client.connected()) {
    reconnect();
  }

  char mqttMsg[] = "powerON";  
  Serial.print("Sending: ");
  Serial.println(mqttMsg);
  client.publish(CONFIG_MQTT_TOPIC_SET, mqttMsg);
  
  attachInterrupt(digitalPinToInterrupt(PIR1), pir1_STATE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIR2), pir2_STATE, CHANGE);
}


/////////// SET-UP WI-FI /////////////////////////////////////////////////////////////////////////////////
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(CONFIG_WIFI_SSID);

  WiFi.mode(WIFI_STA); // Disable the built-in WiFi access point.
  WiFi.begin(CONFIG_WIFI_SSID, CONFIG_WIFI_PASS);
  
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    status = WiFi.begin(CONFIG_WIFI_SSID, CONFIG_WIFI_PASS);
    Serial.print(".");
    delay(1000);
    yield();
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  yield();
}


//// CALLBACK ////////////////////////////////////////////////////////////////////////////////////////////
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]\t");

  char message[length + 1];
  for (int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
  }
  message[length] = '\0';
  Serial.println(message);

  if (!processJson(message)) {
    return;
  }
  yield();
}


//////// PROCESS JSON ////////////////////////////////////////////////////////////////////////////////////
bool processJson(char* message) {
  
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.parseObject(message);

  if (!root.success()) {
    Serial.println("parseObject() failed");
    return false;
  }

    // If "flash" is included, treat differently
  if (root.containsKey("flash"))  {
    flashLength = (int)root["flash"] * 1000;
    flash = true;
    startFlash = true;
  }
  
//  if (root.containsKey("state")) {      // react to 'state' in payload
  else if (root.containsKey("state")) {   // don't react to 'state' in payload
    if (strcmp(root["state"], CONFIG_MQTT_PAYLOAD_ON) == 0) {
      stateOn = true;
    }
    else if (strcmp(root["state"], CONFIG_MQTT_PAYLOAD_OFF) == 0) {
      stateOn = false;
    }
  }

  return true;
}



///// RECONNECT //////////////////////////////////////////////////////////////////////////////////////////
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect (don't need user/password for Mosquitto)
//    if (client.connect(CONFIG_MQTT_CLIENT_ID, CONFIG_MQTT_USER, CONFIG_MQTT_PASS)) {
    if (client.connect(CONFIG_MQTT_CLIENT_ID)) {
      Serial.print("connected to: ");
      Serial.println(CONFIG_MQTT_CLIENT_ID);
      client.subscribe(CONFIG_MQTT_TOPIC_SET);          // Lights
      client.publish(outTopic, CONFIG_MQTT_CLIENT_ID);  // PIRs
    } 
    else {
      Serial.print("failed, state: ");
      Serial.print(client.state());
      Serial.println(", try again in 5 seconds");
      delay(2000);  // Wait 2 seconds before retrying
    }
  }
}


///// LOOP ///////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  if (change_pir1) {
    change_pir1 = 0;
    char mqttMsg[1] = {0};
    if (digitalRead(PIR1)) {
      sprintf(mqttMsg, "pir1");  
      Serial.print("Sending: ");
      Serial.println(mqttMsg);
      client.publish(outTopic, mqttMsg);
      yield();
    }
  } 
  
  if (change_pir2) {
    change_pir2 = 0;
    char mqttMsg[1] = {0};
    if (digitalRead(PIR2)) {
      sprintf(mqttMsg, "pir2");  
      Serial.print("Sending: ");
      Serial.println(mqttMsg);
      client.publish(outTopic, mqttMsg);
      yield();
    }
  } 
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (flash) {
    if (startFlash) {
      startFlash = false;
      flashStartTime = millis();
    }
    if ((millis() - flashStartTime) <= flashLength) {
      if ((millis() - flashStartTime) % 1001 <= 500) {  // 1000 -> 1001 prevents relay toggle when flashing is finished
        if (!CONFIG_INVERT_LED_LOGIC) {
          digitalWrite(CONFIG_PIN_WHITE, !stateOn);
          digitalWrite(BUILTIN_LED, stateOn);
        }
        else {
          digitalWrite(CONFIG_PIN_WHITE, stateOn);
          digitalWrite(BUILTIN_LED, !stateOn);
        }
      }
      else {
        if (!CONFIG_INVERT_LED_LOGIC) {
          digitalWrite(CONFIG_PIN_WHITE, stateOn);
          digitalWrite(BUILTIN_LED, !stateOn);
        }
        else {
          digitalWrite(CONFIG_PIN_WHITE, !stateOn);
          digitalWrite(BUILTIN_LED, stateOn);
      }
    }
    else {
    flash = false;
    }
  }   // end Flash //////////////////////

  else  { // 'stateOn'
    if (!CONFIG_INVERT_LED_LOGIC) {
      digitalWrite(CONFIG_PIN_WHITE, stateOn);
      digitalWrite(BUILTIN_LED, !stateOn);
    }
    else {
      digitalWrite(CONFIG_PIN_WHITE, !stateOn);
      digitalWrite(BUILTIN_LED, stateOn);
    }
  }
  yield();
}



///// PIR ////////////////////////////////////////////////////////////////////////////////////////////
void pir1_STATE() {
  change_pir1 = 1;
}

void pir2_STATE() {
  change_pir2 = 1;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////


