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

#include <DHT.h>
// Initialize DHT sensor 
// NOTE: For working with a faster than ATmega328p 16 MHz Arduino chip, like an ESP8266,
// you need to increase the threshold for cycle counts considered a 1 or 0.
// You can do this by passing a 3rd parameter for this threshold.  It's a bit
// of fiddling to find the right value, but in general the faster the CPU the
// higher the value.  The default for a 16mhz AVR is a value of 6.  For an
// Arduino Due that runs at 84mhz a value of 30 works.
// This is for the ESP8266 processor on ESP-01 
DHT dht(DHTPIN, DHTTYPE, 11); // 11 works fine for ESP8266
float humidity, temp_c;  // Values read from sensor
const long interval = 5000;              // interval at which to read sensor

// Generally, you should use "unsigned long" for variables that hold time
unsigned long previousMillis = 0;        // will store last temp was read
int status = WL_IDLE_STATUS;             // the Wifi radio's status
byte wiFiMAC[6];                         // the MAC address of your Wifi shield

uint8_t  change = 0;

const bool rgb = (CONFIG_STRIP == RGB) || (CONFIG_STRIP == RGBW);
const bool includeWhite = (CONFIG_STRIP == BRIGHTNESS) || (CONFIG_STRIP == RGBW);

const int BUFFER_SIZE = JSON_OBJECT_SIZE(20);

// Maintained state for reporting to HA
byte red = 255;
byte green = 255;
byte blue = 255;
byte white = 255;
byte brightness = 255;

// Real values to write to the LEDs (ex. including brightness and state)
byte realRed = 0;
byte realGreen = 0;
byte realBlue = 0;
byte realWhite = 0;

bool stateOn = false;

// Globals for fade/transitions
bool startFade = false;
unsigned long lastLoop = 0;
uint16_t transitionTime = 0;
bool inFade = false;
int loopCount = 0;
float stepR, stepG, stepB, stepW; // pb
float redVal, grnVal, bluVal, whtVal; // pb

// Globals for flash
bool flash = false;
bool startFlash = false;
int flashLength = 0;
unsigned long flashStartTime = 0;
byte flashRed = red;
byte flashGreen = green;
byte flashBlue = blue;
byte flashWhite = white;
byte flashBrightness = brightness;

// Globals for colorfade
bool colorfade = false;
int currentColor = 0;
// {red, grn, blu, wht}
const byte colors[][4] = {
  {255, 0, 0, 0},
  {0, 255, 0, 0},
  {0, 0, 255, 0},
  {255, 80, 0, 0},
  {163, 0, 255, 0},
  {0, 255, 255, 0},
  {255, 255, 0, 0}
};
const int numColors = 7;

WiFiClient espClient;
PubSubClient client(espClient);


/////////// SET-UP ///////////////////////////////////////////////////////////////////////////////////////
void setup() {
  if (CONFIG_DEBUG) {
    Serial.begin(115200);
  }
  Serial.println('\n');

  setup_wifi();
  client.setServer(CONFIG_MQTT_HOST, CONFIG_MQTT_PORT);
  client.setCallback(callback);
//  Serial.println("MQTT client set");

  
  /// Washing Machine ///
  pinMode(pausePin, OUTPUT);
  pinMode(onPin, INPUT);
      
  char mqttMsg[1] = {0};
  uint8_t washOFF = !digitalRead(onPin);
  sprintf(mqttMsg, "%s%d", mqttMsg, washOFF);  
  Serial.print("Sending: ");
  Serial.println(mqttMsg);
  if (!client.connected()) {
    reconnect();
  }
  
//  client.publish(outTopic2, mqttMsg);
//  
//  attachInterrupt(digitalPinToInterrupt(onPin), washMachineSTATE, CHANGE);

  
  /// RGB Light Strip ///
  if (rgb) {
    pinMode(CONFIG_PIN_RED, OUTPUT);
    pinMode(CONFIG_PIN_GREEN, OUTPUT);
    pinMode(CONFIG_PIN_BLUE, OUTPUT);
  }
  if (includeWhite) {
    pinMode(CONFIG_PIN_WHITE, OUTPUT);
  }

  // Set the BUILTIN_LED based on the CONFIG_LED_BUILTIN_MODE
  switch (CONFIG_LED_BUILTIN_MODE) {
    case 0:
      pinMode(LED_BUILTIN, OUTPUT);
      digitalWrite(LED_BUILTIN, LOW);
      break;
    case 1:
      pinMode(LED_BUILTIN, OUTPUT);
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    default: // Other options (like -1) are ignored.
      break;
  }
  analogWriteRange(255);
  

//  WiFi.macAddress(wiFiMAC);
//  Serial.print("MAC: ");
//  Serial.print(wiFiMAC[0],HEX);
//  Serial.print(":");
//  Serial.print(wiFiMAC[1],HEX);
//  Serial.print(":");
//  Serial.print(wiFiMAC[2],HEX);
//  Serial.print(":");
//  Serial.print(wiFiMAC[3],HEX);
//  Serial.print(":");
//  Serial.print(wiFiMAC[4],HEX);
//  Serial.print(":");
//  Serial.println(wiFiMAC[5],HEX);
  
  dht.begin();           // initialize temperature sensor
//  Serial.println("dht begun");
}


/////////// SET-UP WI-FI /////////////////////////////////////////////////////////////////////////////////
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(CONFIG_WIFI_SSID);

  WiFi.mode(WIFI_STA); // Disable the built-in WiFi access point.
//  WiFi.disconnect();
  WiFi.begin(CONFIG_WIFI_SSID, CONFIG_WIFI_PASS);

//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.print(".");
//  }

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
//    WiFi.disconnect();
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
//////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*
  SAMPLE PAYLOAD (BRIGHTNESS):
    {
      "brightness": 120,
      "flash": 2,
      "transition": 5,
      "state": "ON"
    }

  SAMPLE PAYLOAD (RGBW):
    {
      "brightness": 120,
      "color": {
        "r": 255,
        "g": 100,
        "b": 100
      },
      "white_value": 255,
      "flash": 2,
      "transition": 5,
      "state": "ON",
      "effect": "colorfade_fast"
    }
  */


//void washMachine(char* topic2, byte* payload, unsigned int length) {
//  digitalWrite(pausePin, HIGH);
//  delay(100);
//  digitalWrite(pausePin, LOW);
//  }


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

// ENVIRON //////////////////
//  if(message == "getEnv"){
  if(strstr(message, "getEnv")) {
    Serial.println("getEnv REQ received");
    humidity = -1;
    temp_c = -127;
    gettemperature();
    dtostrf(temp_c , 2, 1, message);
    char tempPB[] = "temp:";
    strcat( tempPB, message );
    client.publish(outTopic, tempPB);
    Serial.print("\tSending temperature: ");
    Serial.println(temp_c);
    Serial.print("\tMsg: ");
    Serial.println(tempPB);
    
    dtostrf(humidity , 2, 0, message);
    char humidPB[] = "humidity:";
    strcat( humidPB, message );
    client.publish(outTopic, humidPB);
    Serial.print("\tSending humidity: ");
    Serial.println(humidity);
    Serial.print("\tMsg: ");
    Serial.println(humidPB);
  }
  else if(strstr(message, "temp")) {
    Serial.print("\tTemp: ");
    Serial.println(message);
  }
  else if(strstr(message, "humid")) {
    Serial.print("\tHumidity: ");
    Serial.println(message);
  }
  
// WASHING MACHINE ////////////
  else if(strstr(message, "pause")) {
    digitalWrite(pausePin, HIGH);
    Serial.println("HIGH");
    delay(500);
    digitalWrite(pausePin, LOW);
    Serial.println("LOW");
    yield();
  }
// LIGHTS //////////////////
  else if (!processJson(message)) {
    return;
  }
  yield();

  if (stateOn) {
    // Update lights
    Serial.println("StateOn: Update lights");
    realRed = map(red, 0, 255, 0, brightness);
    realGreen = map(green, 0, 255, 0, brightness);
    realBlue = map(blue, 0, 255, 0, brightness);
    realWhite = map(white, 0, 255, 0, brightness);
    
    Serial.print("realRed: ");
    Serial.print(realRed);
    Serial.print(", realGreen: ");
    Serial.print(realGreen);
    Serial.print(", realBlue: ");
    Serial.println(realBlue);
    Serial.print("red: ");
    Serial.print(red);
    Serial.print(", green: ");
    Serial.print(green);
    Serial.print(", blue: ");
    Serial.println(blue);
  }
  else {
    realRed = 0;
    realGreen = 0;
    realBlue = 0;
    realWhite = 0;
  }

  yield();
  startFade = true;
  inFade = false; // Kill the current fade

//  if (USE_sendState){
//    sendState();
//  }
}


//////// PROCESS JSON ////////////////////////////////////////////////////////////////////////////////////
bool processJson(char* message) {

//  int number = (int) strtol( &message[0], NULL, 16);
//  // Split them up into r, g, b values
//  realRed = byte(number >> 16);
//  realGreen = byte(number >> 8 & 0xFF);
//  realBlue = byte(number & 0xFF);
//
//
////  Serial.print("msg: ");
////  Serial.println(message);
//  Serial.print("r: ");
//  Serial.print(realRed);
//  Serial.print(", g: ");
//  Serial.print(realGreen);
//  Serial.print(", b: ");
//  Serial.println(realBlue);
//
//  setColor(realRed, realGreen, realBlue, realWhite);  // pETEb



  
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.parseObject(message);

  if (!root.success()) {
    Serial.println("parseObject() failed");
    return false;
  }

  if (root.containsKey("state")) {
    if (strcmp(root["state"], CONFIG_MQTT_PAYLOAD_ON) == 0) {
      stateOn = true;
    }
    else if (strcmp(root["state"], CONFIG_MQTT_PAYLOAD_OFF) == 0) {
      stateOn = false;
    }
  }

  // If "flash" is included, treat RGB and brightness differently
  if (root.containsKey("flash") ||
       (root.containsKey("effect") && strcmp(root["effect"], "flash") == 0)) {

    if (root.containsKey("flash")) {
      flashLength = (int)root["flash"] * 1000;
    }
    else {
      flashLength = CONFIG_DEFAULT_FLASH_LENGTH * 1000;
    }

    if (root.containsKey("brightness")) {
      flashBrightness = root["brightness"];
    }
    else {
      flashBrightness = brightness;
    }

    if (rgb && root.containsKey("color")) {
      flashRed = root["color"]["r"];
      flashGreen = root["color"]["g"];
      flashBlue = root["color"]["b"];
    }
    else {
      flashRed = red;
      flashGreen = green;
      flashBlue = blue;
    }

    if (includeWhite && root.containsKey("white_value")) {
      flashWhite = root["white_value"];
    }
    else {
      flashWhite = white;
    }

    flashRed = map(flashRed, 0, 255, 0, flashBrightness);
    flashGreen = map(flashGreen, 0, 255, 0, flashBrightness);
    flashBlue = map(flashBlue, 0, 255, 0, flashBrightness);
    flashWhite = map(flashWhite, 0, 255, 0, flashBrightness);

    flash = true;
    startFlash = true;
  }
// end Flash
  
  else if (rgb && root.containsKey("effect") &&
      (strcmp(root["effect"], "colorfade_slow") == 0 || strcmp(root["effect"], "colorfade_fast") == 0)) {
    Serial.print("effect: ");
    red = root["color"]["r"];
    green = root["color"]["g"];
    blue = root["color"]["b"];
    flash = false;
    colorfade = true;
    currentColor = 0;
    if (strcmp(root["effect"], "colorfade_slow") == 0) {
      transitionTime = CONFIG_COLORFADE_TIME_SLOW;
      Serial.println("colorfade_slow");
    }
    else {
      transitionTime = CONFIG_COLORFADE_TIME_FAST;
      Serial.println("colorfade_fast");
    }
  } 
  else if (colorfade && !root.containsKey("color") && root.containsKey("brightness")) {
    // Adjust brightness during colorfade
    // (will be applied when fading to the next color)
    brightness = root["brightness"];
  }
// end ColourFade
  
  else { // No effect
    flash = false;
    colorfade = false;

    if (rgb && root.containsKey("color")) {
      red = root["color"]["r"];
      green = root["color"]["g"];
      blue = root["color"]["b"];
      
      if (CONFIG_DEBUG) {
        Serial.print("\tContains: color: r: ");
        Serial.print(realRed);
        Serial.print(", g: ");
        Serial.print(realGreen);
        Serial.print(", b: ");
        Serial.println(realBlue);
      }
    }

    if (includeWhite && root.containsKey("white_value")) {
      white = root["white_value"];
    }

    if (root.containsKey("brightness")) {
      brightness = root["brightness"];
    }

    if (root.containsKey("transition")) {
      transitionTime = root["transition"];
    }
    else {
      transitionTime = 0;
    }
//    setColor(red, green, blue, realWhite);  // pETEb
//    inFade = false;  // pETEb
  }

  return true;
}


////////// SEND STATE ////////////////////////////////////////////////////////////////////////////////////
//void sendState() {
//  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;
//
//  JsonObject& root = jsonBuffer.createObject();
//
//  root["state"] = (stateOn) ? CONFIG_MQTT_PAYLOAD_ON : CONFIG_MQTT_PAYLOAD_OFF;
//  if (rgb) {
//    JsonObject& color = root.createNestedObject("color");
//    color["r"] = red;
//    color["g"] = green;
//    color["b"] = blue;
//  }
//
//  root["brightness"] = brightness;
//
//  if (includeWhite) {
//    root["white_value"] = white;
//  }
//
//  if (rgb && colorfade) {
//    if (transitionTime == CONFIG_COLORFADE_TIME_SLOW) {
//      root["effect"] = "colorfade_slow";
//      Serial.println("Received colorfade_slow");
//    }
//    else {
//      root["effect"] = "colorfade_fast";
//      Serial.println("Received colorfade_fast");
//    }
//  }
//  else {
//    root["effect"] = "null";
//  }
//
//  char buffer[root.measureLength() + 1];
//  root.printTo(buffer, sizeof(buffer));
//
//  client.publish(CONFIG_MQTT_TOPIC_STATE, buffer, true);
//}


///// RECONNECT //////////////////////////////////////////////////////////////////////////////////////////
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
//    if (client.connect(CONFIG_MQTT_CLIENT_ID, CONFIG_MQTT_USER, CONFIG_MQTT_PASS)) {
    if (client.connect(CONFIG_MQTT_CLIENT_ID)) {  // pETEb
      Serial.print("connected to: ");
      Serial.println(CONFIG_MQTT_CLIENT_ID);
      // Lights
      client.subscribe(CONFIG_MQTT_TOPIC_SET);
      // Environment
      client.publish(outTopic, CONFIG_MQTT_CLIENT_ID);
//      client.subscribe(inTopic);
      // Washing Machine
//      client.subscribe(inTopic2);
    } 
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }

//  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
//    WiFi.disconnect();;
//    status = WiFi.begin(CONFIG_WIFI_SSID, CONFIG_WIFI_PASS);
//    delay(1000);
//    yield();
//  }
}


///// SET COLOUR /////////////////////////////////////////////////////////////////////////////////////////
void setColor(int inR, int inG, int inB, int inW) {
  if (CONFIG_INVERT_LED_LOGIC) {
    inR = (255 - inR);
    inG = (255 - inG);
    inB = (255 - inB);
    inW = (255 - inW);
  }

  if (rgb) {
    analogWrite(CONFIG_PIN_RED, inR);
    analogWrite(CONFIG_PIN_GREEN, inG);
    analogWrite(CONFIG_PIN_BLUE, inB);
  }

  if (includeWhite) {
    analogWrite(CONFIG_PIN_WHITE, inW);
  }

  if (CONFIG_DEBUG) {
    Serial.print("Setting LEDs: {");
    if (rgb) {
      Serial.print("r: ");
      Serial.print(inR);
      Serial.print(" , g: ");
      Serial.print(inG);
      Serial.print(" , b: ");
      Serial.print(inB);
    }

    if (includeWhite) {
      if (rgb) {
        Serial.print(", ");
      }
      Serial.print("w: ");
      Serial.print(inW);
    }

    Serial.println("}");
  }
}


///// LOOP ///////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
//  Serial.println("start Loop");

//  if (change) {
//    change = 0;
//    char mqttMsg[1] = {0};
//    uint8_t washOFF = !digitalRead(onPin);
//    sprintf(mqttMsg, "%s%d", mqttMsg, washOFF);  
//    Serial.print("Sending: ");
//    Serial.println(mqttMsg);
//    client.publish(outTopic2, mqttMsg);
//    yield();
//  }

  
  bool rainbow = false;
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
      if ((millis() - flashStartTime) % 1000 <= 500) {
        setColor(flashRed, flashGreen, flashBlue, flashWhite);
      }
      else {
        setColor(0, 0, 0, 0);
        // If you'd prefer the flashing to happen "on top of"
        // the current color, uncomment the next line.
        // setColor(realRed, realGreen, realBlue, realWhite);
      }
    }
    else {
      flash = false;
      setColor(realRed, realGreen, realBlue, realWhite);
    }
  }
// end Flash //////////////////////

  
  else if (rgb && colorfade && !inFade) {
    realRed = map(red, 0, 255, 0, brightness);      // pb
    realGreen = map(green, 0, 255, 0, brightness);  // pb
    realBlue = map(blue, 0, 255, 0, brightness);    // pb
    realWhite = map(white, 0, 255, 0, brightness);  // pb

//  else if (rgb && colorfade && rainbow) {
//    realRed = map(colors[currentColor][0], 0, 255, 0, brightness);
//    realGreen = map(colors[currentColor][1], 0, 255, 0, brightness);
//    realBlue = map(colors[currentColor][2], 0, 255, 0, brightness);
//    realWhite = map(colors[currentColor][3], 0, 255, 0, brightness);
//    currentColor = (currentColor + 1) % numColors;
    startFade = true;
  }


  if (startFade) {
    Serial.print("realRed: ");
    Serial.print(realRed);
    Serial.print(", realGreen: ");
    Serial.print(realGreen);
    Serial.print(", realBlue: ");
    Serial.print(realBlue);
    Serial.print(", realWhite: ");
    Serial.println(realWhite);
        
    // If we don't want to fade, skip it.
    if (transitionTime == 0) {
      Serial.println("transitionTime == 0");
      setColor(realRed, realGreen, realBlue, realWhite);

      redVal = realRed;
      grnVal = realGreen;
      bluVal = realBlue;
      whtVal = realWhite;

      startFade = false;
    }
    else {
      loopCount = 0;
      stepR = calculateStep(redVal, realRed);
      stepG = calculateStep(grnVal, realGreen);
      stepB = calculateStep(bluVal, realBlue);
      stepW = calculateStep(whtVal, realWhite);
      Serial.print("stepR: ");
      Serial.print(stepR);
      Serial.print(", stepG: ");
      Serial.print(stepG);
      Serial.print(", stepB: ");
      Serial.print(stepB);
      Serial.print(", stepW: ");
      Serial.println(stepW);
        
      inFade = true;
    }
  }

  if (inFade) {
    startFade = false;
    unsigned long now = millis();
    if (now - lastLoop > transitionTime) {
//    if ( (now - lastLoop > transitionTime) && inFade ) {  // pb
      if (loopCount <= 1020) {
        lastLoop = now;

        redVal = calculateVal(stepR, redVal, loopCount);
        grnVal = calculateVal(stepG, grnVal, loopCount);
        bluVal = calculateVal(stepB, bluVal, loopCount);
        whtVal = calculateVal(stepW, whtVal, loopCount);
        if ((int)redVal==realRed && (int)grnVal==realGreen && (int)bluVal==realBlue){
          inFade = false;
          colorfade = false;
        }
        Serial.print("redVal: ");
        Serial.print(redVal);
        Serial.print(", grnVal: ");
        Serial.print(grnVal);
        Serial.print(", bluVal: ");
        Serial.print(bluVal);
        Serial.print(", whtVal: ");
        Serial.println(whtVal);
        
        setColor(redVal, grnVal, bluVal, whtVal); // Write current values to LED pins

        Serial.print("Loop count: ");
        Serial.print(loopCount);
        Serial.print("/");
        Serial.print(transitionTime);
        loopCount++;
      }
      else {
        inFade = false;
//        colorfade = false; // pETEb
        loopCount = 0;
      }
    }
  }
//  Serial.println("end Loop b4 yield");
  yield();
//  Serial.println("end Loop aft yield");
}


///// GET TEMP ///////////////////////////////////////////////////////////////////////////////////////////
void gettemperature() {
  // Wait at least 2 seconds seconds between measurements.
  // if the difference between the current time and last time you read
  // the sensor is bigger than the interval you set, read the sensor
  // Works better than delay for things happening elsewhere also
  unsigned long currentMillis = millis();
 
  if(currentMillis - previousMillis >= interval) {
    // save the last time you read the sensor 
    previousMillis = currentMillis;   

    // Reading temperature for humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
      humidity = dht.readHumidity();          // Read humidity (percent)
      temp_c = dht.readTemperature();         // Read temperature as Celcius
    
    // Check if any reads failed and exit early (to try again).
    if (isnan(humidity) || isnan(temp_c)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }
  }
  yield();
}


///// WASHING ////////////////////////////////////////////////////////////////////////////////////////////
//void washMachineSTATE() {
//  change = 1;
////  char mqttMsg[1] = {0};
////  uint8_t washOFF = digitalRead(onPin);
////  sprintf(mqttMsg, "%s%d", mqttMsg, washOFF);  
////  Serial.print("Sending: ");
////  Serial.println(mqttMsg);
////  client.publish(outTopic2, mqttMsg);
//}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// From https://www.arduino.cc/en/Tutorial/ColorCrossfader
/* BELOW THIS LINE IS THE MATH -- YOU SHOULDN'T NEED TO CHANGE THIS FOR THE BASICS
*
* The program works like this:
* Imagine a crossfade that moves the red LED from 0-10,
*   the green from 0-5, and the blue from 10 to 7, in
*   ten steps.
*   We'd want to count the 10 steps and increase or
*   decrease color values in evenly stepped increments.
*   Imagine a + indicates raising a value by 1, and a -
*   equals lowering it. Our 10 step fade would look like:
*
*   1 2 3 4 5 6 7 8 9 10
* R + + + + + + + + + +
* G   +   +   +   +   +
* B     -     -     -
*
* The red rises from 0 to 10 in ten steps, the green from
* 0-5 in 5 steps, and the blue falls from 10 to 7 in three steps.
*
* In the real program, the color percentages are converted to
* 0-255 values, and there are 1020 steps (255*4).
*
* To figure out how big a step there should be between one up- or
* down-tick of one of the LED values, we call calculateStep(),
* which calculates the absolute gap between the start and end values,
* and then divides that gap by 1020 to determine the size of the step
* between adjustments in the value.
*/
float calculateStep(int prevValue, int endValue) {
    float step = (float)endValue - prevValue; // What's the overall gap?
    Serial.print("endVal-prevVal: ");
    Serial.println(endValue - prevValue);
    Serial.print("step: ");
    Serial.println(step);
    if (step) {                      // If its non-zero,
//        step = 1020/step;            //   divide by 1020
        step = step/1020;            //   divide by 1020 pETEb
    }

    return step;
}

/* The next function is calculateVal. When the loop value, i,
*  reaches the step size appropriate for one of the
*  colors, it increases or decreases the value of that color by 1.
*  (R, G, and B are each calculated separately.)
*/
float calculateVal(float step, float val, int i) {
//    if ((step) && i % (int)step == 0) { // If step is non-zero and its time to change a value,
    if ((step)) { // If step is non-zero, // pETEb
//    if ((step) && i % (int)step == 0) { // If step is non-zero and its time to change a value,
///      val += step;
//      val = step*i;
          val = val + step; // pb
//        if (step > 0) {              //   increment the value if step is positive...
//            val += 1;
//        }
//        else if (step < 0) {         //   ...or decrement it if step is negative
//            val -= 1; 
//        }
    }

    // Defensive driving: make sure val stays in the range 0-255
    if (val > 255) {
        val = 255;
    }
    else if (val < 0) {
        val = 0;
    }

    return val;
}
