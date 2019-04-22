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
//#include "config.h"

//// https://github.com/bblanchon/ArduinoJson
//#include <ArduinoJson.h>
//
//#include <ESP8266WiFi.h>
//
//// http://pubsubclient.knolleary.net/
//#include <PubSubClient.h>

//#include <DHT.h>
#include "DHT.h"
// Environmental
#define DHTTYPE DHT22
// BEDHEAD
//#define DHTPIN  D1// (GPIO05)    // *** Not D8 (GPIO15) ***
// BEDROOM
#define DHTPIN 5 //2
// Initialize DHT sensor 
// NOTE: For working with a faster than ATmega328p 16 MHz Arduino chip, like an ESP8266,
// you need to increase the threshold for cycle counts considered a 1 or 0.
// You can do this by passing a 3rd parameter for this threshold.  It's a bit
// of fiddling to find the right value, but in general the faster the CPU the
// higher the value.  The default for a 16mhz AVR is a value of 6.  For an
// Arduino Due that runs at 84mhz a value of 30 works.
// This is for the ESP8266 processor on ESP-01 
//DHT dht(DHTPIN, DHTTYPE, 11); // 11 works fine for DHT11 on ESP8266
//DHT dht(DHTPIN, DHTTYPE, 11); // 11 works fine for DHT11 on ESP8266
DHT dht(DHTPIN, DHTTYPE); // 

#define CONFIG_PIN_RED   12 // For RGB(W)
#define CONFIG_PIN_GREEN 13 // For RGB(W) BuiltinBlueLED
#define CONFIG_PIN_BLUE  14 // BUILTIN_LED //3  // For RGB(W)

float humidity, temp_c;  // Values read from sensor


void setup() {
  pinMode(CONFIG_PIN_RED, OUTPUT);
  pinMode(CONFIG_PIN_GREEN, OUTPUT);
  pinMode(CONFIG_PIN_BLUE, OUTPUT);

  Serial.begin(115200);

  dht.begin();           // initialize temperature sensor
}





void loop() {
  
  humidity = -1;
  temp_c = -127;
//  gettemperature();
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float humidity = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float temp_c = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temp_c)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("\ttemperature: ");
  Serial.println(temp_c);
  
  Serial.print("\thumidity: ");
  Serial.println(humidity);



  digitalWrite(CONFIG_PIN_RED, HIGH);
  delay(500);
  digitalWrite(CONFIG_PIN_RED, LOW);
  delay(500);
  
  digitalWrite(CONFIG_PIN_GREEN, HIGH);
  delay(500);
  digitalWrite(CONFIG_PIN_GREEN, LOW);
  delay(500);

  digitalWrite(CONFIG_PIN_BLUE, HIGH);
  delay(1000);
  digitalWrite(CONFIG_PIN_BLUE, LOW);
  delay(500);

  digitalWrite(CONFIG_PIN_RED, HIGH);
  digitalWrite(CONFIG_PIN_GREEN, HIGH);
  delay(500);
  digitalWrite(CONFIG_PIN_RED, LOW);
  digitalWrite(CONFIG_PIN_GREEN, LOW);
  delay(500);

  digitalWrite(CONFIG_PIN_GREEN, HIGH);
  digitalWrite(CONFIG_PIN_BLUE, HIGH);
  delay(500);
  digitalWrite(CONFIG_PIN_GREEN, LOW);
  digitalWrite(CONFIG_PIN_BLUE, LOW);
  delay(500);

  digitalWrite(CONFIG_PIN_BLUE, HIGH);
  digitalWrite(CONFIG_PIN_RED, HIGH);
  delay(500);
  digitalWrite(CONFIG_PIN_BLUE, LOW);
  digitalWrite(CONFIG_PIN_RED, LOW);
  delay(500);


  digitalWrite(CONFIG_PIN_RED, HIGH);
  digitalWrite(CONFIG_PIN_GREEN, HIGH);
  digitalWrite(CONFIG_PIN_BLUE, HIGH);
  delay(500);
  digitalWrite(CONFIG_PIN_RED, LOW);
  digitalWrite(CONFIG_PIN_GREEN, LOW);
  digitalWrite(CONFIG_PIN_BLUE, LOW);
  delay(500);
}

//void gettemperature() {
//  // Wait at least 2 seconds seconds between measurements.
//  // if the difference between the current time and last time you read
//  // the sensor is bigger than the interval you set, read the sensor
//  // Works better than delay for things happening elsewhere also
// 
//  // Reading temperature for humidity takes about 250 milliseconds!
//  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
//    humidity = dht.readHumidity();          // Read humidity (percent)
//    temp_c = dht.readTemperature();         // Read temperature as Celcius
//  
//  // Check if any reads failed and exit early (to try again).
//  if (isnan(humidity) || isnan(temp_c)) {
//    Serial.println("Failed to read from DHT sensor!");
//    return;
//  }
//}
