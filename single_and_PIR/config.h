/*
 * This is a sample configuration file for the "mqtt_esp8266" light.
 *
 * Change the settings below and save the file as "config.h"
 * You can then upload the code using the Arduino IDE.
 */


// Enables Serial and print statements
#define CONFIG_DEBUG true

// Pins
#define CONFIG_PIN_WHITE   12 // For RGB(W)
#define PIR1  13
#define PIR2  14

// WiFi
#define CONFIG_WIFI_SSID "SSID"
#define CONFIG_WIFI_PASS "WIFI_PASS"

// MQTT
#define CONFIG_MQTT_HOST "192.168.1.2"
#define CONFIG_MQTT_PORT 1883 // Usually 1883

// Bedroom Main
#define CONFIG_MQTT_CLIENT_ID "internal_bedroom_main" // Must be unique on the MQTT network
#define CONFIG_MQTT_TOPIC_SET "internal/main/bedroom"
const char* outTopic  = "internal/pir/bedroom";


#define CONFIG_MQTT_PAYLOAD_ON "ON"
#define CONFIG_MQTT_PAYLOAD_OFF "OFF"

// Miscellaneous
// Default number of flashes if no value was given
#define CONFIG_DEFAULT_FLASH_LENGTH 3

// Reverse the LED logic
// false: 0 (off) - 1 (on)
// true: 1 (off) - 0 (on)
#define CONFIG_INVERT_LED_LOGIC false

// Set the mode for the built-in LED on some boards.
// -1 = Do nothing. Leave the pin in its default state.
//  0 = Explicitly set the BUILTIN_LED to LOW.
//  1 = Explicitly set the BUILTIN_LED to HIGH. (Off for Wemos D1 Mini)
//#define CONFIG_BUILTIN_LED_MODE -1
#define BUILTIN_LED 2
