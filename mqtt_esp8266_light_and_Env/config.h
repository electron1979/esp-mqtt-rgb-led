/*
 * This is a sample configuration file for the "mqtt_esp8266" light.
 *
 * Change the settings below and save the file as "config.h"
 * You can then upload the code using the Arduino IDE.
 */

// CONFIG ///////////////////////////////////////////////////////
#define CONFIG_DEBUG true // Enables Serial and print statements

#define CONFIG_STRIP RGB  // Choose one of the options from above.

#define CONFIG_DEFAULT_TRANSITION_TIME 0

#define FLASH_ONTOP true  // Flash on top of current colour

// WiFi
#define CONFIG_WIFI_SSID "WIFI_SSID"
#define CONFIG_WIFI_PASS "WIFI_PASSWORD"

// MQTT
#define CONFIG_MQTT_HOST "xxx.xxx.xxx.xxx"
#define CONFIG_MQTT_PORT 1883 // Usually 1883
#define CONFIG_MQTT_USER "MQTT_USER"
#define CONFIG_MQTT_PASS "MQTT_PASSWORD"

// Default number of flashes if no value was given
#define CONFIG_DEFAULT_FLASH_LENGTH 3
// Number of seconds for one transition in colorfade mode
#define CONFIG_COLORFADE_TIME_SLOW 10
#define CONFIG_COLORFADE_TIME_FAST 3

// Reverse the LED logic
// false: 0 (off) - 255 (bright)
// true: 255 (off) - 0 (bright)
#define CONFIG_INVERT_LED_LOGIC false

// Set the mode for the built-in LED on some boards.
// -1 = Do nothing. Leave the pin in its default state.
//  0 = Explicitly set the LED_BUILTIN to LOW.
//  1 = Explicitly set the LED_BUILTIN to HIGH. (Off for Wemos D1 Mini)
#define CONFIG_LED_BUILTIN_MODE -1
      
//// Environmental
#define DHTTYPE DHT22
#define DHTPIN  5


// In case of BRIGHTNESS: only WHITE is used
// In case of RGB(W): red, green, blue(, white) is used
// All values need to be present, if they are not needed, set to -1,
// it will be ignored.

#define LED_BUILTIN 1
#define CONFIG_PIN_RED   0 // For RGB(W)
#define CONFIG_PIN_GREEN 2 // For RGB(W)
#define CONFIG_PIN_BLUE  3 // For RGB(W)
#define CONFIG_PIN_WHITE -1 // For BRIGHTNESS and RGBW
#define CONFIG_MQTT_CLIENT_ID "external_RGB_Something" // Must be unique on the MQTT network
#define CONFIG_MQTT_TOPIC_SET "external/RBG/Some/thing"
//const char* outTopic = "external/envSensor/Something";

// END_CONFIG ///////////////////////////////////////////////////








// Node-RED rgbColourPicker
const char    start[2]    = "(";
const char    delim[2]    = ",";

// Leave this here. These are the choices for CONFIG_STRIP below.
enum strip {
  BRIGHTNESS, // only one color/only white
  RGB,        // RGB LEDs
  RGBW        // RGB LEDs with an extra white LED per LED
};

#define CONFIG_MQTT_PAYLOAD_ON "ON"
#define CONFIG_MQTT_PAYLOAD_OFF "OFF"
