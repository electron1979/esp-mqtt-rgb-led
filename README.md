# ESP8266 MQTT Lights Using JSON for Home Assistant & Node-RED

This project adds an easy way to create DIY lighting for [Home Assistant](https://home-assistant.io/), an amazing, extensible, open-source home automation system.

By sending a JSON payload (in an MQTT message), Home Assistant or Node-RED can include whichever fields are necessary, reducing the round trips from 3 to 1. For example, this is a sample payload including most of the fields:
```json
{
  "state": "ON",
  "flash": 5
}
```

### The Light
I'm using ESP8266 microcontrollers for my lights because they are so cheap and small. The downside of the size and price is that programming them can be a bit of a hassle. There are many sites that go into detail, so I won't do it here. You'll need an ESP set up to work with the Arduino IDE. See the readme [here](https://github.com/esp8266/Arduino) for instructions. Another good device to work with is the [Wemos D1 Mini](https://wiki.wemos.cc/products:d1:d1_mini), which has a built-in micro-USB port and is much easier to program.

1. Using the Library Manager in the Arduino IDE, install [ArduinoJSON](https://github.com/bblanchon/ArduinoJson/) and [PubSubClient](http://pubsubclient.knolleary.net/). You can find the Library Manager in the "Sketch" menu under "Include Library" -> "Manage Libraries..."
  * **NOTE:**: At the moment, this project only supports ArduinoJSON version 5. The default is now version 6, so you'll have to specify the version to install in the dropdown in the Library Manager window.
2. Open the `mqtt_esp8266_light` project in the Arduino IDE.
3. Update the `config-sample.h` file with your settings for LED type, pin numbers, WiFi settings, and MQTT settings.
  1. Review the comments to help with the options. For the `CONFIG_STRIP` option, choose one of `BRIGHTNESS`, `RGB`, or `RGBW`.
  2. Ensure that the `CONFIG_MQTT_CLIENT_ID` setting is a unique value for your network.
  3. Set `CONFIG_MQTT_TOPIC_STATE` and `CONFIG_MQTT_TOPIC_SET` to match the values you put in your `configuration.yaml`.
4. Save the configuration file as `config.h`.
5. Open the `.ino` file in the Arduino IDE and upload to an ESP with the correct connections.


#### Wiring
Note that the MOSFETs have pull-**up** resistors in this setup. This means that the lights may flash on when the module resets, but it was necessary to keep the ESP's pins in the right start state.
