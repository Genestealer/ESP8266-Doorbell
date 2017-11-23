/***************************************************
  Wi-Fi Doorbell
  Richard Huish 207
  ESP8266 NodeMCU based with push notifications to pushingbox.com,
  which triggers Pushbullet notification.
    ----------
  Key Libraries:
  ESP8266WiFi.h     https://github.com/esp8266/Arduino
  ESP8266mDNS.h     https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266mDNS
  WiFiUdp.h         https://github.com/esp8266/Arduino
  ArduinoOTA.h      https://github.com/esp8266/Arduino
  ArduinoJson.h     https://bblanchon.github.io/ArduinoJson/
  ----------
  OTA updates
  Notification to Pushbullet via pushingbox
  ----------
  The circuit:
  NodeMCU Amica (ESP8266)
  Inputs:
    Pulldown to ground from doorbell
    Doorbell button - GPIO pin 5 (NodeMCU Pin D1) pulldown
  Outputs:
    GET HTTP to pushingbox.
    pushingbox then send the notication via Pushbullet to phones
    ----------
  Notes:
    NodeMCU lED lights to show DoorBell state.
    ESP lED lights to show WIFI conenction.

  ----------
  Sources:
    Elements of code from PushingBox_Arduino_Ethernet_Official
      https://github.com/Clement87/PushingBox-for-Arduino
****************************************************/

#include <private.h>               // Passwords etc not for github
#include <ESP8266WiFi.h>           // ESP8266 core for Arduino https://github.com/esp8266/Arduino
#include <PubSubClient.h>          // Arduino Client for MQTT https://github.com/knolleary/pubsubclient
#include <ESP8266mDNS.h>           // Needed for Over-the-Air ESP8266 programming https://github.com/esp8266/Arduino
#include <WiFiUdp.h>               // Needed for Over-the-Air ESP8266 programming https://github.com/esp8266/Arduino
#include <ArduinoOTA.h>            // Needed for Over-the-Air ESP8266 programming https://github.com/esp8266/Arduino
#include <ArduinoJson.h>           // For sending MQTT JSON messages https://bblanchon.github.io/ArduinoJson/

// WiFi parameters
const char* wifi_ssid = secret_wifi_ssid; // Wifi access point SSID
const char* wifi_password = secret_wifi_password; // Wifi access point password

// MQTT Settings
const char* mqtt_server = secret_mqtt_server; // E.G. 192.168.1.xx
const char* clientName = secret_clientName; // Client to report to MQTT
const char* mqtt_username = secret_mqtt_username; // MQTT Username
const char* mqtt_password = secret_mqtt_password; // MQTT Password
bool willRetain = true; // MQTT Last Will and Testament
const char* willMessage = "offline"; // MQTT Last Will and Testament Message
const int json_buffer_size = 256;


// Publish
const char* publishLastWillTopic = secret_publishLastWillTopic; //
const char* publishStatusJsonTopic = secret_publishStatusJsonTopic;

// MQTT instance
WiFiClient espClient;
PubSubClient mqttClient(espClient);
char message_buff[100];
long lastReconnectAttempt = 0; // Reconnecting MQTT - non-blocking https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_reconnect_nonblocking/mqtt_reconnect_nonblocking.ino

// MQTT publish frequency
unsigned long previousMillis = 0;
const long publishInterval = 6000; // Publish requency in milliseconds 60000 = 1 min

// LED output parameters
const int DIGITAL_PIN_LED_ESP = 2; // Define LED on ESP8266 sub-modual
const int DIGITAL_PIN_LED_NODEMCU = 16; // Define LED on NodeMCU board - Lights on pin LOW

// Doorbell parameters
const int doorbellButton = 5; //push button attached to this pin
int buttonState = LOW; //this variable tracks the state of the button, low if not pressed, high if pressed
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 500;    // the debounce time; increase if the output flickers

// Pushingbox DevID code
char DEVID1[] = secret_pushingbox_DevID;


char serverName[] = "api.pushingbox.com";
boolean pinDevid1State = false;                // Save the last state of the Pin for DEVID1
boolean lastConnected = false;                 // State of the connection last time through the main loop


// Setp the connection to WIFI and the MQTT Broker. Normally called only once from setup
void setup_wifi() {
  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);

  // Connect to the WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.print(wifi_ssid);
  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.print("");
  Serial.println("WiFi connected");

  Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());
  Serial.printf("Hostname: %s\n", WiFi.hostname().c_str());

}

// Setup Over-the-Air programming, called from the setup.
// https://www.penninkhof.com/2015/12/1610-over-the-air-esp8266-programming-using-platformio/
void setup_OTA() {
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);
  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");
  // No authentication by default
  // ArduinoOTA.setPassword("admin");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}



//Function for sending the request to PushingBox
// https://www.pushingbox.com/
void sendToPushingBox(char devid[]) {
  // Call on the background functions to allow them to do their thing
  yield();
  // espClient.stop();
  // Serial.println("Connecting...");
  // Call on the background functions to allow them to do their thing
  yield();
  if (espClient.connect(serverName, 80)) {
    Serial.println("Connected to pushingbox");
    Serial.println("Sending GET request");
    espClient.print("GET /pushingbox?devid=");
    espClient.print(devid);
    espClient.println(" HTTP/1.1");
    espClient.print("Host: ");
    espClient.println(serverName);
    espClient.println("User-Agent: Arduino");
    espClient.println();
    // Read all the lines of the reply from server and print them to Serial
    while (espClient.available()) {
      // Call on the background functions to allow them to do their thing
      yield();
      String line = espClient.readStringUntil('\r');
      Serial.print(line);
    }
    Serial.println("Sent GET request to pushingbox");
    digitalWrite(DIGITAL_PIN_LED_NODEMCU, HIGH); //turn LED off, lights on HIGH
  }
  else {
    Serial.println("Connection Failed");
  }
}








void publishNodeState() {
  // Update status to online, retained = true - last will Message will drop in if we go offline
  mqttClient.publish(publishLastWillTopic, "online", true);
  // Gather data
  char bufIP[16]; // Wifi IP address
  sprintf(bufIP, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
  char bufMAC[6]; // Wifi MAC address
  sprintf(bufMAC, "%02x:%02x:%02x:%02x:%02x:%02x", WiFi.macAddress()[0], WiFi.macAddress()[1], WiFi.macAddress()[2], WiFi.macAddress()[3], WiFi.macAddress()[4], WiFi.macAddress()[5] );
  // Create and publish the JSON object.
  StaticJsonBuffer<json_buffer_size> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  root["ClientName"] = String(clientName);
  root["IP"] = String(bufIP);
  root["MAC"] = String(bufMAC);
  root["RSSI"] = String(WiFi.RSSI());
  root["HostName"] = String(WiFi.hostname());
  root["ConnectedSSID"] = String(WiFi.SSID());
  root.prettyPrintTo(Serial);
  Serial.println(""); // Add new line as prettyPrintTo leaves the line open.
  char data[json_buffer_size];
  root.printTo(data, root.measureLength() + 1);
  if (!mqttClient.publish(publishStatusJsonTopic, data, true)) // retained = true
    Serial.print(F("Failed to publish JSON Status to [")), Serial.print(publishStatusJsonTopic), Serial.print("] ");
  else
    Serial.print(F("JSON Status Published [")), Serial.print(publishStatusJsonTopic), Serial.println("] ");
}

/*
  Non-Blocking mqtt reconnect.
  Called from checkMqttConnection.
  Based on example from 5ace47b Sep 7, 2015 https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_reconnect_nonblocking/mqtt_reconnect_nonblocking.ino
*/
boolean mqttReconnect() {
  // Call on the background functions to allow them to do their thing
  yield();
  // Attempt to connect
  if (mqttClient.connect(clientName, mqtt_username, mqtt_password, publishLastWillTopic, 0, willRetain, willMessage)) {
    Serial.print("Attempting MQTT connection...");
    // Publish node state data
    publishNodeState();
    Serial.println("Connected to MQTT server");
  }
  else
  {
    Serial.print("Failed MQTT connection, rc=");
    Serial.print(mqttClient.state());
    Serial.println(" try again in 1.5 seconds");
  }
  return mqttClient.connected(); // Return connection state
}


/*
  Checks if connection to the MQTT server is ok. Client connected
  using a non-blocking reconnect function. If the client loses
  its connection, it attempts to reconnect every 5 seconds
  without blocking the main loop.
  Called from main loop.
*/
void checkMqttConnection() {
  if (!mqttClient.connected()) {
    // We are not connected. Turn off the wifi LED
    digitalWrite(DIGITAL_PIN_LED_ESP, HIGH); // Lights on LOW
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (mqttReconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // We are connected.
    digitalWrite(DIGITAL_PIN_LED_ESP, LOW); // Lights on LOW
    //Call on the background functions to allow them to do their thing.
    yield();
    // Client connected: MQTT client loop processing
    mqttClient.loop();
  }
}

// MQTT Publish with normal or immediate option.
void mqttPublishData(bool ignorePublishInterval) {
  // Only run when publishInterval in milliseonds expires or ignorePublishInterval == true
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= publishInterval || ignorePublishInterval == true) {
    previousMillis = currentMillis; // Save the last time this ran
    // Check conenction to MQTT server
    if (mqttClient.connected()) {
      // Publish node state data
      publishNodeState();
    }}}



// Check button state
void checkButtonState() {
  // Sample the state of the button - is it pressed or not?
  buttonState = digitalRead(doorbellButton);
  // Filter out any noise by setting a time buffer
  if ( (millis() - lastDebounceTime) > debounceDelay) {
    // If the button has been pressed, send notification
    if (buttonState == LOW) {
      Serial.println("Button Pressed");
      digitalWrite(DIGITAL_PIN_LED_NODEMCU, LOW); //turn LED on, lights on HIGH
      lastDebounceTime = millis(); //set the current time
      sendToPushingBox(DEVID1);
    }
  }
}



void setup() {
  // Initialize pins
  pinMode(DIGITAL_PIN_LED_NODEMCU, OUTPUT);
  pinMode(DIGITAL_PIN_LED_ESP, OUTPUT);
  pinMode(doorbellButton, INPUT_PULLUP);
  // Initialize pin start values
  digitalWrite(DIGITAL_PIN_LED_NODEMCU, HIGH); // Lights on HIGH
  digitalWrite(DIGITAL_PIN_LED_ESP, HIGH); // Lights on LOW
  // Set serial speed
  Serial.begin(115200);
  Serial.println("Setup Starting");
  // Call on the background functions to allow them to do their thing
  yield();
  // Setup wifi
  setup_wifi();
  // Call on the background functions to allow them to do their thing
  yield();
  // Setup OTA updates.
  setup_OTA();
  // Call on the background functions to allow them to do their thing
  yield();
  // Set MQTT settings
  mqttClient.setServer(mqtt_server, 1883);
  // Call on the background functions to allow them to do their thing
  yield();
  // Call on the background functions to allow them to do their thing
  Serial.println("Setup Complete");
}




// Main working loop
void loop() {
  // Call on the background functions to allow them to do their thing
  yield();
  // First check if we are connected to the MQTT broker
  checkMqttConnection();
  // Call on the background functions to allow them to do their thing.
  yield();
  // Check the status and do actions
  checkButtonState();
  // Publish MQTT
  mqttPublishData(false); // Normal publish cycle
  //Call on the background functions to allow them to do their thing.
  yield();
  // Check for Over The Air updates
  ArduinoOTA.handle();
  // Deal with millis rollover, hack by resetting the esp every 48 days
  if (millis() > 4147200000)
    ESP.restart();

  // // DEBUG part
  // // Write the respons from PushingBox Server.
  // // You should see a "200 OK"
  // if (espClient.available()) {
  //   char c = espClient.read();
  //   Serial.print(c);
  // }
  //
  // // If there's no net connection, but there was one last time
  // // through the loop, then stop the client:
  // if (!espClient.connected() && lastConnected) {
  //   Serial.println();
  //   Serial.println("disconnecting.");
  //   espClient.stop();
  //   digitalWrite(DIGITAL_PIN_LED_NODEMCU, HIGH); //turn LED off
  // }
  //
  // lastConnected = espClient.connected();

}
