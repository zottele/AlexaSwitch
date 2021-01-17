#include <Arduino.h>
#if defined(ESP8266)
    #include <ESP8266WiFi.h>
#elif defined(ESP32)
    #include <WiFi.h>
#endif
#include <ESPAsyncWebServer.h>

#define MQTT
#ifdef MQTT
    #include <pubsubclient.h>
#endif
#include "fauxmoESP.h"
#include "credentials.h"
#include <RCSwitch.h>

fauxmoESP fauxmo;
AsyncWebServer server(80);
RCSwitch mySwitch = RCSwitch();

bool status_todo = false;
int device [2] =  {-1,-1} ;

// -----------------------------------------------------------------------------

#define SERIAL_BAUDRATE                 115200
#define LED                             2

#ifdef MQTT
  // MQTT-Settings
  WiFiClient espClient;
  PubSubClient mqttClient; 
    
  const char *mqtt_server = "192.168.1.115";
  const char *mqtt_username = ""; // "ioBroker"; // MQTT-User
  const char *mqtt_key = ""; // "iobroker"; // MQTT-Pass
  const int mqtt_port = 1883;
    
  const char* PTopic = "haus/keller/esp/schalter"; /* Topic to Publish  */
  const char* KTopic = "haus/keller/esp/keepAlive"; /* Topic to Publish  */
  const char* STopic = "haus/keller/esp/outTopic";   /* Topic to Subscripe */
#endif
// -----------------------------------------------------------------------------
// Wifi
// -----------------------------------------------------------------------------

void wifiSetup() {

    // Set WIFI module to STA mode
    WiFi.mode(WIFI_STA);

    // Connect
    Serial.printf("[WIFI] Connecting to %s ", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    // Wait
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(100);
    }
    Serial.println();

    // Connected!
    Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());


#ifdef MQTT
    mqttClient.setClient(espClient);
    mqttClient.setServer(mqtt_server, mqtt_port);
    // mqttClient.setCallback(callback);

    while (!mqttClient.connected()) {
        Serial.print("Connecting to MQTT...");
        if (mqttClient.connect("ESP32Client", mqtt_username, mqtt_key,"letzterWille: ",0,true,"Nun gehe ich von euch",true )) {
            Serial.println("connected");
        } else {
            Serial.print("failed with state ");
            Serial.println(mqttClient.state());
            delay(2000);
        }
    }
    if (mqttClient.publish(PTopic, "Connected to MQTT from keller/esp/schalte") == true){
        Serial.print("Publishing to Topic: ");
        Serial.println(PTopic);
    } else {
        Serial.println("Got publishing Error");
    }
    if (mqttClient.publish(KTopic, "Connected to MQTT from keller/esp/keepAlive") == true){
        Serial.print("Publishing to Topic: ");
        Serial.println(KTopic);
    } else {
        Serial.println("Got publishing Error");
    }
    /*
    if (mqttClient.subscribe(STopic) == true){
        Serial.print("Subscribed to Topic: ");
        Serial.println(STopic);
    } else {
        Serial.println("Got Subscribed Error");
    }  
    */
#endif
}

void serverSetup() {

    // Custom entry point (not required by the library, here just as an example)
    server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/plain", "Hello, world");
    });

    // These two callbacks are required for gen1 and gen3 compatibility
    server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        if (fauxmo.process(request->client(), request->method() == HTTP_GET, request->url(), String((char *)data))) return;
        // Handle any other body request here...
    });
    server.onNotFound([](AsyncWebServerRequest *request) {
        String body = (request->hasParam("body", true)) ? request->getParam("body", true)->value() : String();
        if (fauxmo.process(request->client(), request->method() == HTTP_GET, request->url(), body)) return;
        // Handle not found request here...
    });

    // Start the server
    server.begin();

}


void rfSetup() {
     
  mySwitch.enableTransmit(0);  // Receiver on interrupt 0 => that is pin #2
  // Optional set pulse length.
  // mySwitch.setPulseLength(320); 
}

void setup() {

    // Init serial port and clean garbage
    Serial.begin(SERIAL_BAUDRATE);
    Serial.println();
    Serial.println();

    // LED
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH); // Our LED has inverse logic (high for OFF, low for ON)

    // Wifi
    wifiSetup();

    // Web server
    serverSetup();

    // RF Setup
    rfSetup();

    fauxmo.createServer(false);
    fauxmo.setPort(80); // This is required for gen3 devices
    fauxmo.enable(true);

    // Add virtual devices
    fauxmo.addDevice("keller links");
    fauxmo.addDevice("keller rechts");
    fauxmo.addDevice("keller decke");

    fauxmo.onSetState([](unsigned char device_id, const char * device_name, bool state, unsigned char value) {
        
        Serial.printf("[MAIN] Device #%d (%s) state: %s value: %d\n", device_id, device_name, state ? "ON" : "OFF", value);
        status_todo = true;
        switch (device_id){

            case 0:
                if (state == true){
                    device [0] = 0;
                    device [1] = 255;
                } else {
                    device [0] = 0;
                    device [1] = 0;
                }
                break;
            case 1:
                if (state == true){
                    device [0] = 1;
                    device [1] = 255;
                } else {
                    device [0] = 1;
                    device [1] = 0;
                }
                break;
            case 2:
                if (state == true){
                    device [0] = 2;
                    device [1] = 255;
                } else {
                    device [0] = 2;
                    device [1] = 0;
                }
                break;
        }
    } );// onSetState

} // setup

void loop() {

    fauxmo.handle();

    static unsigned long last = millis();
    if (millis() - last > 5000) {
        last = millis();
        Serial.printf("[MAIN] Free heap: %d bytes\n", ESP.getFreeHeap());
        mqttClient.publish(KTopic, "keepAlive");
    }

    if (status_todo == true){
        switch (device[0]){
            case 0:
                if (device[1] !=0 ) {
                    mySwitch.switchOn("11111","10000");
                    #ifdef MQTT 
                        mqttClient.publish(PTopic, "Device 0 switched to On");
                    #endif
                } else {                
                    mySwitch.switchOff("11111","10000");
                    #ifdef MQTT 
                        mqttClient.publish(PTopic, "Device 0 switched to Off");     
                    #endif
                }
                break;
            case 1:
                if (device[1] !=0 ) {
                    mySwitch.switchOn("11111","01000");
                    #ifdef MQTT 
                        mqttClient.publish(PTopic, "Device 1 switched to On");
                    #endif
                } else {
                    mySwitch.switchOff("11111","01000");
                    #ifdef MQTT 
                        mqttClient.publish(PTopic, "Device 1 switched to Off");
                    #endif
                }
                break;
            case 2:
                if (device[1] !=0 ) {
                    mySwitch.switchOn("10111","00010");
                    #ifdef MQTT 
                        mqttClient.publish(PTopic, "Device 2 switched to On");
                    #endif
                }else{
                    mySwitch.switchOff("10111","00010");
                    #ifdef MQTT 
                        mqttClient.publish(PTopic, "Device 2 switched to Off");
                    #endif
                }
                break;
            default: 
                break;
        }    
        status_todo=false;
        device[0]=-1;device[1]=-1;
        delay(1000);
    }
}