
// ESP32 I2C Scanner
// Based on code of Nick Gammon  http://www.gammon.com.au/forum/?id=10896
// ESP32 DevKit - Arduino IDE 1.8.5
// Device tested PCF8574 - Use pullup resistors 3K3 ohms !
// PCF8574 Default Freq 100 KHz 

#include "credentials.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

Adafruit_BME280 bme;

//OTA includes
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>

#include <SPI.h>

#include <TimeLib.h>


#define mphal_i2c_wait_a() os_delay_us(20)
#define mphal_i2c_wait_b() os_delay_us(10)


const int  buttonPin = 19;    // the pin that the pushbutton is attached to
const int Analog_channel_pin = 35;    // the pin that the voltage sensor is attached to
int ADC_VALUE = 0;
int voltage_value = 0; 
float vout = 0.0;
float vin = 0.0;
float R1 = 30000.0; //  
float R2 = 7500.0; // 

// Variables will change:
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button


int trigPin = 5;            // HC-SR04 trigger pin
int echoPin = 4;            // HC-SR04 echo pin
float duration, distance;

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWD;

const char* mqtt_server = "10.1.1.22";

const int valve4Pin = 32;
const int valve1Pin = 33;
const int valve2Pin = 25;
const int valve3Pin = 26;

WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;
char msg[50];
int value = 0;

float temperature = 0;
float pressure = 0;
float humidity = 0;
float moisture = 0;
float tanklevel = 0;

char data[120];
StaticJsonBuffer<512> jsonBuffer;


void setup() {
  Serial.begin(115200);
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  //status = bme.begin();  
  pinMode(buttonPin, INPUT_PULLDOWN);
  pinMode(valve4Pin, OUTPUT);
  pinMode(valve1Pin, OUTPUT);
  pinMode(valve2Pin, OUTPUT);
  pinMode(valve3Pin, OUTPUT);  
  pinMode(trigPin, OUTPUT); // define trigger pin as output
  Wire.begin (21, 22);   // sda= GPIO_21 /scl= GPIO_22
    if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  
  digitalWrite(valve1Pin, LOW);
  digitalWrite(valve4Pin, LOW);
  digitalWrite(valve2Pin, LOW);
  digitalWrite(valve3Pin, LOW);  

  setup_wifi();
  

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  delay(1000);

}


void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
    ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

}

void callback(char* topic, byte* payload, unsigned int length) {
    
    if(strcmp(topic, "esp32/driveway/valve1") == 0)
  {
            if (payload[0] == '1')
        {
           digitalWrite(valve1Pin, HIGH);
           client.publish("esp32/driveway/valve1/state", "on");
        }
        //turn the light off if the payload is '0' and publish to the MQTT server a confirmation message
        else if (payload[0] == '0')
        {
           digitalWrite(valve1Pin, LOW);
           client.publish("esp32/driveway/valve1/state", "off");
        }
  }
  else if(strcmp(topic, "esp32/driveway/valve4") == 0)
  {
            if (payload[0] == '1')
        {
           digitalWrite(valve4Pin, HIGH);
           client.publish("esp32/driveway/valve4/state", "on");
        }
        //turn the light off if the payload is '0' and publish to the MQTT server a confirmation message
        else if (payload[0] == '0')
        {
           digitalWrite(valve4Pin, LOW);
           client.publish("esp32/driveway/valve4/state", "off");
        }
  }
  else if(strcmp(topic, "esp32/driveway/valve2") == 0)
  {
            if (payload[0] == '1')
        {
           digitalWrite(valve2Pin, HIGH);
           client.publish("esp32/driveway/valve2", "Valve On");
        }
        //turn the light off if the payload is '0' and publish to the MQTT server a confirmation message
        else if (payload[0] == '0')
        {
           digitalWrite(valve2Pin, LOW);
           client.publish("esp32/driveway/valve2", "Valve Off");
        }
  }
    else if(strcmp(topic, "esp32/driveway/valve3") == 0)
  {
            if (payload[0] == '1')
        {
           digitalWrite(valve3Pin, HIGH);
           client.publish("esp32/driveway/valve3", "Valve On");
        }
        //turn the light off if the payload is '0' and publish to the MQTT server a confirmation message
        else if (payload[0] == '0')
        {
           digitalWrite(valve3Pin, LOW);
           client.publish("esp32/driveway/valve3", "Valve Off");
        }
  }
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client",MQTT_USER,MQTT_PASSWD)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/driveway/valve1");
      client.subscribe("esp32/driveway/valve2");
      client.subscribe("esp32/driveway/valve3");            
      client.subscribe("esp32/driveway/valve4");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
      delay(5000);
    }
  }
}





void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  ArduinoOTA.handle();

  
  // read the pushbutton input pin:
  buttonState = digitalRead(buttonPin);

  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button went from off to on:
      buttonPushCounter++;
      Serial.println("on");
      client.publish("esp32/driveway/motion", "on");
 
      Serial.print("number of button pushes: ");
      Serial.println(buttonPushCounter);
    } else {
      // if the current state is LOW then the button went from on to off:
      Serial.println("off");
      client.publish("esp32/driveway/motion", "off");      
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;
  
  long now = millis();
  if (now - lastMsg > 30000) {
    lastMsg = now;
    
    // Read Humidity
    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println("%");
    humidity = bme.readHumidity();
    
    char humiString[8];
    dtostrf(humidity, 6, 0, humiString);
    client.publish("esp32/driveway/humidity", humiString);    
    // Temperature in Celsius
    temperature = bme.readTemperature();   
    // Uncomment the next line to set temperature in Fahrenheit 
    // (and comment the previous temperature line)
    //temperature = 1.8 * bme.readTemperature() + 32; // Temperature in Fahrenheit

    if (temperature < 50) {
      
      // Convert the value to a char array
      char tempString[8];
      dtostrf(temperature, 1, 2, tempString);
    
      Serial.print("Temperature = ");
      Serial.print(tempString);
      client.publish("esp32/driveway/temp", tempString);
      Serial.println(" *C");
    }
    pressure = bme.readPressure();

    if (pressure < 120000) {
          
      // Convert the value to a char array
      char preString[8];
      dtostrf(pressure, 6, 0, preString);
      Serial.print("Pressure = ");
      Serial.println(preString);
      client.publish("esp32/driveway/pressure", preString);
    }
    ADC_VALUE = analogRead(Analog_channel_pin);
    Serial.print("ADC VALUE = ");
    Serial.println(ADC_VALUE);
    delay(1000);
    voltage_value = (ADC_VALUE * 3.3 ) / (4395) ;
    vin = voltage_value / (R2/(R1+R2)) ; 
    Serial.print("Voltage = ");
    Serial.print(voltage_value);
    Serial.print("volts");
    char volString[8];    
    dtostrf(vin, 6, 0, volString);    
    client.publish("esp32/driveway/voltage", volString);
}    


}
