#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>           // MQTT support
#include <ESP8266WiFi.h>
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VEML6070.h"
#include "Adafruit_SI1145.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
extern "C" {
#include "user_interface.h"
}

#define LENG 31   //0x42 + 31 bytes equal to 32 bytes
#define SEALEVELPRESSURE_HPA (1013.25)

// Update the below parameters for your project
const char* ssid = "xxx";
const char* password = "xxx";
const char* mqtt_server = "192.168.1.xx"; 
const char* mqtt_user = "xxx";
const char* mqtt_password = "xxx";
const char* clientID = "weather";
const char* topicStatus = "/weather/status";
const char* topicUVI = "/weather/uvi";
const char* topicUVIndex = "/weather/uvindex";
const char* topicPM1 = "/weather/pm1";
const char* topicPM2_5 = "/weather/pm2_5";
const char* topicPM10 = "/weather/pm10";
const char* topicLight = "/weather/light";
const char* topicIR = "/weather/infrared";
const char* topicTemperature = "/weather/temperature";
const char* topicHumidity = "/weather/humidity";
const char* topicPressure = "/weather/pressure";

unsigned char buf[LENG];
unsigned long lastTick, uptime, seconds;
char msg[50];
String message = "";
String webPage = "";
String webStat = "";
String webSensors = "";
String webFooter = "";
String mqttStat = "";

 
int PM01Value=0;          //define PM1.0 value of the air detector module
int PM2_5Value=0;         //define PM2.5 value of the air detector module
int PM10Value=0;         //define PM10 value of the air detector module

Adafruit_VEML6070 uv = Adafruit_VEML6070();
Adafruit_SI1145 uva = Adafruit_SI1145();
Adafruit_BME280 bme; // I2C

os_timer_t myTimer;
MDNSResponder mdns;
ESP8266WebServer server(80);
WiFiClient espClient;
PubSubClient mqtt(mqtt_server, 1883, 0, espClient);



// This routing just puts the status string together which will be sent over MQTT
void refreshStats() {
  // Initialize the strings for MQTT 
  mqttStat = "{\"rssi\":";
  mqttStat += WiFi.RSSI();
  webStat = "<p style=\"font-size: 90%; color: #FF8000;\">RSSI: ";
  webStat += WiFi.RSSI();
  webStat += "<br/>";
  
  mqttStat += ",\"uptime\":";
  mqttStat += uptime;
  mqttStat += "}";
  webStat += "Uptime [min]: ";
  webStat += uptime;
  webStat += "<br/>";
  webStat += "</p>";
}

// This is the 1 second timer callback function
uint8_t sec=0;
void timerCallback(void *pArg) {
  sec++;
  seconds++;
  if (seconds==10) {
    // Send MQTT update
    readSensors();
    refreshStats();
    if (mqtt_server!="") {
      mqtt.publish(topicStatus, mqttStat.c_str());
      Serial.print(F("Status: "));
      Serial.println(mqttStat);
    }
    
    seconds = 0;
  }
}

// MQTT reconnect logic
void reconnect() {
  //String mytopic;
  // Loop until we're reconnected
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt.connect(clientID, mqtt_user, mqtt_password)) {
      Serial.println(F("connected"));
      // ... and resubscribe
      //mqtt.subscribe(topicSleep);
    } else {
      Serial.print(F("failed, rc="));
      Serial.print(mqtt.state());
      Serial.println(F(" try again in 5 seconds"));
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Convert the incoming byte array to a string
  String strTopic = String((char*)topic);
  payload[length] = '\0'; // Null terminator used to terminate the char array
  String message = (char*)payload;

  Serial.print(F("Message arrived on topic: ["));
  Serial.print(topic);
  Serial.print(F("], "));
  Serial.println(message);

}

void setup()
{
  Serial.begin(9600);   
  delay(10);
  Serial.println();
  Serial.println("Weather station: BME280, Si1145, VEML6959, PMS5003");
  uptime = 0;
  sec = 0;
  seconds = 0;

  webPage = "<html><body><h1>ESP8266 Weather Station</h1><p>This sketch gets sensor readings from BME280, Si1145, VEML6959, PMS5003 and publishes them to MQTT.</p>";
  webSensors = "<p style=\"font-size: 90%; color: #007FFF;\">No sensor readings available yet.</p>";
  webStat = "<p style=\"font-size: 90%; color: #FF8000;\">No stats available yet.</p>";
  webFooter = "<p style=\"font-size: 80%; color: #08088A;\">ESP8266 Weather Station v1.0 | <a href=\"mailto:csongor.varga@gmail.com\">email me</a></p></body></html>";


  Serial.print(F("Connecting to Wifi"));
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
    seconds++;
    if (seconds>180) {
      // reboot the ESP if cannot connect to wifi
      ESP.restart();
    }
  }
  seconds = 0;
  Serial.println("");
  Serial.print(F("Connected to "));
  Serial.println(ssid);
  Serial.print(F("IP address: "));
  Serial.println(WiFi.localIP());
  Serial.print(F("Signal [RSSI]: "));
  Serial.println(WiFi.RSSI());

  os_timer_setfn(&myTimer, timerCallback, NULL);
  os_timer_arm(&myTimer, 1000, true);

  // Set up the MDNS and the HTTP server
  if (mdns.begin("Weather", WiFi.localIP())) {
    Serial.println(F("MDNS responder started"));
  }  
  server.on("/", [](){                        // landing page
    server.send(200, "text/html", webPage+webSensors+webStat+webFooter);
  });
  server.begin();
  Serial.println(F("HTTP server started"));  

  // Set up the MQTT server connection
  if (mqtt_server!="") {
    mqtt.setServer(mqtt_server, 1883);
    mqtt.setCallback(callback);
  }

  uv.begin(VEML6070_1_T);  // pass in the integration time constant
  Serial.println("VEML6070 initialized");
  if (! uva.begin()) {
    Serial.println("Didn't find Si1145");
  } else {
    Serial.println("Si1145 initialized");
  }
  bool status;
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  } else {
    Serial.println("BME 280 initialized");
  }
}
 
void loop() {

  // Handle HTTP server requests
  server.handleClient();

  // Handle MQTT connection/reconnection
  if (mqtt_server!="") {
    if (!mqtt.connected()) {
      reconnect();
    }
    mqtt.loop();
  }

  // Uptime calculation
  if (millis() - lastTick >= 60000) {            
    lastTick = millis();            
    uptime++;            
  }    
  
  if(Serial.find(0x42)){    //start to read when detect 0x42
    Serial.readBytes(buf,LENG);
 
    if(buf[0] == 0x4d){
      if(checkValue(buf,LENG)){
        PM01Value=transmitPM01(buf); //count PM1.0 value of the air detector module
        PM2_5Value=transmitPM2_5(buf);//count PM2.5 value of the air detector module
        PM10Value=transmitPM10(buf); //count PM10 value of the air detector module 
      }           
    } 
  }

 
}

void readSensors() {

  // PMS5003 dust sensor
  webSensors = "<p style=\"font-size: 90%; color: #007FFF;\"><b>PMS5003 air dust sensor</b><br/>";
  Serial.print("PM1.0: ");  
  Serial.print(PM01Value);
  Serial.println("  ug/m3");  
  mqtt.publish(topicPM1, String(PM01Value).c_str());  
  webSensors+= "PM1.0: ";
  webSensors+= PM01Value;
  webSensors+= " &micro;g/m&sup3;<br/>";       

  Serial.print("PM2.5: ");  
  Serial.print(PM2_5Value);
  Serial.println("  ug/m3");     
  mqtt.publish(topicPM2_5, String(PM2_5Value).c_str());
  webSensors+= "PM2.5: ";
  webSensors+= PM2_5Value;
  webSensors+= " &micro;g/m&sup3;<br/>";       

  Serial.print("PM10 : ");  
  Serial.print(PM10Value);
  Serial.println("  ug/m3");  
  mqtt.publish(topicPM10, String(PM10Value).c_str()); 
  webSensors+= "PM10: ";
  webSensors+= PM10Value;
  webSensors+= " &micro;g/m&sup3;</p>";       

  // VEML6070 UV sensor
  webSensors+= "<p style=\"font-size: 90%; color: #007FFF;\"><b>VEML6070 UV sensor</b><br/>";
  long uvi = uv.readUV();
  Serial.print("UVI            : "); 
  Serial.println(uvi);
  mqtt.publish(topicUVI, String(uvi).c_str());
  webSensors+= "UVI: ";
  webSensors+= uvi;
  webSensors+= "<br/>";  
  Serial.print("UV index       : "); 
  Serial.println(uvi / 187);
  mqtt.publish(topicUVIndex, String(uvi / 187).c_str());
  webSensors+= "UV Index: ";
  webSensors+= uvi / 187;
  webSensors+= "</p>";  
  
  // Si1145 Light sensor
  webSensors+= "<p style=\"font-size: 90%; color: #007FFF;\"><b>Si1145 light sensor</b><br/>";
  long light = uva.readVisible();
  Serial.print("Light intensity: "); Serial.println(light);
  mqtt.publish(topicLight, String(light).c_str());
  webSensors+= "Light intensity: ";
  webSensors+= light;
  webSensors+= "<br/>";  
  long ir = uva.readIR();
  Serial.print("Infra red      : "); Serial.println(ir);
  mqtt.publish(topicIR, String(ir).c_str());
  webSensors+= "IR intensity: ";
  webSensors+= ir;
  webSensors+= "</p>";  
  
  // BME 280 temperature, humidity and pressure
  webSensors+= "<p style=\"font-size: 90%; color: #007FFF;\"><b>BME 280 environmental sensor</b><br/>";
  float temp = bme.readTemperature();
  Serial.print("Temperature    : ");
  Serial.print(temp);
  Serial.println(" *C");
  mqtt.publish(topicTemperature, String(temp).c_str());
  webSensors+= "Temperature: ";
  webSensors+= temp;
  webSensors+= " &deg;C<br/>";  
    
  // Convert temperature to Fahrenheit
  /*Serial.print("Temperature = ");
  Serial.print(1.8 * bme.readTemperature() + 32);
  Serial.println(" *F");*/

  float pressure = bme.readPressure() / 100.0F;
  Serial.print("Pressure       : ");
  Serial.print(pressure);
  Serial.println(" hPa");
  mqtt.publish(topicPressure, String(pressure).c_str());
  webSensors+= "Pressure: ";
  webSensors+= pressure;
  webSensors+= " hPa<br/>";  
  
  float humidity = bme.readHumidity();
  Serial.print("Humidity       : ");
  Serial.print(humidity);
  Serial.println(" %");
  Serial.println();
  mqtt.publish(topicHumidity, String(humidity).c_str());
  webSensors+= "Humidity: ";
  webSensors+= humidity;
  webSensors+= " &#37;<p/>"; 
  webSensors+= "<p style=\"font-size: 90%; color: #007FFF;\">Sensor data does not auto update on this page</p>"; 
}
  
char checkValue(unsigned char *thebuf, char leng)
{  
  char receiveflag=0;
  int receiveSum=0;
 
  for(int i=0; i<(leng-2); i++){
  receiveSum=receiveSum+thebuf[i];
  }
  receiveSum=receiveSum + 0x42;
 
  if(receiveSum == ((thebuf[leng-2]<<8)+thebuf[leng-1]))  //check the serial data 
  {
    receiveSum = 0;
    receiveflag = 1;
  }
  return receiveflag;
}
int transmitPM01(unsigned char *thebuf)
{
  int PM01Val;
  PM01Val=((thebuf[3]<<8) + thebuf[4]); //count PM1.0 value of the air detector module
  return PM01Val;
}
//transmit PM Value to PC
int transmitPM2_5(unsigned char *thebuf)
{
  int PM2_5Val;
  PM2_5Val=((thebuf[5]<<8) + thebuf[6]);//count PM2.5 value of the air detector module
  return PM2_5Val;
  }
//transmit PM Value to PC
int transmitPM10(unsigned char *thebuf)
{
  int PM10Val;
  PM10Val=((thebuf[7]<<8) + thebuf[8]); //count PM10 value of the air detector module  
  return PM10Val;
}
