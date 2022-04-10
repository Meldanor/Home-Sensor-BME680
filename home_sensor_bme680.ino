/*

THIS PROGRAM IS BASED ON THE EXAMPLES FOR MQTT AND BME680 SENSOR. IT READS THE SENSOR DATA FROM BME680
(WIRING DIAGRAM FOLLOWS) AND SENDS THEM VIA MQTT TO A MQTT SERVER.

IT PUBLISHES THE FOLLOWING DATA:

  - TEMPERATURE IN °C
  - PRESSURE IN khPa
  - HUMIDITY IN %
  - GAS RESISTANCE IN kOhm
  - IAQ (AIR QUALITY) - RAW AND INTERPRETED -> LOWER IS BETTER QUALITY
  - IAQ ACCURACCY - RAW AND INTERPRETED -> DIFFERENT STATS TO INDICATE HOW TRUSTWORTHY IS THE MEASUREMENT

THE WIRING IS THE FOLLOWING:

NODE MCU ESP 8266 12-F AS THE COMPUTER
JOY-IC BME680 AS THE SENSOR

SENSOR    BOARD
===============
VCC       3V
GND       G
SCL       D4
SDA       D3
SD0       G (THIS ENABLES USING I2C)
*/

#include <bsec.h>

#include "secrets.h"
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);
 
// Create an object of the class Bsec
Bsec iaqSensor;

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiClientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_SERVERPORT, MQTT_USERNAME, MQTT_KEY);

/****************************** Feeds ***************************************/
#define PREFIX "home_sensor/"
Adafruit_MQTT_Publish temperatureFeed = Adafruit_MQTT_Publish(&mqtt, PREFIX HOST_NAME "/temperature");
Adafruit_MQTT_Publish pressureFeed = Adafruit_MQTT_Publish(&mqtt, PREFIX HOST_NAME "/pressure");
Adafruit_MQTT_Publish humidityFeed = Adafruit_MQTT_Publish(&mqtt, PREFIX HOST_NAME "/humidity");
Adafruit_MQTT_Publish gasResistanceFeed = Adafruit_MQTT_Publish(&mqtt, PREFIX HOST_NAME "/gasResistance");
Adafruit_MQTT_Publish iaqRawFeed = Adafruit_MQTT_Publish(&mqtt, PREFIX HOST_NAME "/iaqRaw");
Adafruit_MQTT_Publish iaqAccuracyRawFeed = Adafruit_MQTT_Publish(&mqtt, PREFIX HOST_NAME "/iaqAccuracyRaw");
Adafruit_MQTT_Publish iaqFeed = Adafruit_MQTT_Publish(&mqtt, PREFIX HOST_NAME "/iaq");
Adafruit_MQTT_Publish iaqAccuracyFeed = Adafruit_MQTT_Publish(&mqtt, PREFIX HOST_NAME "/iaqAccuracy");

/******* Measured Values *********/
float temperature = 0;
float pressure = 0;
float humidity = 0;
float gasResistance = 0;
float iaq = 0;
uint8_t iaqAccuracy = 0;
String iaqString = "";
String iaqAccuracyString = "";

/** TIMER */
// Send the data via MQTT only every 10 seconds
#define MQTT_TIMER_MS 10000
unsigned long lastLoop = 0l;
uint32_t timer = MQTT_TIMER_MS;

void setup(void) {
  Serial.begin(9600);
  setupBme680();
  connectWlan();
}

void setupBme680() {
  Wire.begin(0, 2);
  iaqSensor.begin(BME680_I2C_ADDR_PRIMARY, Wire);
  checkIaqSensorStatus();
 
  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };
 
  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();
}

void connectWlan() {

  Serial.println("======");
  WiFi.hostname(HOST_NAME);
  Serial.print("Connect to ");
  Serial.println(F(WLAN_SSID));
  WiFi.begin(WLAN_SSID, WLAN_PASS);
   while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  Serial.println("======");
}
 
// Function that is looped forever
void loop(void)
{  
  unsigned long curLoop = millis();
  timer = curLoop - lastLoop;
  lastLoop = curLoop;
  if (iaqSensor.run()) { // If new data is available
    temperature = iaqSensor.temperature;
    pressure = iaqSensor.pressure / 100.0;
    humidity = iaqSensor.humidity;
    gasResistance = iaqSensor.gasResistance / 1000.0;
    iaq = iaqSensor.iaq;
    iaqAccuracy = iaqSensor.iaqAccuracy;
    iaqString = readableIaq(iaq);
    iaqAccuracyString = readableAccuracy(iaqAccuracy);
    printMeasuredValues();

    if (timer <= 0) {
      Serial.println("Send data via MQTT");
      timer = MQTT_TIMER_MS;
      sendValuesViaMqtt();
      Serial.println("Finished sending data via MQTT!");
    }

 } else {
    checkIaqSensorStatus();
  }
}

void sendValuesViaMqtt() {
  MQTT_connect();
  temperatureFeed.publish(temperature);
  pressureFeed.publish(pressure);
  humidityFeed.publish(humidity);
  gasResistanceFeed.publish(gasResistance);
  iaqRawFeed.publish(iaq);
  iaqAccuracyRawFeed.publish(iaqAccuracy);
  iaqFeed.publish(iaqString.c_str());
  iaqAccuracyFeed.publish(iaqAccuracyString.c_str());
}

void printMeasuredValues() {

  Serial.print(F("Temperature = "));
  Serial.print(temperature);
  Serial.println(F(" *C"));

  Serial.print(F("Pressure = "));
  Serial.print(pressure);
  Serial.println(F(" khPa"));

  Serial.print(F("Humidity = "));
  Serial.print(humidity);
  Serial.println(F(" %"));

  Serial.print(F("Gas = "));
  Serial.print(gasResistance);
  Serial.println(F(" KOhms"));
  
  Serial.print(F("IAQ(Raw) = "));
  Serial.println(iaq);
  
  Serial.print(F("IAQ = "));
  Serial.println(iaqString);

  Serial.print(F("IAQ_Accuracy(Raw) = "));
  Serial.println(iaqAccuracy);

  Serial.print(F("IAQ_Accuracy = "));
  Serial.println(iaqAccuracyString);

  Serial.println();
}

String readableIaq(float iaq) {
  if (iaq >= 0.0 && iaq <= 50.0) {
    return "Excellent";
  } else if (iaq > 50.0 && iaq <= 100.0) {
    return "Good";
  } else if (iaq > 100.0 && iaq <= 150.0) {
    return "Lightly polluted";
  } else if (iaq > 150.0 && iaq <= 200.0) {
    return "Moderately polluted";
  } else if (iaq > 200.0 && iaq <= 250.0) {
    return "Heavily polluted";
  } else if (iaq > 250.0 && iaq <= 350.0) {
    return "Severely polluted";
  } else if(iaq > 350) {
    return "Extremeley polluted";
  } else {
    return "UNKNOWN";
  }
}

String readableAccuracy(uint8_t iaqAccuracy) {
  switch(iaqAccuracy) {
    case 0: 
      return "Started";
    case 1:
      return "Uncertain";
    case 2:
      return "Calibrating";
    case 3:
      return "Calibrated";
    default:
      return "UNKNOWN";
  }
}
 
// Helper function definitions
void checkIaqSensorStatus(void)
{
  String output;
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }
 
  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
}
 
void errLeds(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}