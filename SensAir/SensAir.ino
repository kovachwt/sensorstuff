#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <SdsDustSensor.h>
#include <MqttClient.h>
#include <Adafruit_BME680.h>
#include <Wire.h>
#include <SPI.h>

//#include <OneWire.h>
//#include <DallasTemperature.h>

#define HW_UART_SPEED                 115200L
#define WLAN_SSID        "sensair"
#define WLAN_PASS        "********"
#define MQTT_ID          "sensair"
#define MQTT_SERVER      "5.196.95.208"
#define MQTT_SERVERPORT  1883                   // use 8883 for SSL
#define MQTT_USERNAME    "sensair"
#define MQTT_PASSWORD    "********"

// determine SDS011 DutyCycle = SDS011_WORK_SECONDS / (SDS011_WORK_SECONDS + SLEEP_SECONDS)
#define SDS011_WORK_SECONDS    30              // should be at least 30 if SDS011 is connected
#define SLEEP_SECONDS          30              // should be at least 30 if SDS011 is connected

// this is usually 4 degrees Celsius, because of the heating element on the BME680
#define BME680_TEMP_OFFSET     -4


static MqttClient *mqtt = NULL;
static WiFiClient network;
// ============== Object to supply system functions ============================
class System: public MqttClient::System {
public:

  unsigned long millis() const {
    return ::millis();
  }

  void yield(void) {
    ::yield();
  }
};

//#define ONE_WIRE_BUS       D4
//OneWire oneWire(ONE_WIRE_BUS);
//DallasTemperature sensors(&oneWire);

int rxPin = D6;
int txPin = D7;

//#define BME_SCK D1
//#define BME_MISO D6
//#define BME_MOSI D2
//#define BME_CS D5

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);
SdsDustSensor sds(rxPin, txPin);

int co2pwmPin = D5;
volatile unsigned long co2timehigh, co2timelow, co2prevhigh, co2prevlow, co2ppm, prevco2ppm, publishco2ppm = 0;
volatile byte donerising, donefalling = 0;

ICACHE_RAM_ATTR void co2rising() {
  attachInterrupt(digitalPinToInterrupt(co2pwmPin), co2falling, FALLING);

  co2prevhigh = millis();
  co2timelow = co2prevhigh - co2prevlow;
}
 
ICACHE_RAM_ATTR void co2falling() {
  attachInterrupt(digitalPinToInterrupt(co2pwmPin), co2rising, RISING);

  co2prevlow = millis();
  co2timehigh = co2prevlow - co2prevhigh;

  prevco2ppm = co2ppm;
  co2ppm = 5000 * (co2timehigh - 2) / (co2timehigh + co2timelow - 4);

  if (prevco2ppm > 0 && co2timehigh > 0 && co2timelow > 0) {
    publishco2ppm = co2ppm;
  }
}

void setup() {
  delay(1000);
  Serial.begin(115200);

  //pinMode(ONE_WIRE_BUS, INPUT_PULLUP);

  sds.begin();
  Serial.println(sds.setQueryReportingMode().toString()); // ensures sensor is in 'query' reporting mode

  bme.begin();
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  pinMode(co2pwmPin, INPUT); 
  digitalWrite(co2pwmPin, HIGH);
  attachInterrupt(digitalPinToInterrupt(co2pwmPin), co2rising, RISING);

  // Setup WiFi network
  WiFi.mode(WIFI_STA);
  WiFi.hostname("ESP_" MQTT_ID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("Connected to WiFi, IP: ");
  Serial.println(WiFi.localIP().toString().c_str());

  // Setup MqttClient
  MqttClient::System *mqttSystem = new System;
  MqttClient::Logger *mqttLogger = new MqttClient::LoggerImpl<HardwareSerial>(Serial);
  MqttClient::Network * mqttNetwork = new MqttClient::NetworkClientImpl<WiFiClient>(network, *mqttSystem);
  //// Make 128 bytes send buffer
  MqttClient::Buffer *mqttSendBuffer = new MqttClient::ArrayBuffer<128>();
  //// Make 128 bytes receive buffer
  MqttClient::Buffer *mqttRecvBuffer = new MqttClient::ArrayBuffer<128>();
  //// Allow up to 2 subscriptions simultaneously
  MqttClient::MessageHandlers *mqttMessageHandlers = new MqttClient::MessageHandlersImpl<2>();
  //// Configure client options
  MqttClient::Options mqttOptions;
  ////// Set command timeout to 10 seconds
  mqttOptions.commandTimeoutMs = 10000;
  //// Make client object
  mqtt = new MqttClient(
    mqttOptions, *mqttLogger, *mqttSystem, *mqttNetwork, *mqttSendBuffer,
    *mqttRecvBuffer, *mqttMessageHandlers
  );

}

void loop() {
  if (!mqtt->isConnected()) {
    // Close connection if exists
    network.stop();
    // Re-establish TCP connection with MQTT broker
    Serial.println("Connecting");
    network.connect(MQTT_SERVER, MQTT_SERVERPORT);
    if (!network.connected()) {
      Serial.println("Can't establish the TCP connection");
      delay(5000);
      ESP.reset();
    }
    // Start new MQTT connection
    MqttClient::ConnectResult connectResult;
    // Connect
    {
      MQTTPacket_connectData options = MQTTPacket_connectData_initializer;
      options.MQTTVersion = 4;
      options.clientID.cstring = (char*)MQTT_ID;
      options.username.cstring = (char*)MQTT_USERNAME;
      options.password.cstring = (char*)MQTT_PASSWORD;
      options.cleansession = true;
      options.keepAliveInterval = 15; // 15 seconds
      MqttClient::Error::type rc = mqtt->connect(options, connectResult);
      if (rc != MqttClient::Error::SUCCESS) {
        Serial.print("Connection error: ");
        Serial.println(rc);
        return;
      }
    }
  }
  
  // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
  Serial.print(F("BME680 reading started at "));
  Serial.print(millis());
  Serial.print(F(" and will finish at "));
  Serial.println(endTime);

  Serial.println("Waking up SDS011");
  sds.wakeup();

  // wait at least 30 seconds for SDS sensor to work a little
  Serial.print("Waiting "); Serial.print(SDS011_WORK_SECONDS); Serial.println(" seconds for sensor to work");
  if (mqtt->isConnected()) {
    mqtt->yield(1000L * SDS011_WORK_SECONDS);
  } else {
    delay(1000 * SDS011_WORK_SECONDS);
  }

  PmResult pm = sds.queryPm();
  if (pm.isOk()) {
    float pm25 = pm.pm25;
    float pm10 = pm.pm10;
    Serial.print("PM2.5 = ");
    Serial.print(pm25);
    Serial.print(", PM10 = ");
    Serial.println(pm10);
    publishMessage("k0doma/dust2", String(pm25).c_str());
    publishMessage("k0doma/dust10", String(pm10).c_str());
  } else {
    Serial.print("Could not read values from sensor, reason: ");
    Serial.println(pm.statusToString());
  }
  Serial.println("Putting the SDS011 to sleep.");
  WorkingStateResult state = sds.sleep();


  //sensors.begin();
  //sensors.setResolution(12);
  //Serial.print("Requesting temperatures...");
  //sensors.requestTemperatures(); // Send the command to get temperatures
  //Serial.println("DONE");
  //Serial.print("Temperature for the device 1 (index 0) is: ");
  //Serial.println(sensors.getTempCByIndex(0));  
  //pinMode(ONE_WIRE_BUS, INPUT_PULLUP);


  if (bme.endReading()) {
    Serial.print(F("Reading completed at "));
    Serial.println(millis());

    float temperature = bme.temperature + BME680_TEMP_OFFSET;
    Serial.print(F("Temperature = "));
    Serial.print(temperature);
    Serial.println(F(" *C"));
    publishMessage("k0doma/temp", String(temperature).c_str());

    float pressure = bme.pressure / 100.0;
    Serial.print(F("Pressure = "));
    Serial.print(pressure);
    Serial.println(F(" hPa"));
    publishMessage("k0doma/pressure", String(pressure).c_str());

    float humidity = bme.humidity;
    Serial.print(F("Humidity = "));
    Serial.print(humidity);
    Serial.println(F(" %"));
    publishMessage("k0doma/humid", String(humidity).c_str());

    float gas = bme.gas_resistance / 1000.0;
    Serial.print(F("Gas = "));
    Serial.print(gas);
    Serial.println(F(" KOhms"));
    publishMessage("k0doma/volatile", String(gas).c_str());
  
    Serial.print(F("Approx. Altitude = "));
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(F(" m"));
  } else {    
    Serial.println(F("Failed to complete reading :("));
  }

  if (publishco2ppm > 0) {
    long co2val = publishco2ppm;
    publishco2ppm = 0;

    Serial.print(F("CO2 = "));
    Serial.print(co2val);
    Serial.println(F(" PPM"));
    publishMessage("k0doma/co2", String(co2val).c_str());
  } else {
    Serial.print(F("CO2 NO co2timelow="));
    Serial.println(co2timelow);
    Serial.print(F("CO2 NO co2timehigh="));
    Serial.println(co2timehigh);
    Serial.print(F("CO2 NO co2ppm="));
    Serial.println(co2ppm);
  }

  Serial.println();



  if (state.isWorking()) {
    Serial.println("Problem with sleeping the sensor.");
  } else {
    Serial.print("Sensor is sleeping for "); Serial.print(SLEEP_SECONDS); Serial.println(" seconds.");

    // wait 30 seconds
    if (mqtt->isConnected()) {
      mqtt->yield(1000L * SLEEP_SECONDS);
    } else {
      delay(1000 * SLEEP_SECONDS);
    }
  }
  Serial.println();
}

void publishMessage(const char *topic, const char *buf) {
  if (mqtt->isConnected()) {
    MqttClient::Message message;
    message.qos = MqttClient::QOS1;
    message.retained = true;
    //message.dup = false;
    message.payload = (void*) buf;
    message.payloadLen = strlen(buf);
    mqtt->publish(topic, message);
    Serial.print("Published "); Serial.print(buf); Serial.print(" to "); Serial.println(topic);
  }
}
