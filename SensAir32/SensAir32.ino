#include <Arduino.h>
#include <Wire.h>
#include <esp_task_wdt.h>

#include <Adafruit_BME680.h>
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>
#include <MHZ19.h>
#include <SdsDustSensor.h>
#include <BH1750.h>
// #include <bme68x.h>
// #include <bme68x_defs.h>

// #include <SPI.h>
// #include <Zanshin_BME680.h>
// #include <OneWire.h>
// #include <DallasTemperature.h>

#include "ethmqtt.h"

// #define ONE_WIRE_BUS       D4
//  OneWire oneWire(ONE_WIRE_BUS);
//  DallasTemperature sensors(&oneWire);

// determine SDS011 DutyCycle = SDS011_WORK_SECONDS / (SDS011_WORK_SECONDS + SLEEP_SECONDS)
#define SDS011_WORK_SECONDS 30     // should be at least 30 if SDS011 is connected
#define SDS_READ_EVERY_X_CYCLES 4  // should be at least 30 if SDS011 is connected

// int sdsRxPin = 16;
// int sdsTxPin = 17;
// SoftwareSerial sdsSerial(sdsRxPin, sdsTxPin);               // (Uno example) create device to MH-Z19 serial
SdsDustSensor sds(Serial1);

// this is usually 4 degrees Celsius, because of the heating element on the BME680
#define BME680_TEMP_OFFSET 1.0

#define SEALEVELPRESSURE_HPA (1013.25)

#define BME_SCK 18   // SCL
#define BME_MISO 19  // SDO
#define BME_MOSI 23  // SDA
#define BME_CS 5

#define BME_SCL 18  // SCL
#define BME_SDA 23  // SDA

Adafruit_BME680 bme;  // I2C
// Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

Adafruit_BME280 bme280;  // I2C

BH1750 lightMeter;

/*
#define CO2_ANALOG_PIN 34
*/
#define CO2_RX_PIN 26                              // Rx pin which the MHZ19 Tx pin is attached to
#define CO2_TX_PIN 25                              // Tx pin which the MHZ19 Rx pin is attached to
#define CO2_BAUDRATE 9600                          // Device to MH-Z19 Serial baudrate (should not be changed)
MHZ19 myMHZ19;                                     // Constructor for library
SoftwareSerial co2Serial(CO2_RX_PIN, CO2_TX_PIN);  // (Uno example) create device to MH-Z19 serial

unsigned bme280status;


void setup() {
  delay(500);
  Serial.begin(115200);
  Serial.println("BOOT");

  Wire.setPins(BME_SDA, BME_SCL);
  Wire.begin();

  lightMeter.begin();


  // pinMode(ONE_WIRE_BUS, INPUT_PULLUP);

  Serial.println("Beginning SDS");

  sds.begin();
  Serial.println(sds.setQueryReportingMode().toString());  // ensures sensor is in 'query' reporting mode

  Serial.println("Beginning BME");
  bme.begin();
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);  // 320*C for 150 ms


  // BME280 default settings
  bme280status = bme280.begin();
  if (!bme280status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x");
    Serial.println(bme280.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.println("        ID of 0x61 represents a BME 680.\n");
  }

  Serial.println("Beginning CO2 sensor");
  co2Serial.begin(CO2_BAUDRATE);  // (Uno example) device to MH-Z19 serial start
  myMHZ19.begin(co2Serial);       // *Serial(Stream) reference must be passed to library begin().
  myMHZ19.autoCalibration();      // Turn auto calibration ON (OFF autoCalibration(false))
  //pinMode(CO2_ANALOG_PIN, INPUT);
  //analogReadResolution(12);

  Serial.println("Enabling watchdog");

  delay(1000);                  /* Done so that the Arduino doesn't keep resetting infinitely in case of wrong configuration */
  esp_task_wdt_init(60, true);  // enable 60 second watchdog
  esp_task_wdt_add(NULL);       // add current thread to WDT watch

  Serial.println("Starting network");
  networkSetup();
}

int cyclenum = 0;

void loop() {
  esp_task_wdt_reset(); /* Reset the watchdog */
  reconnect();

  if (cyclenum % SDS_READ_EVERY_X_CYCLES == 1) {
    PmResult pm = sds.queryPm();
    if (pm.isOk()) {
      float pm25 = pm.pm25;
      float pm10 = pm.pm10;
      Serial.print("PM2.5 = ");
      Serial.print(pm25);
      Serial.print(", PM10 = ");
      Serial.println(pm10);

      mqttpublish(&mqtt, FEEDPM25, pm25, 2);
      mqttpublish(&mqtt, FEEDPM10, pm10, 2);
    } else {
      Serial.print("Could not read values from sensor, reason: ");
      Serial.println(pm.statusToString());
    }
    Serial.println("Putting the SDS011 to sleep.");
    WorkingStateResult state = sds.sleep();
  }

  // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
  Serial.print(F("BME680 reading started at "));
  Serial.print(millis());
  Serial.print(F(" and will finish at "));
  Serial.println(endTime);

  float lux = lightMeter.readLightLevel();
  Serial.print(F("Light lux = "));
  Serial.println(lux);
  if (lux > -1) {
    Serial.print(F("LUX = "));
    Serial.println(lux);
    mqttpublish(&mqtt, FEEDLUX, lux, 2);
  }

  //int analogValue = analogRead(CO2_ANALOG_PIN);
  //Serial.print(F("CO2 analogValue = "));
  //Serial.println(analogValue);
  // int co2val = myMHZ19.getCO2(); // Request CO2 (as ppm)
  //int co2val = (int)((float)(3300 * analogValue / 4096) * 0.85245); // calibrate here
  int co2val = myMHZ19.getCO2();
  if (co2val > 0) {
    Serial.print(F("CO2 = "));
    Serial.print(co2val);
    Serial.println(F(" PPM"));
    mqttpublish(&mqtt, FEEDCO2, co2val, 2);

    float co2Temp = myMHZ19.getTemperature();  // Request Temperature (as Celsius)
    Serial.print("CO2 Temperature (C): ");
    Serial.println(co2Temp);
    mqttpublish(&mqtt, FEEDTEMP3, co2Temp, 2);
  } else {
    Serial.print(F("NO CO2 co2val="));
    Serial.println(co2val);
  }

  int delaymore = bme.remainingReadingMillis();
  if (delaymore > 0)
    delayMqtt(delaymore);

  Serial.println("Reading BME680 sensor.");
  if (!bme.endReading())
    Serial.println(F("Failed to complete reading :("));
  else {
    float temperature = bme.temperature - BME680_TEMP_OFFSET;
    float humid = bme.humidity;
    float pressure = bme.pressure / 100.0;
    float voc = bme.gas_resistance / 1000.0;
    // float iaq = bme.

    Serial.print(F("Temperature = "));
    Serial.print(temperature);
    Serial.println(F(" *C"));
    mqttpublish(&mqtt, FEEDTEMP, temperature, 2);

    Serial.print(F("Humidity = "));
    Serial.print(humid);
    Serial.println(F(" %"));
    mqttpublish(&mqtt, FEEDHUMID, humid, 2);

    Serial.print(F("Gas = "));
    Serial.print(voc);
    Serial.println(F(" KOhms"));
    mqttpublish(&mqtt, FEEDVOC, voc, 2);
  }
  Serial.println();

  if (bme280status) {
    float temperature280 = bme280.readTemperature();
    float humid280 = bme280.readHumidity();
    if (temperature280 > 0 || humid280 > 0) {
      Serial.print(F("Temperature280 = "));
      Serial.print(temperature280);
      Serial.println(F(" *C"));
      mqttpublish(&mqtt, FEEDTEMP2, temperature280, 2);

      Serial.print(F("Humidity280 = "));
      Serial.print(humid280);
      Serial.println(F(" %"));
      mqttpublish(&mqtt, FEEDHUMID2, humid280, 2);
    }
  }

  if (cyclenum % SDS_READ_EVERY_X_CYCLES == 0) {
    Serial.println("Waking up SDS011");
    sds.wakeup();
    // wait at least 30 seconds for SDS sensor to work a little
    Serial.print("Waiting ");
    Serial.print(SDS011_WORK_SECONDS);
    Serial.println(" seconds for sensor to work");
  } else {
    Serial.print("Waiting ");
    Serial.print(SDS011_WORK_SECONDS);
    Serial.println(" seconds for new loop");
  }
  delayMqtt(1000 * SDS011_WORK_SECONDS);
  esp_task_wdt_reset(); /* Reset the watchdog */

  cyclenum++;
}

void mqttcallback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Got MQTT packet on topic=");
  Serial.print(topic);
  Serial.print(" length=");
  Serial.print(length);
  Serial.print(" with value=");
  if (length == 0) {
    Serial.println("ZERO_LENGTH");
    return;
  }

  payload[length] = '\0';  // Null terminator used to terminate the char array
  // String gotval = String((char *)payload);

  if (strcmp((char *)payload, "") == 0) {
    Serial.println("EMPTY");
    return;
  }

  /*
  if (strcmp(topic, "SUBTIMESYN") == 0)
  {
    Serial.println((char *)payload);
    long val = atol((char *)payload);
  }
  else*/
  {
    Serial.println((char *)payload);
    Serial.println("WARNING, UNKNOWN TOPIC!");
  }
}
