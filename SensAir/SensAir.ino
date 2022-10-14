#include <Arduino.h>
#include <SdsDustSensor.h>
#include <Wire.h>
//#include <SPI.h>
#include <Zanshin_BME680.h>

//#include <OneWire.h>
//#include <DallasTemperature.h>

#define HW_UART_SPEED 115200L

#include "ethmqtt.h"

// determine SDS011 DutyCycle = SDS011_WORK_SECONDS / (SDS011_WORK_SECONDS + SLEEP_SECONDS)
#define SDS011_WORK_SECONDS 30 // should be at least 30 if SDS011 is connected
#define SLEEP_SECONDS 90       // should be at least 30 if SDS011 is connected

// this is usually 4 degrees Celsius, because of the heating element on the BME680
#define BME680_TEMP_OFFSET 4.0

//#define ONE_WIRE_BUS       D4
// OneWire oneWire(ONE_WIRE_BUS);
// DallasTemperature sensors(&oneWire);

int rxPin = D6;
int txPin = D7;

//#define BME_SCK D1
//#define BME_MISO D6
//#define BME_MOSI D2
//#define BME_CS D5

#define SEALEVELPRESSURE_HPA (1013.25)

BME680_Class BME680; ///< Create an instance of the BME680 class
// Adafruit_BME680 bme; // I2C
SdsDustSensor sds(rxPin, txPin);

int co2pwmPin = D5;
volatile unsigned long co2timehigh, co2timelow, co2prevhigh, co2prevlow, co2ppm, prevco2ppm, publishco2ppm = 0;
volatile byte donerising, donefalling = 0;

ICACHE_RAM_ATTR void co2rising()
{
  attachInterrupt(digitalPinToInterrupt(co2pwmPin), co2falling, FALLING);

  co2prevhigh = millis();
  co2timelow = co2prevhigh - co2prevlow;
}

ICACHE_RAM_ATTR void co2falling()
{
  attachInterrupt(digitalPinToInterrupt(co2pwmPin), co2rising, RISING);

  co2prevlow = millis();
  co2timehigh = co2prevlow - co2prevhigh;

  prevco2ppm = co2ppm;
  co2ppm = 5000 * (co2timehigh - 2) / (co2timehigh + co2timelow - 4);

  if (prevco2ppm > 0 && co2timehigh > 0 && co2timelow > 0)
  {
    publishco2ppm = co2ppm;
  }
}

void setup()
{
  delay(500);
  Serial.begin(9600);

  // pinMode(ONE_WIRE_BUS, INPUT_PULLUP);

  sds.begin();
  Serial.println(sds.setQueryReportingMode().toString()); // ensures sensor is in 'query' reporting mode

  /*
  bme.begin();
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  */

  while (!BME680.begin(I2C_STANDARD_MODE))
  { // Start BME680 using I2C, use first device found
    Serial.print(F("-  Unable to find BME680. Trying again in 5 seconds.\n"));
    delay(5000);
  }                                                        // of loop until device is located
  BME680.setOversampling(TemperatureSensor, Oversample16); // Use enumerated type values
  BME680.setOversampling(HumiditySensor, Oversample16);    // Use enumerated type values
  BME680.setOversampling(PressureSensor, Oversample16);    // Use enumerated type values
  BME680.setIIRFilter(IIR4);                               // Use enumerated type values
  BME680.setGas(320, 150);                                 // 320C for 150 milliseconds

  pinMode(co2pwmPin, INPUT);
  digitalWrite(co2pwmPin, HIGH);
  attachInterrupt(digitalPinToInterrupt(co2pwmPin), co2rising, RISING);

  ESP.wdtDisable();        /* Disable the watchdog and wait for more than 8 seconds */
  delay(1000);             /* Done so that the Arduino doesn't keep resetting infinitely in case of wrong configuration */
  ESP.wdtEnable(60 * 1000); /* Enable the watchdog with a timeout of 60 seconds */

  networkSetup();

}

void loop()
{
  reconnect();
/*
  // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
  Serial.print(F("BME680 reading started at "));
  Serial.print(millis());
  Serial.print(F(" and will finish at "));
  Serial.println(endTime);
*/
  Serial.println("Waking up SDS011");
  sds.wakeup();

  // wait at least 30 seconds for SDS sensor to work a little
  Serial.print("Waiting ");
  Serial.print(SDS011_WORK_SECONDS);
  Serial.println(" seconds for sensor to work");
  delayMqtt(1000 * SDS011_WORK_SECONDS);

  PmResult pm = sds.queryPm();
  if (pm.isOk())
  {
    float pm25 = pm.pm25;
    float pm10 = pm.pm10;
    Serial.print("PM2.5 = ");
    Serial.print(pm25);
    Serial.print(", PM10 = ");
    Serial.println(pm10);

    mqttpublish(&mqtt, FEEDPM25, pm25, 2);
    mqttpublish(&mqtt, FEEDPM10, pm10, 2);
  }
  else
  {
    Serial.print("Could not read values from sensor, reason: ");
    Serial.println(pm.statusToString());
  }
  Serial.println("Putting the SDS011 to sleep.");
  WorkingStateResult state = sds.sleep();

  // sensors.begin();
  // sensors.setResolution(12);
  // Serial.print("Requesting temperatures...");
  // sensors.requestTemperatures(); // Send the command to get temperatures
  // Serial.println("DONE");
  // Serial.print("Temperature for the device 1 (index 0) is: ");
  // Serial.println(sensors.getTempCByIndex(0));
  // pinMode(ONE_WIRE_BUS, INPUT_PULLUP);
/*
  if (bme.endReading())
  {
    Serial.print(F("Reading completed at "));
    Serial.println(millis());

    float temperature = bme.temperature + BME680_TEMP_OFFSET;
    Serial.print(F("Temperature = "));
    Serial.print(temperature);
    Serial.println(F(" *C"));
    mqttpublish(&mqtt, FEEDTEMP, temperature, 2);

    float pressure = bme.pressure / 100.0;
    Serial.print(F("Pressure = "));
    Serial.print(pressure);
    Serial.println(F(" hPa"));
    mqttpublish(&mqtt, FEEDPRESSURE, pressure, 2);

    float humidity = bme.humidity;
    Serial.print(F("Humidity = "));
    Serial.print(humidity);
    Serial.println(F(" %"));
    mqttpublish(&mqtt, FEEDHUMID, humidity, 2);

    float gas = bme.gas_resistance / 1000.0;
    Serial.print(F("Gas = "));
    Serial.print(gas);
    Serial.println(F(" KOhms"));
    mqttpublish(&mqtt, FEEDVOC, gas, 2);

    Serial.print(F("Approx. Altitude = "));
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(F(" m"));
  }
  else
  {
    Serial.println(F("Failed to complete reading :("));
  }
*/

  static int32_t temp, humidity, pressure, gas; // BME readings
  uint8_t bmeres = BME680.getSensorData(temp, humidity, pressure, gas, true); // Get readings

  float temperature = ((float)temp / 1000.0) + BME680_TEMP_OFFSET;
  Serial.print(F("Temperature = "));
  Serial.print(temperature);
  Serial.println(F(" *C"));
  mqttpublish(&mqtt, FEEDTEMP, temperature, 2);

  float humid = (float)humidity / 1000.0;
  Serial.print(F("Humidity = "));
  Serial.print(humid);
  Serial.println(F(" %"));
  mqttpublish(&mqtt, FEEDHUMID, humid, 2);

  float voc = (float)gas / 1000.0;
  Serial.print(F("Gas = "));
  Serial.print(voc);
  Serial.println(F(" mOhms"));
  mqttpublish(&mqtt, FEEDVOC, voc, 2);

  if (publishco2ppm > 0)
  {
    long co2val = publishco2ppm;
    publishco2ppm = 0;

    Serial.print(F("CO2 = "));
    Serial.print(co2val);
    Serial.println(F(" PPM"));
    mqttpublish(&mqtt, FEEDCO2, co2val, 2);
  }
  else
  {
    Serial.print(F("CO2 NO co2timelow="));
    Serial.println(co2timelow);
    Serial.print(F("CO2 NO co2timehigh="));
    Serial.println(co2timehigh);
    Serial.print(F("CO2 NO co2ppm="));
    Serial.println(co2ppm);
  }

  Serial.println();

  if (state.isWorking())
  {
    Serial.println("Problem with sleeping the sensor.");
  }
  else
  {
    Serial.print("Sensor is sleeping for ");
    Serial.print(SLEEP_SECONDS);
    Serial.println(" seconds.");

    delayMqtt(1000 * SLEEP_SECONDS);
  }
  Serial.println();
}

void mqttcallback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Got MQTT packet on topic=");
  Serial.print(topic);
  Serial.print(" length=");
  Serial.print(length);
  Serial.print(" with value=");
  if (length == 0)
  {
    Serial.println("ZERO_LENGTH");
    return;
  }

  payload[length] = '\0'; // Null terminator used to terminate the char array
  // String gotval = String((char *)payload);

  if (strcmp((char *)payload, "") == 0)
  {
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

float altitude(const int32_t press, const float seaLevel = 1013.25);
float altitude(const int32_t press, const float seaLevel)
{
  /*!
  @brief     This converts a pressure measurement into a height in meters
  @details   The corrected sea-level pressure can be passed into the function if it is known,
             otherwise the standard atmospheric pressure of 1013.25hPa is used (see
             https://en.wikipedia.org/wiki/Atmospheric_pressure) for details.
  @param[in] press    Pressure reading from BME680
  @param[in] seaLevel Sea-Level pressure in millibars
  @return    floating point altitude in meters.
  */
  static float Altitude;
  Altitude =
      44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903)); // Convert into meters
  return (Altitude);
} // of method altitude()