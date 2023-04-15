#pragma once

#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_task_wdt.h>

// mqtt.biz.mk / 95.216.221.54
#define WLAN_SSID "sensair"
#define WLAN_PASS "******"
#define MQTT_ID "sensairdevice1"
#define MQTT_SERVER "******"
#define MQTT_SERVERPORT 1883 // use 8883 for SSL
#define MQTT_USERNAME "sensairdevice1"
#define MQTT_KEY "******"

#define MQTT_WILL_TOPIC "microstatus/sensairdevice1"
#define MQTT_WILL_MSG "offline"
#define MQTT_BIRTH_MSG "online"

#define MQTT_BUF_SIZE 1500

// Callback function header
void mqttcallback(char *topic, byte *payload, unsigned int length);

static WiFiClient network;
PubSubClient mqtt(MQTT_SERVER, MQTT_SERVERPORT, mqttcallback, network);

#define FEEDTEMP "sensairdevice1/sensor/temp"
#define FEEDHUMID "sensairdevice1/sensor/humid"
#define FEEDPRESSURE "sensairdevice1/sensor/pressure"
#define FEEDVOC "sensairdevice1/sensor/voc"
#define FEEDLUX "sensairdevice1/sensor/lux"
#define FEEDPM25 "sensairdevice1/sensor/pm25"
#define FEEDPM10 "sensairdevice1/sensor/pm10"
#define FEEDCO2 "sensairdevice1/sensor/co2"
#define FEEDTEMP2 "sensairdevice1/sensor/temp2"
#define FEEDHUMID2 "sensairdevice1/sensor/humid2"
#define FEEDTEMP3 "sensairdevice1/sensor/temp3"

void mqttSubscribe()
{
  // mqtt.subscribe(SUBJSONCONF);
}

#define SDSelect 4
#define EthernetSelect 10

void reconnectWifi()
{
  WiFi.mode(WIFI_STA);
  WiFi.hostname("ESP_" MQTT_ID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  unsigned long starttime = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    if (millis() - starttime > 300 * 1000)
    {
      Serial.println("ERROR: Can't connect more than 5 minutes, restarting ESP!");
      ESP.restart(); // or ESP.reset()
    }
  }
  Serial.print("Connected to WiFi, IP: ");
  Serial.println(WiFi.localIP().toString().c_str());
}

void networkSetup()
{
  // WE MUST disable the SD Card interface on the ETH shield, by setting the CS pin 4 to HIGH
  pinMode(SDSelect, OUTPUT);
  digitalWrite(SDSelect, HIGH);

  reconnectWifi();
}

void delayMqtt(int waittime)
{
  esp_task_wdt_reset(); /* Reset the watchdog */
  if (mqtt.connected())
  {
    unsigned long start = millis();
    do
    {
      mqtt.loop();
      delayMicroseconds(1);
    } while (millis() - start < waittime);
  }
  else
  {
    delay(waittime);
  }
}

bool reconnect()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    network.stop();
    delay(1000);
    reconnectWifi();
  }

  if (!mqtt.connected())
  {
    mqtt.setKeepAlive(55); // needs be made before connecting
    int ret;
    esp_task_wdt_reset(); /* Reset the watchdog */
    if ((ret = mqtt.connect(MQTT_ID, MQTT_USERNAME, MQTT_KEY, MQTT_WILL_TOPIC, 1, true, MQTT_WILL_MSG)) != 1)
    {
      Serial.print("MQTT State: ");
      Serial.println(mqtt.state());
      mqtt.disconnect();
    }
    else
    {
      mqtt.setBufferSize(MQTT_BUF_SIZE);
      mqtt.publish(MQTT_WILL_TOPIC, MQTT_BIRTH_MSG, true);
      Serial.println(F("MQTT Connected..."));
      mqttSubscribe();
    }
    esp_task_wdt_reset(); /* Reset the watchdog */
  }
  return mqtt.connected();
}

bool mqttpublish(PubSubClient *mqtt, const char *topic, int32_t i)
{
  char payload[12];
  ltoa(i, payload, 10);
  boolean result = mqtt->publish(topic, payload);
  if (result)
    Serial.print("Successfully wrote ");
  else
    Serial.print("Failed to write ");
  Serial.print(payload);
  Serial.print(" to ");
  Serial.println(topic);
  return result;
}

bool mqttpublish(PubSubClient *mqtt, const char *topic, uint32_t i)
{
  char payload[11];
  ultoa(i, payload, 10);
  boolean result = mqtt->publish(topic, payload);
  if (result)
    Serial.print("Successfully wrote ");
  else
    Serial.print("Failed to write ");
  Serial.print(payload);
  Serial.print(" to ");
  Serial.println(topic);
  return result;
}

bool mqttpublish(PubSubClient *mqtt, const char *topic, double f, uint8_t precision)
{
  char payload[41]; // Need to technically hold float max, 39 digits and minus
                    // sign.
  dtostrf(f, 0, precision, payload);
  boolean result = mqtt->publish(topic, payload);
  if (result)
    Serial.print("Successfully wrote ");
  else
    Serial.print("Failed to write ");
  Serial.print(payload);
  Serial.print(" to ");
  Serial.println(topic);
  return result;
}

bool mqttpublish(PubSubClient *mqtt, const char *topic, const char *payload)
{
  boolean result = mqtt->publish(topic, payload);
  if (result)
    Serial.print("Successfully wrote ");
  else
    Serial.print("Failed to write ");
  Serial.print(payload);
  Serial.print(" to ");
  Serial.println(topic);
  return result;
}
