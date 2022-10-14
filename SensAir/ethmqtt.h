#pragma once

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// mqtt.biz.mk / 95.216.221.54
#define WLAN_SSID "********"
#define WLAN_PASS "********"
#define MQTT_ID "sensair"
#define MQTT_SERVER "256.1.0.1"
#define MQTT_SERVERPORT 1883 // use 8883 for SSL
#define MQTT_USERNAME "********"
#define MQTT_KEY "********"

#define MQTT_WILL_TOPIC "microstatus/sensair"
#define MQTT_WILL_MSG "offline"
#define MQTT_BIRTH_MSG "online"

#define MQTT_BUF_SIZE 1500

// Callback function header
void mqttcallback(char *topic, byte *payload, unsigned int length);

static WiFiClient network;
PubSubClient mqtt(MQTT_SERVER, MQTT_SERVERPORT, mqttcallback, network);

#define FEEDTEMP "sensair/sensor/temp"
#define FEEDHUMID "sensair/sensor/humid"
#define FEEDPRESSURE "sensair/sensor/pressure"
#define FEEDVOC "sensair/sensor/voc"
#define FEEDPM25 "sensair/sensor/pm25"
#define FEEDPM10 "sensair/sensor/pm10"
#define FEEDCO2 "sensair/sensor/co2"

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
    ESP.wdtFeed(); /* Reset the watchdog */
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
    ESP.wdtFeed(); /* Reset the watchdog */
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
