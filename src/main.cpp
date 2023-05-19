#include "Arduino.h"
#include "GBusHelpers.h"
#include "GBusWifiMesh.h"
#include <EEPROM.h>
#include <WiFi.h>
#include "Tasker.h"
#include <ArduinoJson.h>

#define FWVERSION "1.4"
#define MODULNAME "GBusMeshEsp32EltakoWeatherstation"
#define LogLevel ESP_LOG_NONE

#define SendEveryXTimesValuesToServer 5
#define BUFFER_SIZE 64
#define MSG_SIZE 40
#define DELAY_MS 10 // Delay in ms

MeshApp mesh;
Tasker tasker;

// Prototypes
void meshMessage(String msg, uint8_t SrcMac[6]);
void SentNodeInfo();
void RootNotActiveWatchdog();
void meshConnected();

uint8_t ModulType = 255;

uint8_t ActualCount = 0;

char recv_str[BUFFER_SIZE];

float temperature = 0;
float windspeed = 0;
long sun_south = 0;
int sun_west = 0;
int sun_east = 0;
long daylight = 0;
boolean dawn = false;
uint8_t rain = 0;

int recv_sum = 0;
int checksum = 0;

boolean check_checksum(char *string);
void convert_rcvstr(char *string);
void readSerial();

void setup()
{
  Serial.begin(115200);
  Serial2.begin(19200);

  /**
   * @brief Set the log level for serial port printing.
   */
  esp_log_level_set("*", LogLevel);
  esp_log_level_set(TAG, LogLevel);

  MDF_LOGI("ModuleType: %u\n", ModulType);

  mesh.onMessage(meshMessage);
  mesh.onConnected(meshConnected);
  mesh.start(false);

  tasker.setTimeout(RootNotActiveWatchdog, CheckForRootNodeIntervall);

  pinMode(13, OUTPUT); // Initialize GPIO2 pin as an output
  digitalWrite(13, LOW);
}

void loop()
{
  tasker.loop();
  readSerial();
}

void RootNotActiveWatchdog()
{
  ESP.restart();
}

void SentNodeInfo()
{
  mesh_addr_t bssid;
  esp_err_t err = esp_mesh_get_parent_bssid(&bssid);

  char MsgBuffer[300];
  sprintf(MsgBuffer, "MQTT Info ModulName:%s,SubType:%u,MAC:%s,WifiStrength:%d,Parent:%s,FW:%s", MODULNAME, ModulType, WiFi.macAddress().c_str(), getWifiStrength(3), hextab_to_string(bssid.addr).c_str(), FWVERSION);
  String Msg = String(MsgBuffer);
  mesh.SendMessage(Msg);
}

void meshConnected()
{
  SentNodeInfo();
}

void meshMessage(String msg, uint8_t SrcMac[6])
{
  MDF_LOGD("Rec msg %u: %s", msg.length(), msg.c_str());

  String Type = getValue(msg, ' ', 0);
  String Number = getValue(msg, ' ', 1);
  String Command = getValue(msg, ' ', 2);
  uint8_t NumberInt = Number.toInt();

  if (Type == "Output")
  {
  }
  else if (msg.startsWith("I'm Root!"))
  {
    MDF_LOGI("Gateway hold alive received");
    tasker.cancel(RootNotActiveWatchdog);
    tasker.setTimeout(RootNotActiveWatchdog, CheckForRootNodeIntervall);
  }
  else if (Type == "Config")
  {
    Serial.printf("Config\n");
    String ConfigType = getValue(msg, ' ', 1);
    // Config ModulType 2|4|6
    if (ConfigType == "ModulType")
    {
      String Type = getValue(msg, ' ', 2);
      Serial.printf("New ModulType: %s\n", Type.c_str());
      EEPROM.write(0, (uint8_t)Type.toInt());
      EEPROM.commit();
      ESP.restart();
    }
    else if (ConfigType == "WifiPower")
    {
      String Type = getValue(msg, ' ', 2);
      Serial.printf("WifiPower: %s\n", Type.c_str());
      EEPROM.write(2, (uint8_t)Type.toInt());
      EEPROM.commit();
      ESP.restart();
    }
  }
  else if (Type == "GetNodeInfo")
  {
    SentNodeInfo();
  }
  else if (Type == "Reboot")
  {
    ESP.restart();
  }
}

boolean check_checksum(char *string)
// Verify if calculated checksum equals transferred checksum (return true) or not (return false)
{
  char buffer[5];
  boolean ret_val;
  checksum = 0;
  recv_sum = 0;
  for (int i = 0; i < MSG_SIZE - 5; i++)
    checksum += string[i]; // calculate the checksum
  strncpy(buffer, string + MSG_SIZE - 5, 4);
  buffer[4] = '\0';
  recv_sum = atoi(buffer); // convert transferred checksum to integer

  if (recv_sum == checksum)
  {
    ret_val = true;
    delay(DELAY_MS);
  }
  else
  {
    ret_val = false;
    delay(DELAY_MS);
  }
  return (ret_val);
}

void convert_rcvstr(char *EltakoAnswer)
// Split string into single values
{
  char buffer[6];
  StaticJsonDocument<500> WeatherStationJson;

  if (check_checksum(EltakoAnswer))
  {
    // client.publish("gimpire/WeatherEltako/Debug", "checksum OK");
    strncpy(buffer, EltakoAnswer + 1, 5);
    buffer[5] = '\0';
    temperature = atof(buffer);
    //Serial.println("temperature: " + String(temperature));

    strncpy(buffer, EltakoAnswer + 6, 2);
    buffer[2] = '\0';
    sun_south = atoi(buffer);
    //Serial.println("sun_south: " + String(sun_south));

    strncpy(buffer, EltakoAnswer + 8, 2);
    buffer[2] = '\0';
    sun_west = atoi(buffer);
    //Serial.println("sun_west: " + String(sun_west));

    strncpy(buffer, EltakoAnswer + 10, 2);
    buffer[2] = '\0';
    sun_east = atoi(buffer);
    //Serial.println("sun_east: " + String(sun_east));

    EltakoAnswer[12] == 'J' ? dawn = true : dawn = false;
    //Serial.println("dawn: " + String(dawn));

    strncpy(buffer, EltakoAnswer + 13, 3);
    buffer[3] = '\0';
    daylight = atoi(buffer);
    //Serial.println("daylight: " + String(daylight));

    strncpy(buffer, EltakoAnswer + 16, 4);
    buffer[4] = '\0';
    windspeed = atof(buffer);
    //Serial.println("windspe: " + String(windspeed));

    EltakoAnswer[20] == 'J' ? rain = 1 : rain = 0;
    //Serial.println("rain: " + String(rain));

    long LightSum = (sun_south * 1000) + daylight;
    WeatherStationJson["Temperature"] = String(temperature, 1);
    WeatherStationJson["Light"] = String(LightSum);
    WeatherStationJson["Windspeed"] = String(windspeed, 1);
    WeatherStationJson["Rain"] = String(rain);

    String WeatherStationJsonString;
    serializeJson(WeatherStationJson, WeatherStationJsonString);
    String Msg = "MQTT values " + WeatherStationJsonString;
    mesh.SendMessage(Msg);
    //mesh.SendMessage(EltakoAnswer);

    //client.publish("gimpire/EspWeatherStationEltako/Temperature", String(temperature).c_str());
    //client.publish("gimpire/EspWeatherStationEltako/Light", String((sun_south * 1000) + daylight).c_str());
    //client.publish("gimpire/EspWeatherStationEltako/Windspeed", String(windspeed).c_str());
    //client.publish("gimpire/EspWeatherStationEltako/Rain", String(rain).c_str());
  }
  else
  {
    String Msg = "MQTT Checksum Failed ";
    mesh.SendMessage(Msg);
    mesh.SendMessage(EltakoAnswer);
  }
}

void readSerial()
// Read the weather values from the serial bus
{
  // client.publish("gimpire/WeatherEltako/Debug", "readSerial");
  int index;
  // Serial.println("Serial Buffer: " + Serial.available());
  if (Serial2.available() > 0)
  {
    index = 0;

    while (Serial2.available() > 0)
    {
      char inByte = Serial2.read();
      if (index < BUFFER_SIZE - 1)
      {
        recv_str[index] = inByte;
        index++;
      }
      else
      {
        // readErrorCount++;
        //break;
      }
      delay(20);
    }
    delay(DELAY_MS);
    recv_str[index] = '\0';

    if (ActualCount++ > SendEveryXTimesValuesToServer)
    {
      ActualCount=0;
      convert_rcvstr(recv_str);
    }
  }
}