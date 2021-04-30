#include "bluetooth.h"
#include "led.h"
#include "comms.h"
#include "ota.h"

#define roomclock_pin 32

void setup()
{
  Serial.begin(115200);
  BTsetup();
  ledsetup();
  wifiSetup();
  webServSetup();
  otaSetup();

  pinMode(roomclock_pin, OUTPUT);
  digitalWrite(roomclock_pin, HIGH);
}

void loop()
{
  ArduinoOTA.handle();
  ledloop();

  while (SerialBT.available())
  {
    String btString = SerialBT.readString();
    btString.trim();
    SerialBT.print("Received:");
    SerialBT.println(btString);

    if(btString == "help" || "Help")
    {
      SerialBT.println("led=0/1");
      SerialBT.println("clock=0/1");
      SerialBT.println("restart");
      SerialBT.println("status");
    }

    if(btString == "clock=1")
    {
      digitalWrite(roomclock_pin, HIGH);
    }
    else if(btString == "clock=0")
    {
      digitalWrite(roomclock_pin, LOW);
    }
    
    if(btString == "led=1")
    {
      FastLED.setBrightness(255);
    }
    else if(btString =="led=0")
    {
      FastLED.setBrightness(0);
    }
    else if(btString == "status")
    {
      SerialBT.print("WiFi Status: ");
      SerialBT.println(WiFi.status());
      SerialBT.print("IP: ");
      SerialBT.println(WiFi.localIP());
    }
    else if(btString == "restart")
    {
      ESP.restart();
    }
  }
}
