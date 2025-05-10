#include "Arduino.h"
#define BLYNK_PRINT Serial

#define BLYNK_TEMPLATE_ID                                                      \
  "TMPL66wnO-azI" // Template ID in Developer Zone -> Template -> Firmware
                  // Configuration
#define BLYNK_TEMPLATE_NAME                                                    \
  "reboot" // Template Name in Developer Zone -> Template -> Firmware
           // Configuration
#define BLYNK_AUTH_TOKEN                                                       \
  "28an4NkOK0gYDUQmlDPcMDYwrUDr1u8e" // Authorization Token in Devices -> Device

#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include <WiFiClient.h>
const int rebootPin = 14;

char auth[] = BLYNK_AUTH_TOKEN;

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Redmi Humor"; // Input the name of the WIFI here (You can use
                             // your phone's hotspot)
char pass[] = "233233233";   // Input the Passwork of the WIFI here

void setup() {
  Serial.begin(115200);
  pinMode(rebootPin, OUTPUT);
  digitalWrite(rebootPin, HIGH);
  Blynk.begin(auth, ssid, pass);
}
void loop() {
  Blynk.run();
  digitalWrite(rebootPin, HIGH);
  delay(10);
}

BLYNK_WRITE(V0) // Forward
{
  int pinValue1 = param.asInt();
  if (pinValue1 == 1) {
    digitalWrite(rebootPin, LOW);
    delay(1000);
  } else {
    digitalWrite(rebootPin, HIGH);
  }
}
