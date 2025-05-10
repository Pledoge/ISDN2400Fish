#define BLYNK_PRINT Serial

#define BLYNK_TEMPLATE_ID "TMPL6uM7CQ6aR"
#define BLYNK_TEMPLATE_NAME "isdn2400template1"
#define BLYNK_AUTH_TOKEN "5wuyPwHG4Nv_LFcgcNk_DKUyNMmsxepY"

#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include <WiFiClient.h>
char auth[] = BLYNK_AUTH_TOKEN;

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Redmi Humor"; // Input the name of the WIFI here (You can use
                             // your phone's hotspot)
char pass[] = "233233233";   // Input the Passwork of the WIFI here

int Speed_Pin;
// Motor Driver Pins
int motorSpeed = 10;
bool stateIN1 = 0;
bool stateIN2 = 0;

const int ServoHorizontal = 14;  // Pin for small servo
const uint8_t ServoVertical = 5; // Pin for big servo

bool rotateServo0 = false;
bool rotateServo1 = false;

const int switchPin = 33; // Pin for the swin tch

static int pin_state = 0;

static bool is_stop = false;

TaskHandle_t ServoTaskHandle = NULL;
TaskHandle_t BlynkTaskHandle = NULL;
TaskHandle_t WatchdogTaskHandle = NULL;

static int angle = 0;
static int upper_limit = 114;
static int lower_limit = 54;

// Read the pin state of switch
void ReadPin(int &pin_s) {
  static long lastUpdate_ = 0;
  if (millis() - lastUpdate_ >= 500) {
    lastUpdate_ = millis();
    pin_s = analogRead(switchPin);
  }
}

// Set the servo angle
void setServoAngle(int angle, int channel) {
  if (angle <= lower_limit) {
    angle = lower_limit;
  } else if (angle >= upper_limit) {
    angle = upper_limit;
  }
  long pulseWidth = map(angle, 0, 180, 32, 6); // Map the angle to pulse width
  ledcWrite((uint8_t)channel,
            (uint32_t)pulseWidth); // Write the pulse width to the servo
}

// Keep rotating the servo
void RotateServo(int channel) {
  static unsigned long lastUpdate = 0;

  static bool increasing = true;

  if (fabs(millis() - lastUpdate) >= 50) {
    lastUpdate = millis();
    if (increasing) {
      angle += motorSpeed;
      if (angle >= upper_limit) {
        angle = upper_limit;
        if (angle >= 114) {
          angle = 114;
        }
        increasing = false;
      }
    } else {
      angle -= motorSpeed;
      if (angle <= lower_limit) {
        angle = lower_limit;
        if (angle <= 54) {
          angle = 54;
        }
        increasing = true;
      }
    }
    setServoAngle(angle, channel);
  }
}

// Check if the parameter is valid
bool ParamValidCheck(int param) {
  if (param >= 0 && param <= 300) {
    return false;
  }
  return true;
}

unsigned long last_Update = 0;

BLYNK_WRITE(V0) //Blink. Check whether wifi is working or not
{
  if (millis() - last_Update < 500) {
    return;
  }
  last_Update = millis();
  if (ParamValidCheck(param.asInt())) {
    vTaskDelay(10);
    return;
  }
  int pinValue1 = param.asInt();
  rotateServo0 = false;
  for (int i = 0; i < 3; i++) {
    digitalWrite(2, HIGH);
    vTaskDelay(200);
    digitalWrite(2, LOW);
    vTaskDelay(200);
  }
}

BLYNK_WRITE(V1) // Keep rotating
{
  last_Update = millis();
  if (ParamValidCheck(param.asInt())) {
    return;
  }
  int pinValue2 = param.asInt();
  vTaskDelay(10);
  if (pinValue2 == HIGH) {
    rotateServo1 = true;
  } else {
    rotateServo1 = false;
  }
}

BLYNK_WRITE(V3) // Speed
{
  if (millis() - last_Update < 500) {
    return;
  }
  last_Update = millis();
  if (ParamValidCheck(param.asInt())) {
    return;
  }
  int Motor_Speed = param.asInt();
  vTaskDelay(10);
  motorSpeed = Motor_Speed;
}

BLYNK_WRITE(V2) // STOP
{
  if (millis() - last_Update < 500) {
    return;
  }
  last_Update = millis();
  if (ParamValidCheck(param.asInt())) {
    return;
  }
  int pinValue3 = param.asInt();
  vTaskDelay(10);
  if (pinValue3 == HIGH) {
    rotateServo0 = false;
    rotateServo1 = false;
    setServoAngle(90, 0);
    setServoAngle(angle, 1);
  }
}

BLYNK_WRITE(V4) // Set upper angle limit of servo
{
  if (millis() - last_Update < 500) {
    return;
  }
  last_Update = millis();
  if (ParamValidCheck(param.asInt())) {
    return;
  }
  upper_limit = param.asInt();
}
BLYNK_WRITE(V5) // Set the angle of servo
{
  if (millis() - last_Update < 500) {
    return;
  }
  last_Update = millis();
  if (ParamValidCheck(param.asInt())) {
    return;
  }
  int pinValue5 = param.asInt();
  angle = pinValue5;
  setServoAngle(pinValue5, 1);
  rotateServo1 = false;
}

BLYNK_WRITE(V6) { // Set lower angle limit of servo
  if (millis() - last_Update < 500) {
    return;
  }
  last_Update = millis();
  if (ParamValidCheck(param.asInt())) {
    return;
  }
  lower_limit = param.asInt();
}

//Watchdog task
void WatchdogTask(void *pvParameters) {
  while (1) {
    if (ESP.getFreeHeap() < 10000) {
      esp_task_wdt_reset();
      vTaskDelay(1000);
      ESP.restart();
    }
    vTaskDelay(200);
    esp_task_wdt_reset();
  }
}
SemaphoreHandle_t blynkMutex;
void BlynkTask(void *pvParameters) {
  if (Blynk.connected()) {
    Blynk.virtualWrite(V4, 114);
    Blynk.virtualWrite(V6, 54);
  }
  while (1) {
    if (xSemaphoreTake(blynkMutex, portMAX_DELAY)) {
      Blynk.run();
      xSemaphoreGive(blynkMutex);
    }

    esp_task_wdt_reset();
    vTaskDelay(1);
  }
}

static int servoTaskStartTime = 0;
static int taskTimeout = 5;

void ServoTask(void *pvParameters) {
  while (1) {
    if (xSemaphoreTake(blynkMutex, portMAX_DELAY)) {
      if (rotateServo1) {
        RotateServo(1);
      }
      xSemaphoreGive(blynkMutex);
    }
    // if (rotateServo0) {
    //   setServoAngle(105, 0);
    //   vTaskDelay(100);
    //   setServoAngle(90, 0);
    //   rotateServo0 = false;
    //   Blynk.virtualWrite(V0, 0);
    // }
    // ReadPin(pin_state);
    // if (pin_state < 4090 && !rotateServo0) {
    //   setServoAngle(90, 0);
    //   rotateServo0 = true;
    //   Blynk.virtualWrite(V0, 1);
    // }
    esp_task_wdt_reset(); // Reset the watchdog timer
    vTaskDelay(1);
  }
}

void setup() {
  Serial.begin(115200);
  blynkMutex = xSemaphoreCreateMutex();
  esp_task_wdt_init(5, true);
  // Motor
  pinMode(2, OUTPUT);

  ledcSetup((uint8_t)1, (uint32_t)50, (uint8_t)8);
  ledcAttachPin(ServoVertical, 1);
  ledcWrite((uint8_t)1, (uint32_t)0); // Set the initial duty cycle to 0
  for (int i = 0; i < 3; i++) {
    digitalWrite(2, HIGH);
    vTaskDelay(200);
    digitalWrite(2, LOW);
    vTaskDelay(200);
  }
  // WiFi.begin(ssid, pass);
  Blynk.begin(auth, ssid, pass);
  xTaskCreatePinnedToCore(BlynkTask, "BlynkTask", 50000, NULL, 2,
                          &BlynkTaskHandle,
                          0); // Create Blynk task
  xTaskCreatePinnedToCore(ServoTask, "ServoTask", 15000, NULL, 1,
                          &ServoTaskHandle,
                          1); // Create Servo task
  xTaskCreatePinnedToCore(WatchdogTask, "WatchdogTask", 10000, NULL, 3,
                          &WatchdogTaskHandle,
                          1);           // Create Watchdog task
  esp_task_wdt_add(WatchdogTaskHandle); // Add the task to the watchdog
  esp_task_wdt_add(BlynkTaskHandle);    // Add the task to the watchdog
  esp_task_wdt_add(ServoTaskHandle);    // Add the task to the watchdog
}

void loop() {}