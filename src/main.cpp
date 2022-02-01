#include <Arduino.h>
#include <MowbotOdometry.h>
#include <ArduinoLog.h>

// Globals
QueueHandle_t encoderMsgQ;
MowbotOdometry mowbotOdometry(encoderMsgQ);
static TaskHandle_t mowbotOdometryTaskHandle = NULL;

// @brief static function to call run() method
void static startOdometryTask(void* params)
{
  mowbotOdometry.run(params);
}

void setup() {
  bool isok = true;

  // Configure Serial
  Serial.begin(9600);
  // Wait a moment to start (so we don't miss Serial output)
  vTaskDelay(4000);

  Log.begin(LOG_LEVEL_INFO, &Serial);
  Log.infoln("--- Encoder ISR Test ---");

  isok = mowbotOdometry.init();
  if (!isok)
  {
    while(true)   // hang here forever if initialization fails
    {
      Log.fatalln("MowbotOdometry init() failed");
      vTaskDelay(10000);
    }
  }

  // create MowbotOdometry task
  BaseType_t rv = xTaskCreate(
                    startOdometryTask,
                    "MowbotOdometry Task",
                    2048,
                    NULL,
                    1,
                    &mowbotOdometryTaskHandle);
  if (rv != pdTRUE)
  {
    while(true)   // hang here forever if task creation failed
    {
      Log.fatalln("Failed to create MowbotOdometry task");
      vTaskDelay(10000);   // periodically print the failed message
    }
  }
}

void loop() {
  int leftEncoderCounts;
  int rightEncoderCounts;
  static int lastLeftEncoderCounts = 0;
  static int lastRightEncoderCounts = 0;
  float leftSpeed;
  float rightSpeed;

  mowbotOdometry.getOdometry(leftEncoderCounts, leftSpeed, rightEncoderCounts, rightSpeed);
  if (leftEncoderCounts != lastLeftEncoderCounts || rightEncoderCounts != lastRightEncoderCounts)
  {
    Serial.print("left encoder counts: ");
    Serial.print(leftEncoderCounts);
    Serial.print(" left speed: ");
    Serial.print(leftSpeed);
    Serial.print(" right encoder counts: ");
    Serial.print(rightEncoderCounts);
    Serial.print(" right speed: ");
    Serial.print(rightSpeed);
    Serial.println();
    lastLeftEncoderCounts = leftEncoderCounts;
    lastRightEncoderCounts = rightEncoderCounts;
  }
  vTaskDelay(100);
}