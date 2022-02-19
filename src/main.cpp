#include <Arduino.h>
#include <MowbotOdometry.h>
#include <ArduinoLog.h>
#include <cmath>

// Globals
QueueHandle_t encoderMsgQ;
MowbotOdometry mowbotOdometry(encoderMsgQ);

// Local object instantiations
static TaskHandle_t mowbotOdometryTaskHandle = NULL;

// @brief test whether two floats are approximately equal
bool approximatelyEqual(float a, float b, float epsilon)
{
    return fabs(a - b) <= ( (fabs(a) < fabs(b) ? fabs(b) : fabs(a)) * epsilon);
}

// @brief static function to call run() method
void static startOdometryTask(void* params)
{
  mowbotOdometry.run(params);
}

void setup() {
  bool isok = true;

  // Configure Serial
  Serial.begin(115200);
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

static float poseX = 0.0;
static float poseY = 0.0;
static float heading = 0.0;
static float speedX = 0.0;
static float speedY = 0.0;
static float linearSpeed = 0.0;
static float angularSpeed = 0.0;
static float odometer = 0.0;
static float leftSpeed;
static float rightSpeed;
static int leftEnc = 0;
static int rightEnc = 0;
static int lastLeftEnc = 0;
static int lastRightEnc = 0;

void loop() {
  mowbotOdometry.getEncoders(leftEnc, rightEnc);
  mowbotOdometry.getOdometry(poseX, poseY, heading, speedX, speedY, linearSpeed, angularSpeed, odometer, leftSpeed, rightSpeed);
  if (leftEnc != lastLeftEnc || rightEnc != lastRightEnc)
  {
    char odomMsg[100];
    sprintf(odomMsg, "poseX: %0.2f m, poseY: %0.2f m, heading: %0.0f deg, left speed: %0.2f m/s, right speed: %0.2f m/s",
            poseX, poseY, heading * (180 / M_PI), leftSpeed, rightSpeed);
    Serial.println(odomMsg);
    lastLeftEnc = leftEnc;
    lastRightEnc = rightEnc;
  }
  vTaskDelay(100);
}