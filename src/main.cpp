#include <Arduino.h>
#include <MowbotOdometry.h>
#include <Mediator.h>
#include <OdomMediator.h>
#include <cmath>

// Globals
MowbotOdometry mowbotOdometry;
OdomMediator odomMediator(mowbotOdometry);

void setup() {
  bool isok = true;
  Serial.begin(115200);
  delay(3000);  // Wait a moment to start (so we don't miss Serial output)

  isok = mowbotOdometry.init(LOG_LEVEL_INFO, &Serial);
  delay(1000);      // wait for startup messages from mowbotOdometry object to get printed
  if (isok)
    mowbotOdometry.odomLog_.infoln("Successfully initialized mowbotOdometry");
}

void loop() {
  int leftEncoderCount_ = 0;    // running count of left encoder counts
  int rightEncoderCount_ = 0;   // running count of right encoder counts

  mowbotOdometry.getEncoders(leftEncoderCount_, rightEncoderCount_);
  Serial.print("L: ");
  Serial.print(leftEncoderCount_);
  Serial.print(" R: ");
  Serial.print(rightEncoderCount_);
  Serial.println("");
  vTaskDelay(1000);
}