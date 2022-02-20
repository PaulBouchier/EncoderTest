#include <Arduino.h>
#include <MowbotOdometry.h>
#include <OdomMediator.h>
#include <cmath>

// Globals
MowbotOdometry mowbotOdometry;
OdomMediator odomMediator(mowbotOdometry);

void setup() {
  bool isok = true;
  Serial.begin(115200);
  delay(3000);  // Wait a moment to start (so we don't miss Serial output)

  isok = mowbotOdometry.init(&Serial, LOG_LEVEL_VERBOSE);
  delay(1000);      // wait for startup messages from mowbotOdometry object to get printed
  if (isok)
    mowbotOdometry.odomLog_.infoln("Successfully initialized mowbotOdometry");
}

void loop() {
  vTaskDelay(1000);
}