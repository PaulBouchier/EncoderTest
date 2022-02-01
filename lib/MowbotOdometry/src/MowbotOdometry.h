#pragma once

#include <Arduino.h>

struct EncoderInterruptMsg {
  bool leftFlag;  // true if this queue entry is for left encoder, false for right
  TickType_t tickCount;   // time when encoder interrupt happened
};

class MowbotOdometry
{
public:
  MowbotOdometry(QueueHandle_t& encoderMsgQ);
  bool init();      // initialize MowbotOdometry
  void run(void* params);       // odometry task starts running here
  void getOdometry(int& leftCounts, float& speedL, int& rightCounts, float& speedR);
private:
  const float encoderMetersPerTick = 0.0075;

  QueueHandle_t& encoderMsgQ_;
  int leftEncoderCount_ = 0;     // running count of left encoder counts
  int rightEncoderCount_ = 0;    // running count of right encoder counts
  TickType_t lastLeftTick_ = 0;  // Time when last left tick was observed
  TickType_t lastRightTick_ = 0; // Time when last right tick was observed
  float leftSpeed_ = 0.0;
  float rightSpeed_ = 0.0;

};