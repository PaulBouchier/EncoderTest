#pragma once

#include <Arduino.h>

struct EncoderInterruptMsg {
  bool leftFlag;  // true if this queue entry is for left encoder, false for right
  TickType_t tickCount;   // time when encoder interrupt happened
};

class MowbotOdometry
{
public:
 /*
  * @brief MowbotOdometry ctor
  * @param encoderMsgQ A handle to the queue of encoder timestamps from the ISR
  * @param motorCmdStatus Struct providing current motor direction
  */
  MowbotOdometry(QueueHandle_t& encoderMsgQ);
  bool init();      // initialize MowbotOdometry
  void run(void* params);       // odometry task starts running here
  void getOdometry(int& leftCounts, float& speedL, int& rightCounts, float& speedR);
  void setWheelDirections(bool leftFwd, bool rightFwd);
private:
  const float encoderMetersPerIrq = 0.0075;   // how far mowbot travels each endoder transition
  const float wheelbase_m = 0.444;      // distance between mowbot rear wheels

  float poseX_m_ = 0;       // x location relative to start
  float poseY_m_ = 0;       // y location relative to start
  float heading_rad_ = 0;   // heading relative to start, in ENU coord frame (+ve is CCW)
  float odometer_m_ = 0;    // not needed for anything specific

  QueueHandle_t& encoderMsgQ_;
  int leftEncoderCount_ = 0;     // running count of left encoder counts
  int rightEncoderCount_ = 0;    // running count of right encoder counts
  TickType_t lastLeftTick_ = 0;  // Time when last left tick was observed
  TickType_t lastRightTick_ = 0; // Time when last right tick was observed
  float leftSpeed_ = 0.0;
  float rightSpeed_ = 0.0;
  bool leftFwd_ = true;
  bool rightFwd_ = true;
};