#pragma once

#include <Arduino.h>
#include <ArduinoLog.h>
#include "../../../src/OdomMediator.h"

class Mediator;

struct Odometry {
  float poseX_m;
  float poseY_m;
  float heading_rad;
  float speedX_mps;
  float speedY_mps;
  float linear_speed_mps;
  float angular_speed_rps;
  float odometer_m;
  float leftSpeed;
  float rightSpeed;

  int leftEncoderCount;
  int rightEncoderCount;
};

class MowbotOdometry
{
public:
  MowbotOdometry();
  bool init(Stream* logStream, int logLevel); // initialize MowbotOdometry, passing a stream pointer for logging
  void run(void* params);       // odometry task starts running here
  void getOdometry(float& poseX, float& poseY, float& heading,  float& speedX, float& speedY, float& linearSpeed,
                  float& angular_speed, float& odometer, float& speedL, float& speedR);
  void getWheelSpeeds(float& speedL, float& speedR);
  void getEncoders(int& leftEnc, int& rightEnc);
  void setWheelDirections(bool leftFwd, bool rightFwd);
  Logging odomLog_;
  void setMediator(Mediator* mediator);

private:
  void populateOdomStruct(Odometry& odometry);

  const float encoderMetersPerIrq = 0.0075;   // how far mowbot travels each endoder transition
  const float wheelbase_m = 0.444;      // distance between mowbot rear wheels
  const int frameTime_ms = 50;

  float poseX_m_ = 0;           // x location relative to start
  float poseY_m_ = 0;           // y location relative to start
  float heading_rad_ = 0;       // heading relative to start, in ENU coord frame (+ve is CCW)
  float speedX_mps_ = 0;        // speed in X direction
  float speedY_mps_ = 0;        // speed in Y direction
  float linear_speed_mps_ = 0;  // linear speed in m/sec
  float angular_speed_rps_ = 0; // rotational speed in rad/sec
  float odometer_m_ = 0;        // not needed for anything specific
  float leftSpeed_ = 0.0;       // left wheel speed m/s
  float rightSpeed_ = 0.0;      // right wheel speed m/s

  int leftEncoderCount_ = 0;     // running count of left encoder counts
  int rightEncoderCount_ = 0;    // running count of right encoder counts

  TickType_t lastLeftTick_ = 0;  // Time when last left tick was observed
  TickType_t lastRightTick_ = 0; // Time when last right tick was observed
  bool leftFwd_ = true;
  bool rightFwd_ = true;

  TaskHandle_t mowbotOdometryTaskHandle_ = NULL;
  Mediator* mediator_ = NULL;
};