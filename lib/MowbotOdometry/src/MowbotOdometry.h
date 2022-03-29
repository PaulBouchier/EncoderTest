#pragma once

#include <Arduino.h>
#include <ArduinoLog.h>
#include <OdometryMsg.h>
#include <Mediator.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

class Mediator;


class MowbotOdometry
{
public:
  MowbotOdometry();
  bool init(int logLevel, Stream* logStream=NULL); // initialize MowbotOdometry, passing a stream pointer for logging
  void run(void* params);       // odometry task starts running here
  void getOdometry(float& poseX, float& poseY, float& heading, float& odom_heading, float& speedX, float& speedY, float& linearSpeed,
                  float& angular_speed, float& odometer, float& speedL, float& speedR);
  void clearOdometry();
  void getWheelSpeeds(float& speedL, float& speedR);
  void getEncoders(int& leftEnc, int& rightEnc);
  void setWheelDirections(bool leftFwd, bool rightFwd);
  Logging odomLog_;
  void setMediator(Mediator* mediator) { mediator_ = mediator; }
  UBaseType_t getStackHighWaterMark() { return uxTaskGetStackHighWaterMark(mowbotOdometryTaskHandle_);}

private:
  void populateOdomStruct(OdometryMsg& odometry);

  const float encoderMetersPerIrq = 0.00714;   // how far mowbot travels each endoder transition
  const float wheelbase_m = 0.444;            // distance between mowbot rear wheels
  const float wheelRadius_m = 0.127;          // drive wheel radius
  const float wheelCircum_m = 2 * M_PI * wheelRadius_m;
  const float irqsPerRot = wheelCircum_m / encoderMetersPerIrq;
  const int frameTime_ms = 50;

  int32_t seq_ = 0;             // OdomMsg sequence #
  float poseX_m_ = 0;           // x location relative to start
  float poseY_m_ = 0;           // y location relative to start
  float heading_rad_ = 0;       // absolute heading relative to East, measured by BNO055, in ENU coord frame (+ve is CCW) REP-103
  float odom_heading_rad_ = 0;  // heading relative to start, computed from odometry, in ENU coord frame (+ve is CCW)
  float speedX_mps_ = 0;        // speed in X direction
  float speedY_mps_ = 0;        // speed in Y direction
  float linear_speed_mps_ = 0;  // linear speed in m/sec
  float angular_speed_rps_ = 0; // rotational speed in rad/sec
  float odometer_m_ = 0;        // not needed for anything specific
  float leftSpeed_ = 0.0;       // left wheel speed m/s
  float rightSpeed_ = 0.0;      // right wheel speed m/s

  int leftEncoderCount_ = 0;    // running count of left encoder counts
  float leftWheelAngle_rad_ = 0; // wheel angle in radians
  int rightEncoderCount_ = 0;   // running count of right encoder counts
  float rightWheelAngle_rad_ = 0;

  TickType_t lastLeftTick_ = 0;  // Time when last left tick was observed
  TickType_t lastRightTick_ = 0; // Time when last right tick was observed
  bool leftFwd_ = true;
  bool rightFwd_ = true;

  Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
  uint16_t imuCalStatus_ = 0;

  TaskHandle_t mowbotOdometryTaskHandle_ = NULL;
  Mediator* mediator_ = NULL;
  SemaphoreHandle_t wheelDirMutex_;
};