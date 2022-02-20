#pragma once

#include <MowbotOdometry.h>

class MowbotOdometry;
struct Odometry;

class Mediator
{
public:
  virtual void publishOdometry(Odometry odom) = 0;
};

class OdomMediator : public Mediator
{
public:
  OdomMediator(MowbotOdometry& mowbotOdometry);
  void publishOdometry(Odometry odom);

private:
  MowbotOdometry& mowbotOdometry_;

  float poseX_m_ = 0;           // x location relative to start
  float poseY_m_ = 0;           // y location relative to start
  float heading_rad_ = 0;       // heading relative to start, in ENU coord frame (+ve is CCW)
  float speedX_mps_ = 0;        // speed in X direction
  float speedY_mps_ = 0;        // speed in Y direction
  float linear_speed_mps_ = 0;  // linear speed in m/sec
  float angular_speed_rps_ = 0; // rotational speed in rad/sec
  float odometer_m_ = 0;        // not needed for anything specific
  float leftSpeed_ = 0.0;
  float rightSpeed_ = 0.0;

  int leftEncoderCount_ = 0;     // running count of left encoder counts
  int rightEncoderCount_ = 0;    // running count of right encoder counts

  int lastLeftEncCnt_ = 0;       // last received encoder count
  int lastRightEncCnt_ = 0;      // last received encoder count

};