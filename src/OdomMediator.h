#pragma once

#include <MowbotOdometry.h>

class MowbotOdometry;
struct Odometry;

class OdomMediator : public Mediator
{
public:
  OdomMediator(MowbotOdometry& mowbotOdometry);
  void publishOdometry(OdometryMsg odom);

  // dummy defs to staisfy Mediator
  void setWheelDirections(bool leftFwd, bool rightFwd) {}
  void sendLogMsg(char* logMsg, int length) {}
  void setDrive(int8_t leftDrivePct, int8_t rightDrivePct) {}

private:
  MowbotOdometry& mowbotOdometry_;

  // track last received encoder counts so we know when odometry has changed
  int lastLeftEncCnt_ = 0;
  int lastRightEncCnt_ = 0;

};