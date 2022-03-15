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
  void clearOdometry() {}
  void setWheelDirections(bool leftFwd, bool rightFwd) {}
  void sendLogMsg(char* logMsg, int length) {}
  void setDrive(int32_t seq, float linear_vel, float angular_vel) {}
  void setLogLvl(int32_t pilinkLogLevel, int32_t rl500LogLevel, int32_t odomLogLevel) {}
  void publishPlatformData(PlatformDataMsg platformData) {}
  void setRL500BITMode() {}

private:
  MowbotOdometry& mowbotOdometry_;

  // track last received encoder counts so we know when odometry has changed
  int lastLeftEncCnt_ = 0;
  int lastRightEncCnt_ = 0;

};