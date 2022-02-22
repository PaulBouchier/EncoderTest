#include <OdomMediator.h>

OdomMediator::OdomMediator(MowbotOdometry& mowbotOdometry)
  : mowbotOdometry_(mowbotOdometry)
{
  mowbotOdometry_.setMediator(this);
}

void
OdomMediator::publishOdometry(OdometryMsg odom)
{
  if (odom.leftEncoderCount != lastLeftEncCnt_ || odom.rightEncoderCount != lastRightEncCnt_)
  {
    char odomMsg[200];
    snprintf(odomMsg, 200, "EncoderMediator saw poseX: %0.2f m, poseY: %0.2f m, heading: %0.0f deg, left speed: %0.2f m/s, right speed: %0.2f m/s",
            odom.poseX_m, odom.poseY_m, odom.heading_rad * (180 / M_PI), odom.leftSpeed, odom.rightSpeed);
    mowbotOdometry_.odomLog_.verboseln(odomMsg);
    lastLeftEncCnt_ = odom.leftEncoderCount;
    lastRightEncCnt_ = odom.rightEncoderCount;
  }
}
