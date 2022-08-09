#include <OdomMediator.h>

OdomMediator::OdomMediator(MowbotOdometry& mowbotOdometry)
  : mowbotOdometry_(mowbotOdometry)
{
  mowbotOdometry_.setMediator(this);
}

void
OdomMediator::publishOdometry(OdometryMsg odom)
{
  char odomMsg[200];
  snprintf(odomMsg, 200, "EncoderMediator saw poseX: %0.2f m, poseY: %0.2f m, heading: %0.0f deg, odom_heading: %0.0f deg, left speed: %0.2f m/s, right speed: %0.2f m/s",
          odom.poseX_m, odom.poseY_m, odom.heading_rad * (180 / M_PI), odom.odom_heading_rad * (180 / M_PI), odom.leftSpeed, odom.rightSpeed);
  mowbotOdometry_.odomLog_.verboseln(odomMsg);
}
