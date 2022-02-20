#include <OdomMediator.h>

OdomMediator::OdomMediator(MowbotOdometry& mowbotOdometry)
  : mowbotOdometry_(mowbotOdometry)
{
  mowbotOdometry_.setMediator(this);
}

void
OdomMediator::publishOdometry(Odometry odom)
{
  poseX_m_ = odom.poseX_m;
  poseY_m_ = odom.poseY_m;
  heading_rad_ = odom.heading_rad;
  speedX_mps_ = odom.speedX_mps;
  speedY_mps_ = odom.speedY_mps;
  linear_speed_mps_ = odom.linear_speed_mps;
  angular_speed_rps_ = odom.angular_speed_rps;
  odometer_m_ = odom.odometer_m;
  leftSpeed_ = odom.leftSpeed;
  rightSpeed_ = odom.rightSpeed;
  leftEncoderCount_ = odom.leftEncoderCount;
  rightEncoderCount_ = odom.rightEncoderCount;

  if (leftEncoderCount_ != lastLeftEncCnt_ || rightEncoderCount_ != lastRightEncCnt_)
  {
    char odomMsg[200];
    snprintf(odomMsg, 200, "EncoderMediator saw poseX: %0.2f m, poseY: %0.2f m, heading: %0.0f deg, left speed: %0.2f m/s, right speed: %0.2f m/s",
            poseX_m_, poseY_m_, heading_rad_ * (180 / M_PI), leftSpeed_, rightSpeed_);
    mowbotOdometry_.odomLog_.verboseln(odomMsg);
    lastLeftEncCnt_ = leftEncoderCount_;
    lastRightEncCnt_ = rightEncoderCount_;
  }
}