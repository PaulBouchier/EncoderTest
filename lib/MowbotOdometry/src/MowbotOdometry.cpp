#include <MowbotOdometry.h>
#include <cmath>
#include <CircularBuffer.h>
#include <LogStream.h>

extern MowbotOdometry mowbotOdometry;

// Local object instantiations

// @brief static function to call run() method
void static startOdometryTask(void* params) { mowbotOdometry.run(params); }

/*
 * \brief encoder message queue length
 * \details At full speed each wheel will see about 80 transitions (80 interrupts)
 * each resulting in an encInterruptMsg. This queue needs to be big enough to handle
 * them until emptied (which should happen each 50ms frame). Allow 2x capacity margin
 * in case base encoder task is late to run in a subsequent frame. I.e. max number of
 * items should be:
 * 2 (margin) * (80 (counts/sec) / 20 (frames/sec)) (max 8 interrupts/frame)
 */
// Instantiate circular buffers for left & right enconder timestamps
static const size_t bufferSize = 10;
static TickType_t leftEncBuf[bufferSize];
static CircularBuffer_<TickType_t> leftEncCircBuf(leftEncBuf, bufferSize);
static TickType_t rightEncBuf[bufferSize];
static CircularBuffer_<TickType_t> rightEncCircBuf(rightEncBuf, bufferSize);

// Time measurement support
TickType_t startTime;
TickType_t endTime;

// Prefix print function for logs from this object
static void printPrefix(Print* logOutput, int logLevel)
{
  switch(logLevel)
  {
    default:
    case 0: logOutput->print("S: Odom: "); break;
    case 1: logOutput->print("F: Odom: "); break;
    case 2: logOutput->print("E: Odom: "); break;
    case 3: logOutput->print("W: Odom: "); break;
    case 4: logOutput->print("I: Odom: "); break;
    case 5: logOutput->print("T: Odom: "); break;
    case 6: logOutput->print("V: Odom: "); break;
  }
}

// encoder ISRs - global names for now, using global Q name
void IRAM_ATTR leftEncChange()
{
  TickType_t encTimestamp = xTaskGetTickCountFromISR();
  leftEncCircBuf.put(encTimestamp);
}

void IRAM_ATTR rightEncChange()
{
  TickType_t encTimestamp = xTaskGetTickCountFromISR();
  rightEncCircBuf.put(encTimestamp);
}

MowbotOdometry::MowbotOdometry()
{
}

bool
MowbotOdometry::init(int logLevel, Stream* stream_p)
{
  bool isok = true;

  // Start logger
  if (stream_p == NULL)
  {  
    // No logStream provided, use logStream
    LogStream* logStream_p = new LogStream();
    logStream_p->setMediator(mediator_);
    odomLog_.begin(logLevel, logStream_p);
  }
  else
  {
    odomLog_.begin(logLevel, stream_p);
  }
  odomLog_.setPrefix(printPrefix);
  odomLog_.setShowLevel(false);

  odomLog_.infoln("MowbotOdometry::init()");

  // configure encoder input pins & attach interrupt handlers
  // FIXME: There's a discrepancy between the drawio wiring diatram & pins 14/15 w.rt. left/right.
  // RESOLVE IT
  const int interruptPinLeft = 15;
  const int interruptPinRight = 14;

  pinMode(interruptPinLeft, INPUT_PULLUP);
  pinMode(interruptPinRight, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinLeft), leftEncChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinRight), rightEncChange, CHANGE);

  wheelDirMutex_ = xSemaphoreCreateMutex();

  // Initialize the IMU
  while(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check connections */
    Serial.println("No BNO055 detected ... Check wiring!");
    odomLog_.fatalln("No BNO055 detected ... Check wiring!");
    delay(2000);    // periodically print the failure message
  }

  // read compass heading from IMU. Heading 0 is north, increases with CW rotation.
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  heading_rad_ = compass2Heading(euler.x());
  odom_heading_rad_ = heading_rad_;

  // create MowbotOdometry task
  BaseType_t rv = xTaskCreate(
                    startOdometryTask,
                    "MowbotOdometry Task",
                    4096,
                    NULL,
                    1,
                    &mowbotOdometryTaskHandle_);

  // hang here forever if task creation failed
  while(rv != pdTRUE)
  {
    Serial.println("Failed to create MowbotOdometry task; stopped");
    odomLog_.fatalln("Failed to create MowbotOdometry task; stopped");
    delay(2000);   // periodically print the failed message
  }

  return isok;
}

void
MowbotOdometry::run(void* params)
{
  TickType_t frameStartTime;

  odomLog_.infoln("Running MowbotOdometryTask");

  // frame time keeper
  TickType_t lastFrameTime = xTaskGetTickCount();

  // loop forever in the task, updating the global odometry struct
  while (true)
  {
    frameStartTime = xTaskGetTickCount();

    int deltaLCounts = 0;
    int deltaLTicks = 0;
    int deltaRCounts = 0;
    int deltaRTicks = 0;
    int msgCount = 0;
  
    // Read encoder messages while there are any enqueued
    TickType_t encLTime;
    TickType_t encRTime;

    while (!leftEncCircBuf.empty())
    {
      ++msgCount;
      leftEncCircBuf.get(encLTime);
      ++deltaLCounts;
      deltaLTicks = encLTime - lastLeftTick_;
      lastLeftTick_ = encLTime;
    }

    while (!rightEncCircBuf.empty())
    {
      ++msgCount;
      rightEncCircBuf.get(encRTime);
      ++deltaRCounts;
      deltaRTicks = encRTime - lastRightTick_;
      lastRightTick_ = encRTime;
    }

    // Use motor direction from RL500CmdTask to set encoders fwd/backwd
    if (!leftFwd_)
      deltaLCounts = -deltaLCounts;
  
    if (!rightFwd_)
      deltaRCounts = -deltaRCounts;
  
    leftEncoderCount_ += deltaLCounts;
    rightEncoderCount_ += deltaRCounts;
    leftWheelAngle_rad_ = fmod(leftEncoderCount_, irqsPerRot) * TWO_PI;
    rightWheelAngle_rad_ = fmod(rightEncoderCount_, irqsPerRot) * TWO_PI;

    float deltaL_m = deltaLCounts * encoderMetersPerIrq;
    float deltaR_m = deltaRCounts * encoderMetersPerIrq;
    float deltaD_m = (deltaL_m + deltaR_m) / 2.0;   // distance traveled this frame
    odometer_m_ += deltaD_m;
    float deltaHeading = (deltaR_m - deltaL_m) / wheelbase_m;  // radians turned this frame
    angular_speed_rps_ = deltaHeading * (1000 / frameTime_ms);

    odom_heading_rad_ += deltaHeading;
    odom_heading_rad_ -= (float)((int)(odom_heading_rad_ / (2 * M_PI))) * 2 * M_PI;  // clip to +/- 2 * pi
  
    poseX_m_ += deltaD_m * cos(odom_heading_rad_);
    poseY_m_ += deltaD_m * sin(odom_heading_rad_);

    // compute speed based on most recent deltaTicks if any counts received, else speed is 0
    if(deltaLCounts != 0)
    {
      leftSpeed_ = encoderMetersPerIrq / static_cast<float>(deltaLTicks) * 1000.0;
      if (!leftFwd_)
        leftSpeed_ = -leftSpeed_;
    }
    else
    {
      leftSpeed_ = 0.0;
    }
    if(deltaRCounts != 0)
    {
      rightSpeed_ = encoderMetersPerIrq / static_cast<float>(deltaRTicks) * 1000.0;
      if (!rightFwd_)
        rightSpeed_ = -rightSpeed_;
    }
    else
    {
      rightSpeed_ = 0.0;
    }

    linear_speed_mps_ = (leftSpeed_ + rightSpeed_) / 2.0;
    speedX_mps_ = linear_speed_mps_ * sin(odom_heading_rad_);
    speedY_mps_ = linear_speed_mps_ * cos(odom_heading_rad_);

    // read compass heading from IMU. Heading 0 is north, increases with CW rotation.
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    heading_rad_ = compass2Heading(euler.x());

    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    imuCalStatus_ = ((uint16_t) system)<<12 | ((uint16_t)gyro)<<8 | accel<<4 | mag;

    // Publish odometry
    OdometryMsg odom;
    populateOdomStruct(odom);
    mediator_->publishOdometry(odom);
    seq_++;     // increment the OdomMsg seqence #
    odomLog_.verboseln("Odom: deltaL_m: %F deltaR_m: %F heading_rad_: %F odom_heading_rad_: %F leftSpeed_: %F rightSpeed_: %F, IMUCal SGAM: %X, euler.x(): %F",
          deltaL_m, deltaR_m, heading_rad_, odom_heading_rad_, leftSpeed_, rightSpeed_, imuCalStatus_, euler.x());
    odomLog_.verboseln("Odom: IMUCal SGAM: %X, euler.x: %F, y: %F, z: %F", imuCalStatus_, euler.x(), euler.y(), euler.z());

    // detect blown frame
    int32_t now = xTaskGetTickCount();
    int32_t overrun = now - lastFrameTime - 50;
    if (overrun > 0)
    {
      odomLog_.errorln("Blew frame by %d ms, last frametime: %d, frame startTime: %d", overrun, lastFrameTime, frameStartTime);
      lastFrameTime = xTaskGetTickCount();
    }

    // delay until start of next 50ms frame
    vTaskDelayUntil(&lastFrameTime, frameTime_ms);
  }

}

void
MowbotOdometry::getOdometry(float& poseX, float& poseY, float& heading, float& odom_heading,
                            float& speedX, float& speedY, float& linearSpeed,
                            float& angular_speed, float& odometer,
                            float& speedL, float& speedR)
{
  poseX = poseX_m_;
  poseY = poseY_m_;
  heading = heading_rad_;
  odom_heading = odom_heading_rad_;
  speedX = speedX_mps_;
  speedY = speedY_mps_;
  linearSpeed = linear_speed_mps_;
  angular_speed = angular_speed_rps_;
  odometer = odometer_m_;
  speedL = leftSpeed_;
  speedR = rightSpeed_;
}

void
MowbotOdometry::clearOdometry()
{
  poseX_m_ = 0;
  poseY_m_ = 0;
  odom_heading_rad_ = heading_rad_;
  speedX_mps_ = 0;
  speedY_mps_ = 0;
  linear_speed_mps_ = 0;
  angular_speed_rps_ = 0;
  odometer_m_ = 0;
  leftSpeed_ = 0;
  rightSpeed_ = 0;
  leftEncoderCount_ = 0;
  rightEncoderCount_ = 0;
  leftWheelAngle_rad_ = 0;
  rightWheelAngle_rad_ = 0;
  imuCalStatus_ = 0;
}

void
MowbotOdometry::populateOdomStruct(OdometryMsg& odom)
{
  odom.seq = seq_;
  odom.espTimestamp = xTaskGetTickCount();
  odom.poseX_m = poseX_m_;
  odom.poseY_m = poseY_m_;
  odom.heading_rad = heading_rad_;
  odom.odom_heading_rad = odom_heading_rad_;
  odom.speedX_mps = speedX_mps_;
  odom.speedY_mps = speedY_mps_;
  odom.linear_speed_mps = linear_speed_mps_;
  odom.angular_speed_rps = angular_speed_rps_;
  odom.odometer_m = odometer_m_;
  odom.leftSpeed = leftSpeed_;
  odom.rightSpeed = rightSpeed_;

  odom.leftEncoderCount = leftEncoderCount_;
  odom.rightEncoderCount = rightEncoderCount_;
  odom.leftWheelAngle_rad = leftWheelAngle_rad_;
  odom.rightWheelAngle_rad = rightWheelAngle_rad_;
  odom.IMUCalStatus = imuCalStatus_;
}

void
MowbotOdometry::getWheelSpeeds(float& speedL, float& speedR)
{
  speedL = leftSpeed_;
  speedR = rightSpeed_;
}

void
MowbotOdometry::getEncoders(int& leftEnc, int& rightEnc)
{
  leftEnc = leftEncoderCount_;
  rightEnc = rightEncoderCount_;
}

void
MowbotOdometry::setWheelDirections(bool leftFwd, bool rightFwd)
{
  // protect data change with mutex at inter-task interface
  if (pdFALSE == xSemaphoreTake(wheelDirMutex_, 5))
  {
    odomLog_.errorln("Failed to acquire wheelDirMutex - setting speed anyway");
  }
  leftFwd_ = leftFwd;
  rightFwd_ = rightFwd;
  if (pdFALSE == xSemaphoreGive(wheelDirMutex_))
  {
    odomLog_.errorln("Failed to acquire wheelDirMutex - setting speed anyway");
  }
}

float
MowbotOdometry::compass2Heading(float x)
{
  const float twoPi = 2 * M_PI;
  float x_rad = (x / 360) * twoPi;
  float heading_rad;

  const float magnetic_declination = (2.84 / 360) * twoPi;  // 2.84 degrees east in Dallas
  // IMU is mounted with pin 1 facing back right corner, i.e. North edge of chip points out right
  // side of robot, East edge point out back of robot
  const float IMU_mount_declination = (0 / 360) * twoPi;
  const float east_is_0_correction = 0 - M_PI_2;
  const float declination_correction = magnetic_declination + IMU_mount_declination + east_is_0_correction;

  // REP-103 defines yaw = 0 as east, +ve yaw is CCW. 0 from compass is North.
  heading_rad = twoPi - (x_rad + declination_correction);

  // normalize output to 0 - 2*pi
  if (heading_rad > twoPi)
    heading_rad -= twoPi;
  if (heading_rad < 0)
    heading_rad += twoPi;

 return heading_rad;
}

