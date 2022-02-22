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
    vTaskDelay(2000);   // periodically print the failed message
  }

  return isok;
}

void
MowbotOdometry::run(void* params)
{
  odomLog_.infoln("Running MowbotOdometryTask");

  // frame time keeper
  TickType_t lastFrameTime = xTaskGetTickCount();

  // loop forever in the task, updating the global odometry struct
  while (true)
  {
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

    float deltaL_m = deltaLCounts * encoderMetersPerIrq;
    float deltaR_m = deltaRCounts * encoderMetersPerIrq;
    float deltaD_m = (deltaL_m + deltaR_m) / 2.0;   // distance traveled this frame
    odometer_m_ += deltaD_m;
    float deltaHeading = (deltaR_m - deltaL_m) / wheelbase_m;  // radians turned this frame
    angular_speed_rps_ = deltaHeading * (1000 / frameTime_ms);

    heading_rad_ += deltaHeading;
    heading_rad_ -= (float)((int)(heading_rad_ / (2 * M_PI))) * 2 * M_PI;  // clip to +/- 2 * pi
  
    poseX_m_ += deltaD_m * sin(heading_rad_);
    poseY_m_ += deltaD_m * cos(heading_rad_);

    // compute speed based on most recent deltaTicks if any counts received, else speed is 0
    if(deltaLCounts > 0)
    {
      leftSpeed_ = encoderMetersPerIrq / static_cast<float>(deltaLTicks) * 1000.0;
    }
    else
    {
      leftSpeed_ = 0.0;
    }
    if(deltaRCounts > 0)
    {
      rightSpeed_ = encoderMetersPerIrq / static_cast<float>(deltaRTicks) * 1000.0;
    }
    else
    {
      rightSpeed_ = 0.0;
    }

    linear_speed_mps_ = (leftSpeed_ + rightSpeed_) / 2.0;
    speedX_mps_ = linear_speed_mps_ * sin(heading_rad_);
    speedY_mps_ = linear_speed_mps_ * cos(heading_rad_);

    // Publish odometry
    OdometryMsg odom;
    populateOdomStruct(odom);
    mediator_->publishOdometry(odom);

    // delay until start of next 50ms frame
    vTaskDelayUntil(&lastFrameTime, frameTime_ms);
  }

}

void
MowbotOdometry::getOdometry(float& poseX, float& poseY, float& heading,
                            float& speedX, float& speedY, float& linearSpeed,
                            float& angular_speed, float& odometer,
                            float& speedL, float& speedR)
{
  poseX = poseX_m_;
  poseY = poseY_m_;
  heading = heading_rad_;
  speedX = speedX_mps_;
  speedY = speedY_mps_;
  linearSpeed = linear_speed_mps_;
  angular_speed = angular_speed_rps_;
  odometer = odometer_m_;
  speedL = leftSpeed_;
  speedR = rightSpeed_;
}

void
MowbotOdometry::populateOdomStruct(OdometryMsg& odom)
{
  odom.poseX_m = poseX_m_;
  odom.poseY_m = poseY_m_;
  odom.heading_rad = heading_rad_;
  odom.speedX_mps = speedX_mps_;
  odom.speedY_mps = speedY_mps_;
  odom.linear_speed_mps = linear_speed_mps_;
  odom.angular_speed_rps = angular_speed_rps_;
  odom.odometer_m = odometer_m_;
  odom.leftSpeed = leftSpeed_;
  odom.rightSpeed = rightSpeed_;

  odom.leftEncoderCount = leftEncoderCount_;
  odom.rightEncoderCount = rightEncoderCount_;
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
  leftFwd_ = leftFwd;
  rightFwd_ = rightFwd;
}

