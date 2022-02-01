#include <MowbotOdometry.h>
#include <ArduinoLog.h>

extern QueueHandle_t encoderMsgQ;

// encoder ISRs - global names for now, using global Q name
void IRAM_ATTR leftEncChange()
{
  EncoderInterruptMsg encMsg;
  encMsg.leftFlag = true;
  encMsg.tickCount = xTaskGetTickCountFromISR();

  xQueueSendFromISR(encoderMsgQ, &encMsg, NULL);   // enqueue transition time
}

void IRAM_ATTR rightEncChange()
{
  EncoderInterruptMsg encMsg;
  encMsg.leftFlag = false;
  encMsg.tickCount = xTaskGetTickCountFromISR();

  xQueueSendFromISR(encoderMsgQ, &encMsg, NULL);   // enqueue transition time
}

MowbotOdometry::MowbotOdometry(QueueHandle_t& encoderMsgQ)
  : encoderMsgQ_(encoderMsgQ)
{
}

bool
MowbotOdometry::init()
{
  bool isok = true;

  // configure encoder input pins & attach interrupt handlers
  const int interruptPinLeft = 14;
  const int interruptPinRight = 15;

  pinMode(interruptPinLeft, INPUT_PULLUP);
  pinMode(interruptPinRight, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinLeft), leftEncChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinRight), rightEncChange, CHANGE);

/*
 * \brief encoder message queue length
 * \details At full speed each wheel will see about 80 transitions (80 interrupts)
 * each resulting in an encInterruptMsg. This queue needs to be big enough to handle
 * them until emptied (which should happen each 50ms frame). Allow 2x capacity margin
 * in case base encoder task is late to run in a subsequent frame. I.e. max number of
 * items should be:
 * 2 (encoders) * 2 (margin) * (80 (counts/sec) / 20 (frames/sec)) (max 4 interrupts/frame)
 */
  static const uint8_t msg_queue_len = 16;

  // Create queue of encoder transition times
  encoderMsgQ_ = xQueueCreate(msg_queue_len, sizeof(EncoderInterruptMsg));
  if (NULL == encoderMsgQ_)
  {
    isok = false;
  }
  return isok;
}

void
MowbotOdometry::run(void* params)
{
  Log.infoln("Running MowbotOdometryTask");

  // frame time keeper
  TickType_t lastFrameTime = xTaskGetTickCount();

  // loop forever in the task, updating the global odometry struct
  while (true)
  {
    int deltaLCounts = 0;
    int deltaLTicks = 0;
    int deltaRCounts = 0;
    int deltaRTicks = 0;
    EncoderInterruptMsg msgBuf;
    int msgCount = 0;
  
    // Read encoder messages while there are any enqueued
    while (uxQueueMessagesWaiting(encoderMsgQ))
    {
      xQueueReceive(encoderMsgQ, &msgBuf, 0);
      msgCount++;
      if(msgBuf.leftFlag)
      {
        deltaLCounts++;
        deltaLTicks = msgBuf.tickCount - lastLeftTick_;
        lastLeftTick_ = msgBuf.tickCount;
      }
      else
      {
        deltaRCounts++;
        deltaRTicks = msgBuf.tickCount - lastRightTick_;
        lastRightTick_ = msgBuf.tickCount;
      }
    }
  
    leftEncoderCount_ += deltaLCounts;
    rightEncoderCount_ += deltaRCounts;
  
    // compute speed based on most recent deltaTicks if any counts received, else speed is 0
    if(deltaLCounts > 0)
    {
      leftSpeed_ = encoderMetersPerTick / static_cast<float>(deltaLTicks) * 1000.0;
    }
    else
    {
      leftSpeed_ = 0.0;
    }
    if(deltaRCounts > 0)
    {
      rightSpeed_ = encoderMetersPerTick / static_cast<float>(deltaRTicks) * 1000.0;
    }
    else
    {
      rightSpeed_ = 0.0;
    }

    // delay until start of next 50ms frame
    vTaskDelayUntil(&lastFrameTime, 50);
  }

}

void
MowbotOdometry::getOdometry(int& leftCounts, float& speedL, int& rightCounts, float& speedR)
{
  leftCounts = leftEncoderCount_;
  speedL = leftSpeed_;
  rightCounts = rightEncoderCount_;
  speedR = rightSpeed_;
}