#include <Arduino.h>

// Globals
static QueueHandle_t msg_queue;
BaseType_t xHigherPriorityTaskWoken;

int leftEncoderCount = 0;     // running count of left encoder counts
int rightEncoderCount = 0;    // running count of right encoder counts
TickType_t lastLeftTick = 0;  // Time when last left tick was observed
TickType_t lastRightTick = 0; // Time when last right tick was observed
float leftSpeed = 0.0;
float rightSpeed = 0.0;

const int interruptPinLeft = 14;
const int interruptPinRight = 15;

const float encoderMetersPerTick = 0.0075;

struct EncoderInterruptMsg {
  bool leftFlag;  // true if this queue entry is for left encoder, false for right
  TickType_t tickCount;   // time when encoder interrupt happened
};

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

void IRAM_ATTR leftEncChange()
{
  EncoderInterruptMsg encMsg;
  encMsg.leftFlag = true;
  encMsg.tickCount = xTaskGetTickCountFromISR();

  xQueueSendFromISR(msg_queue, &encMsg, NULL);   // enqueue transition time
}

void IRAM_ATTR rightEncChange()
{
  EncoderInterruptMsg encMsg;
  encMsg.leftFlag = false;
  encMsg.tickCount = xTaskGetTickCountFromISR();

  xQueueSendFromISR(msg_queue, &encMsg, NULL);   // enqueue transition time
}

void setup() {
  // Configure Serial
  Serial.begin(9600);
  // Wait a moment to start (so we don't miss Serial output)
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("--- Encoder ISR Test ---");

  pinMode(interruptPinLeft, INPUT_PULLUP);
  pinMode(interruptPinRight, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinLeft), leftEncChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinRight), rightEncChange, CHANGE);

  // Create queue of encoder transition times
  msg_queue = xQueueCreate(msg_queue_len, sizeof(EncoderInterruptMsg));
}

void loop() {
  int deltaLCounts = 0;
  int deltaLTicks = 0;
  int deltaRCounts = 0;
  int deltaRTicks = 0;
  EncoderInterruptMsg msgBuf;
  int msgCount = 0;

  // Read encoder messages while there are any enqueued
  while (uxQueueMessagesWaiting(msg_queue))
  {
    xQueueReceive(msg_queue, &msgBuf, 0);
    msgCount++;
    if(msgBuf.leftFlag)
    {
      deltaLCounts++;
      deltaLTicks = msgBuf.tickCount - lastLeftTick;
      lastLeftTick = msgBuf.tickCount;
    }
    else
    {
      deltaRCounts++;
      deltaRTicks = msgBuf.tickCount - lastRightTick;
      lastRightTick = msgBuf.tickCount;
    }
  }

  leftEncoderCount += deltaLCounts;
  rightEncoderCount += deltaRCounts;

  // compute speed based on most recent deltaTicks if any counts received, else speed is 0
  if(deltaLCounts > 0)
  {
    leftSpeed = encoderMetersPerTick / static_cast<float>(deltaLTicks) * 1000.0;
  }
  else
  {
    leftSpeed = 0.0;
  }
  if(deltaRCounts > 0)
  {
    rightSpeed = encoderMetersPerTick / static_cast<float>(deltaRTicks) * 1000.0;
  }
  else
  {
    rightSpeed = 0.0;
  }

  if (deltaLCounts > 0 || deltaRCounts > 0)
  {
    Serial.print("deltaLCounts: ");
    Serial.print(deltaLCounts);
    Serial.print(" left speed: ");
    Serial.print(leftSpeed);
    Serial.print(" deltaRCounts: ");
    Serial.print(deltaRCounts);
    Serial.print(" right speed: ");
    Serial.print(rightSpeed);
    Serial.println();
  }
  vTaskDelay(50);
}