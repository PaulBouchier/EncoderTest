# Encoder Test

This repo contains a Platform IO project that implements a period-based encoder-count handler. The code
is in src/main.cpp. The code runs on an esp32 and uses FreeRTOS functions for queue and time handling.

This code has barely been tested - it delivers speed and odometry when the wheels are turned by hand, but
it has not been run with motors being driven, or under closed-loop control.

Mowbot's wheel's encoder output transitions each motor
half-revolution. setup() attaches interrupt handlers that are called
each time a wheel's encoder output changes - leftEncChange() and
rightEncChange().

setup() creates a queue into which the interrupt handers, leftEncChange()
and rightEncChange() deposit an EncoderInterruptMsg containing a flag
indicating which wheel the transition came from, and a timestamp
which is the current time in milliseconds as given by the FreeRTOS call
xTaskGetTickCountFromISR().

loop() reads the queue every 50ms and extracts the EncoderInterruptMsg
messages. Each message represents a half-revolution of the motor (~7.5mm
of wheel motion) and the number of left and right messages is added to
left/rightEncoderCount, thus providing an odometer. loop() also computes
left/rightSpeed as 7.5mm divided by the period between the last two timestamps
from each wheel. If no messages were received from a wheel in the current
loop interation, speed is set to zero. The timestamp of the last received
message is saved for use next time through loop().

No direction information is available or saved. Thus the odometer only increases,
regardless of direction of motion.

A possible improvement of the algorithm would be to use more than just the last two timestamps
for speed calculation - it is unclear whether that would produce better (more average) results.
