This branch is designed for adjusting the ESC corrsponding to the controllable range of throttle(1000 to 1900).

As for writeMicroseconds(), a parameter value of 1000 on standard servos is fully counter-clockwise, 2000 is fully clockwise, and 1500 is in the middle. http://www.arduino.cc/en/Reference/ServoWriteMicroseconds

Controlling the ESC just likes controlling the servo. 

The way to Calibrate the ESC is to ouput the throttle directly.

First, set the Max Throttle to 1900, and the Min Throttle to 1000, using the Constrain(). Make sure that controlling the throttle stick will change the throttle value within the range in a constant step.

Second, output the throttle value using writeMicroseconds() directly when boot up.

Last, unplug the power of ESC, push the throttle stick to the max point. Then connect the power of ESC. ESC would sound 3 beeps(indicate 3S), and then 2 beeps(indicate capture the max throttle.) Push the throttle stick to min point. ESC will sound 1 beep(indicate capturing the min throttle).

Done.

!!!!Pay Attention!!!
Before calibrating ESCs, please ensure that your copter has NO PROPS on it and that the APM is NOT CONNECTED to your computer via USB and the Lipo battery is disconnected.
