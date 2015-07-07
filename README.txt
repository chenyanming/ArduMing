
* This is a pure arduino based quadcopter project. Developed on Arduino 1.0.6.
* CHEN Yanming
* Department of Computing
* The Hong Kong Polytechnic University

PS:
Because the PPM signal of Remote Control is decoded by Timer5 Input Capture which will conflict with the Servo.h library default setting, changing the timer3 and timer5 order can fix this qusetion.

Revise one line of the file /Applications/Arduino.app/Contents/Resources/Java/libraries/Servo/Servo.h as following:

//typedef enum { _timer5, _timer1, _timer3, _timer4, _Nbr_16timers } timer16_Sequence_t ;
typedef enum { _timer3, _timer1, _timer5, _timer4, _Nbr_16timers } timer16_Sequence_t ;