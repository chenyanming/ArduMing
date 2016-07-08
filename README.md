# ArduMing
	A pure arduino based x configuration quadcopter project. Developed on Arduino 1.0.6.
    
1. [Parts](#toc_parts)
2. [Photo](#toc_photo)
3. [Attention](#toc_attention)
4. [Manual](#toc_manual)
5. [Contact](#toc_contact)

## Parts <a name="toc_parts"></a>
1. *Remote Control:* WFT06X-A 
2. *ESC:* Hobbywing, Sky Walker, QUATTRO 25A X 4 
3. *Motors:* SunnySky, X2212-13 KV: 980 II 
4. *Body:* F330 or F450
5. *Baterry:* CUAV 2200mAh, 3S, XT60

## Photo <a name="toc_photo"></a>
![Drone](https://raw.githubusercontent.com/chenyanming/ArduMing/dev/drone.jpg)

## Attention <a name="toc_attention"></a>
Because the PPM signal of Remote Control is decoded by Timer5 Input Capture which will conflict with the Servo.h library default setting, changing the timer3 and timer5 order can fix this qusetion.

>Revise one line of the file: 
/Applications/Arduino.app/Contents/Resources/Java/libraries/Servo/Servo.h as following:

```c
//typedef enum { _timer5, _timer1, _timer3, _timer4, _Nbr_16timers } timer16_Sequence_t ; typedef enum { _timer3, _timer1, _timer5, _timer4, _Nbr_16timers } timer16_Sequence_t ;
```

## Manual <a name="toc_manual"></a>
See https://github.com/chenyanming/ArduMing/blob/dev/ArduMingManualV1Jan31.pdf.

## Contact <a name="toc_contact"></a>
* [CHEN Yanming](mailto:elecming@gmail.com)
* [Department of Computing](http://www.comp.polyu.edu.hk/)
* [The Hong Kong Polytechnic University](http://www.polyu.edu.hk/web/en/home/index.html)
