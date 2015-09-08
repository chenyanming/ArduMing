// input channel angle from remote
 long roll, pitch, throttle, yaw, AUX1;


//This function will return a mapping value(in range out_min-out_min) for the input x(in range in_min-in_max)
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min)* (out_max - out_min)/(in_max - in_min)+out_min;
}

 
 void read_input(){
 
 
   
      uint16_t inputs[8];  // store value of 8 input channels 
  
     //Read RC(radio code) singals from 8 input channels and store in inputs array
      hal.rcin->read(inputs, 8);
      
      
      
       roll = map(inputs[3], 1050, 1840, -45, 45);
       pitch = map(inputs[1], 1110, 1860, -45, 45);
       throttle = map(inputs[2], 1130, 1880, 1000, 1800);  //2000 is the maximum of throttle, set a lower value to remain some room for PID controls
       yaw = map(inputs[0], 1150, 1880, -180, 180);
       AUX1 =  inputs[4];
   //     hal.console->printf_P(PSTR("remote input 5 is %d\r\n"),inputs[4]);
  //fixing small errors of signals from remote controller
  if(abs(roll)<=5)
      roll = 0;
  if(abs(pitch)<=5)
      pitch = 0;
  if(abs(yaw)<=5)
      yaw = 0;
 
 }
