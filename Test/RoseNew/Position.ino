

long Pre_PositionX,Pre_PositionY;
//int AutoXYcontrol=0;
int Reset_flag = 0;



//for 3DR wireless module
char buffer[100] ;
int indexBuffer = 0;

uint32_t timer_ask = 0;
uint32_t timer_get = 0;
int asked = 0;

void extractMsg()
{
  long x = strtol(strtok(buffer, ","), NULL, 10);  
  long y = strtol(strtok(NULL, ","), NULL, 10); 
  long z = strtol(strtok(NULL, ","), NULL, 10);
  long yaw = strtol(strtok(NULL, ","), NULL, 10); 
//  long x_change = strtol(strtok(NULL, ","), NULL, 10);  
//  long y_change = strtol(strtok(NULL, ","), NULL, 10); 
 // long z_change = strtol(strtok(NULL, ","), NULL, 10);
  long ck = strtol(strtok(NULL, ","), NULL, 10); 
  
//  long checksum = x + y + z + yaw + x_change + y_change + z_change;
  long checksum = x + y + z + yaw;
  checksum = checksum%1000;
//   if(checksum == ck)
//   {
//   
//    // hal.console->printf_P(PSTR("checksum ok , x %ld y %ld z %ld yaw %ld \n"), x,y,z,yaw);
//   
//   }
////  
//  
  //hal.console->printf_P(PSTR("time: %d "),hal.scheduler->millis());
/*  
  if(checksum == ck)
  {
    x -= x_change;
    y -= y_change;
    //z -= z_change;
    
    if(abs(z_change) <= 50)
      z_change_sonar = -z_change;
    
    if(x > 99 && x < 330)
      x_ground = x;
    else
      hal.console->printf_P(PSTR("x_ground error %ld\n"), x);
    
    if(y > 99 && y < 330)
      y_ground = y;
    else
      hal.console->printf_P(PSTR("y_ground error %ld\n"), y);
      
    if(z > -230 && z < 0)
      z_ground = z;
    else
      hal.console->printf_P(PSTR("z_ground error %ld\n"), z);
      
    yaw_ground = yaw;
    
   
    time_lastMsgRead = hal.scheduler->millis();  
    groundMsgReady = 1;
    hal.console->printf_P(PSTR("checksum ok , x %ld y %ld z %ld yaw %ld x_change %ld y_change %ld z_change %ld\n"), x,y,z,yaw,x_change,y_change,z_change);
  }
  else
    hal.console->printf_P(PSTR("checksum fail\n"));
*/    
    
   
     
      if(checksum == ck)
      {
//        if(AutoXYcontrol == 1){
//        
//          if((Reset_flag == 0)&&( x<10 )&&( x>-10 )){
//          
//              Reset_flag = 1;
//              PositionX = x/5;
//              PositionY = y/5;    
//          
//          }else if(Reset_flag == 1){
//          
//              PositionX = x/5;
//              PositionY = y/5;  
//          
//          }
//
//      
//        
//        }else{
    
 
      }
    // hal.uartC->end();
      
    
}




void get3DRData()
{
  hal.gpio->write(25, 1);
  int bytesAvailable = hal.uartC->available();

  while(bytesAvailable > 0)
  {
  
    char c = (char)hal.uartC->read(); 
    if(c == 'S')
    {
   
      indexBuffer = 0;
    }else if (c == 'E')
    {
      buffer[indexBuffer] = '\0';
     // extractMsg();
     extractMsg();
    }  
    else
    {
      buffer[indexBuffer] = c;
      indexBuffer++;
    }
    bytesAvailable--;
  }
}



void Position_Init()
{
  
 
 
 /*enable 3DR wireless module on uartB, with 57600 baudrate, 128 read buffer, 128 transmition buffer*/

  hal.uartC->begin(57600,128,128);
  hal.gpio->pinMode(25, GPIO_OUTPUT);
//  hal.gpio->write(25, 0);
  
  
  
}


void Reset_opt(){

       hal.uartC->print("r");


}



void AskPos(){
       hal.gpio->write(25, 0);
       hal.uartC->print("a");
      

}

void PosXY_Update() 
{
  
    uint32_t now = hal.scheduler->micros();
    
    if((now - timer_ask) > 1000000L/50){
      
  
      
      
      timer_ask = now;
      if (asked == 0) {
        AskPos();
        asked = 1;
      } else {
       get3DRData();
       asked = 0;
     }
   }

   
    

    
  
//hal.scheduler->delay(100);



}
