

int height_current, height_lock;
int AutoZcontrol;
long throttle_lock;
extern  long throttle;
extern long AUX1;

void Sonar_Init(){


   /*enable sonar*/
  AP_HAL::AnalogSource *analog_source = hal.analogin->channel(0);
  sonar = new AP_RangeFinder_MaxsonarXL(analog_source, &mode_filter);
  sonar->calculate_scaler(AP_RANGEFINDER_MAXSONARXL, 5);   // setup scaling for sonar
   height_current = 0;
   AutoZcontrol = 0;
}


void Sonar_update(){
    int temp_sonar_data = sonar->read();
    if(temp_sonar_data > 20 && temp_sonar_data < 300)       height_current  = temp_sonar_data;
    
  //  hal.console->printf_P(PSTR("height is %d\n"),height_current);
         if( (AUX1 > 1500) && (AutoZcontrol == 0)){
         AutoZcontrol = 1;
         height_lock = height_current;
         throttle_lock = throttle;
//         if(flag_opt == 0)
//         {
//          ResetPos();
//         
//         hal.scheduler->delay(45);
//         } 
//          
//        
//         PosX_lock = PositionX;
//         PosY_lock = PositionY;
    

         }else if(AUX1 < 1500){
     
          AutoZcontrol = 0 ;
          height_lock = 0;
          throttle_lock= 0;
         }

}
