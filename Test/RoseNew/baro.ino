int baro_current,baro_lock;
long barothrottle_lock;
extern  long throttle;
void Baro_Init(){

  
    /* What's this for? */
    hal.gpio->pinMode(63, GPIO_OUTPUT);
    hal.gpio->write(63, 1);
    
    baro.init();
    baro.calibrate();

    timer = hal.scheduler->micros();
    
    baro_current = 0, baro_lock =0;



}


float Baro_Filter(float alt){

 static  float Fit_pre ,Fit,Fit_lat;
  float temp,altitude;
  static int status_count;
  
  if(status_count == 0)
  {
     Fit = alt;
     status_count++;
  }else if(status_count == 1)
  {
      Fit_pre = Fit;
      Fit = alt;
      status_count++;
  }else if(status_count == 2)
 {  
       Fit_lat = Fit ; 
       Fit = alt;
       status_count = 0;
       
 }

   temp = (Fit_pre + Fit + Fit_lat)/3.0;
//      hal.console->print(" temp is:");
//      hal.console->print(temp);
//      hal.console->println();
      
   if(abs(temp - alt)< 0.07)
   {
//      altitude = alt;     
//   }else{
     altitude = temp;
   }
     return altitude;
}


void  Baro_Update(){

  
     
    if((hal.scheduler->micros() - timer) > 100000UL/4) {
        timer = hal.scheduler->micros();
        baro.read();
        uint32_t read_time = hal.scheduler->micros() - timer;
        if (!baro.healthy) {
            hal.console->println("not healthy");
            return;
        }
//        hal.console->print("Pressure:");
//        hal.console->print(baro.get_pressure());
//        hal.console->print(" Temperature:");
//        hal.console->print(baro.get_temperature());
//        hal.console->print(" Altitude:");
//        hal.console->print(baro.get_altitude());
         baro.get_pressure();
         baro.get_temperature();
        float alt = baro.get_altitude();
        float altitude = Baro_Filter(alt);
         baro_current = (int)(altitude*100); 
         
//          hal.console->print(" Altitude is:");
//          hal.console->print(altitude);
//         
      
       
//        hal.console->printf(" climb=%.2f t=%u samples=%u",
//                      baro.get_climb_rate(),
//                      (unsigned)read_time,
//                      (unsigned)baro.get_pressure_samples());
    //    hal.console->println();
    }


}
