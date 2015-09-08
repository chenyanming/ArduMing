
float Pre_X, Pre_Y,PositionX_current, PositionY_current;
float start_X,start_Y;
//for start point record
long PosX_lock,PosY_lock;

int AutoXYcontrol=0;
extern long AUX1;


long PositionX = 0 ,PositionY = 0;
int runonce_gps;
//2 mins the accumulated error should decrease to 0 again
int G_count;

//for hover test
float X_lock,Y_lock;


//uint32_t timer_gps;
extern float heading;
extern int AutoZcontrol;
extern long AUX1;

void GPS_Init(){
  
    hal.uartB->begin(57600,128,128);
 //   gps.print_errors = true;

//    hal.console->println("GPS UBLOX library test");
//    gps.init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_2G);       // GPS Initialization
  
}


float Body2Global_X(float Body_X, float Body_Y){

           float X = Body_X * cos(heading /180.0 * PI)   - Body_Y * sin(heading /180.0 * PI);
           float Y = Body_X * sin(heading /180.0 * PI)   + Body_Y * cos(heading /180.0 * PI);


           return X;
}

float Body2Global_Y(float Body_X, float Body_Y){

           float X = Body_X * cos(heading /180.0 * PI)   - Body_Y * sin(heading /180.0 * PI);
           float Y = Body_X * sin(heading /180.0 * PI)   + Body_Y * cos(heading /180.0 * PI);

           return Y;
}


float Global2Body_X(float Glo_X, float Glo_Y){

           float X = Glo_Y * sin(heading /180.0 * PI)   + Glo_X * cos(heading /180.0 * PI);
           float Y = Glo_Y * cos(heading /180.0 * PI)   - Glo_X * sin(heading /180.0 * PI);


           return X;
}

float Global2Body_Y(float Glo_X, float Glo_Y){

           float X = Glo_Y * sin(heading /180.0 * PI)   + Glo_X * cos(heading /180.0 * PI);
           float Y = Glo_Y * cos(heading /180.0 * PI)   - Glo_X * sin(heading /180.0 * PI);


           return Y;



}


//convert latitude and longitude to centimeter in order to do position control
//Y direction 在经线(longitude)上，纬度每差1度，实际距离为111×cos(θ)千米；（其中θ表示该纬线的纬度。在不同纬线上，经度每差1度的实际距离是不相等的）.
//X direction 在纬线(latitude)上，经度每差1度，实际距离为111千米。
float GPS_ConvertX(float lat){

                float X_dis = (lat - start_X) * 111 *1000  ;      
                return X_dis;

 }

float GPS_ConvertY(float lon){

                float Y_dis = (lon - start_Y) * 111 *1000  *cos(22.3/180.0 * PI);
                return Y_dis;
                
}





float GPS_Filter_X(float parameter){

               static float Fit_1, Fit_2, Fit_3,Fit_4,Fit_5;
               static float temp_GPS;
               static int status_count;
                float Dis_X;
               
               if(status_count == 0)
               {
               Fit_1 = parameter;
               status_count++;
                 }else if(status_count == 1){
                  
                    Fit_2 = parameter;
                    status_count++;
                 
                 }else if(status_count == 2){
                      Fit_3 = parameter;
                      status_count++;   
                    
                    
                    
                     
                 }else if(status_count == 3){
                 
                   Fit_4 = parameter;
                    status_count++;
                 
                 
                 
                 }else if(status_count == 4){
                 
                   Fit_5 = parameter;
                    status_count = 0;
                 
                 temp_GPS = 0.2*Fit_1 + 0.2*Fit_2 + 0.2*Fit_3 + 0.2*Fit_4 + 0.2*Fit_5;
                 
                 
               //  Dis_X = temp_GPS - Pre_X;
                  hal.console->printf_P(PSTR("X is %f\r\n"), temp_GPS);
                   
             //      Pre_X = temp_GPS;
                 
                 }

       //     temp_GPS = (Fit_pre + Fit + Fit_lat)/3.0;
        
            return temp_GPS;

}


float GPS_Filter_Y(float parameter){

               static float Filter_1, Filter_2, Filter_3,Filter_4,Filter_5;
               static float temp_GPS_F;
               static int status_count_F;
               float Dis_Y;
               
               if(status_count_F == 0)
               {
               Filter_1 = parameter;
               status_count_F++;
                 }else if(status_count_F == 1){
                  
                    Filter_2 = parameter;
                    status_count_F++;
                 
                 }else if(status_count_F == 2){
                      Filter_3 = parameter;
                      status_count_F++;   
                    
                    
                    
                     
                 }else if(status_count_F == 3){
                 
                   Filter_4 = parameter;
                    status_count_F++;
                 
                 
                 
                 }else if(status_count_F == 4){
                 
                   Filter_5 = parameter;
                    status_count_F = 0;
                 
                 temp_GPS_F = 0.2*Filter_1 + 0.2*Filter_2 + 0.2*Filter_3 + 0.2*Filter_4+ 0.2 *Filter_5;
                 
                //   Dis_Y = temp_GPS_F - Pre_Y;
                           hal.console->printf_P(PSTR("Y is %f\r\n"), temp_GPS_F);
                 //      Pre_Y  = temp_GPS_F ;
                 }

 
            return temp_GPS_F;

}











void GPS_Update(){
  
     if((hal.scheduler->micros()- timer_gps) > 1000000L/5){
         timer_gps = hal.scheduler->micros();
       gps.update();
      //  hal.gpio->write(26, 1);
    if (gps.new_data) {
      
      hal.gpio->write(26, 0);
//        hal.console->print("gps:");
//        hal.console->print(" Lat:");
//        hal.console->print((float)gps.latitude / T7, DEC);
//        hal.console->print(" Lon:");
//        hal.console->print((float)gps.longitude / T7, DEC);
//        hal.console->print(" Alt:");
//        hal.console->print((float)gps.altitude / 100.0, DEC);
//        hal.console->print(" GSP:");
//        hal.console->print(gps.ground_speed / 100.0);
//        hal.console->print(" COG:");
//        hal.console->print(gps.ground_course / 100.0, DEC);
//        

        float start_X_temp = (float)gps.latitude / T7;
        float start_Y_temp = (float)gps.longitude / T7;
        if((runonce_gps == 0)&&(abs(start_X_temp > 5.0))&&(abs(start_Y_temp>5.0)))
        {
        start_X = (float)gps.latitude / T7;
        start_Y = (float)gps.longitude / T7;
        runonce_gps = 1;
        }
        
        float temp_x = (float)gps.latitude / T7;
        float temp_y = (float)gps.longitude / T7;
        
        
        PositionX_current = GPS_ConvertX(temp_x);
        PositionY_current = GPS_ConvertY(temp_y);
        
        
       hal.console->print("PositionX_current");   
       hal.console->print(PositionX_current);
         hal.console->print("PositionY_current");   
       hal.console->print(PositionY_current);
       
        float dis_x = GPS_Filter_X(PositionX_current);
        float dis_y = GPS_Filter_Y(PositionY_current);
        
        
         
        dis_x = Global2Body_X(dis_x,dis_y);
        dis_y = Global2Body_Y(dis_x,dis_y);
  
  
         PositionX = dis_x;
         PositionY = dis_y;
         
         
         
             
      if( (AUX1 > 1500) && (AutoXYcontrol == 0)){
      
          AutoXYcontrol = 1; 
        PosX_lock =  PositionX ;
        PosY_lock  = PositionY;
  
      }else if(AUX1 < 1500){
       
          AutoXYcontrol = 0 ;
         }
        
//        Vector2f vel = Vector2f(gps.velocity_north(), gps.velocity_east());
//        hal.console->printf(" VEL: %.2f %.2f %.2f",
//							vel.x,
//							vel.y,
//							vel.length());
        hal.console->print(" SAT:");
        hal.console->print(gps.num_sats, DEC);
        hal.console->println();
//        hal.console->print(" FIX:");
//        hal.console->print(gps.fix, DEC);
//        hal.console->print(" TIM:");
//        hal.console->print(gps.time, DEC);
//        hal.console->println();
        gps.new_data = 0; // We have readed the data
    }


     }



}
