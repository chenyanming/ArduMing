float heading; 
 
 void   Compass_Init(){
 
 
   
      if (!compass.init()) {
        hal.console->println("compass initialisation failed!");
        while (1) ;
    }
    hal.console->println("init done");

    compass.set_orientation(AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD); // set compass's orientation on aircraft.
    compass.set_offsets(0,0,0); // set offsets to account for surrounding interference
    compass.set_declination(ToRad(0.0)); // set local difference between magnetic north and true north

    hal.console->print("Compass auto-detected as: ");
    switch( compass.product_id ) {
    case AP_COMPASS_TYPE_HIL:
        hal.console->println("HIL");
        break;
    case AP_COMPASS_TYPE_HMC5843:
        hal.console->println("HMC5843");
        break;
    case AP_COMPASS_TYPE_HMC5883L:
        hal.console->println("HMC5883L");
        break;
    case AP_COMPASS_TYPE_PX4:
        hal.console->println("PX4");
        break;
    default:
        hal.console->println("unknown");
        break;
    }

    hal.scheduler->delay(1000);
    timer_compass = hal.scheduler->micros();
 
 
 
 
 }
 
 
 void Compass_Update(){
 
   
         static float min[3], max[3], offset[3];

    compass.accumulate();

    if((hal.scheduler->micros()- timer_compass) > 100000L/4)
    {
        timer_compass = hal.scheduler->micros();
        compass.read();
        unsigned long read_time = hal.scheduler->micros() - timer_compass;
     

        if (!compass.healthy) {
            hal.console->println("not healthy");
            return;
        }
   //     heading = compass.calculate_heading(0,0); // roll = 0, pitch = 0 for this example
        heading = ToDeg(compass.calculate_heading(0,0)) - 6; // roll = 0, pitch = 0 for this example
//          hal.console->print("heading is");
//          hal.console->print(heading);
//          hal.console->println();
//       
            
        compass.null_offsets();

        // capture min
        if( compass.mag_x < min[0] )
            min[0] = compass.mag_x;
        if( compass.mag_y < min[1] )
            min[1] = compass.mag_y;
        if( compass.mag_z < min[2] )
            min[2] = compass.mag_z;

        // capture max
        if( compass.mag_x > max[0] )
            max[0] = compass.mag_x;
        if( compass.mag_y > max[1] )
            max[1] = compass.mag_y;
        if( compass.mag_z > max[2] )
            max[2] = compass.mag_z;

        // calculate offsets
        offset[0] = -(max[0]+min[0])/2;
        offset[1] = -(max[1]+min[1])/2;
        offset[2] = -(max[2]+min[2])/2;

//        // display all to user
//        hal.console->printf("Heading: %.2f (%3d,%3d,%3d) i2c error: %u",
//			    ToDeg(heading),
//			    (int)compass.mag_x,
//			    (int)compass.mag_y,
//			    (int)compass.mag_z, 
//			    (unsigned)hal.i2c->lockup_count());
//
//        // display offsets
//        hal.console->printf(" offsets(%.2f, %.2f, %.2f)",
//                      offset[0], offset[1], offset[2]);
//
//        hal.console->printf(" t=%u", (unsigned)read_time);

  //      hal.console->println();
    } else {
	    hal.scheduler->delay(1);
    }
 
 
 
 
 
 
 
 }
