


void Output_Init(){


       /*enable output channels for motors*/
  hal.rcout->set_freq(0xF, 490);  //specify indicated output channels with output frequench 490Hz.
  hal.rcout->enable_mask(0xFF);   //enable output channels, output channels 1-4 should be connected to motors. 



}



void LED_Test(){

//  hal.gpio->pinMode(25, GPIO_OUTPUT);
//  hal.gpio->write(25, 0);
//  hal.gpio->pinMode(26, GPIO_OUTPUT);
//  hal.gpio->write(26, 0);
  hal.gpio->pinMode(27, GPIO_OUTPUT);
  hal.gpio->write(27, 0);
  
  



}
