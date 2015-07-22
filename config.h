#ifndef CONFIG_H
#define CONFIG_H


#define MAX_SIGNAL 1900
#define MIN_SIGNAL 1000

// Remote Control 
void rc_setup();
void rc_get();
extern float throttle;

#endif
