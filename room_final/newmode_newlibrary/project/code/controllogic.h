#ifndef CONTROLLOGIC_H
#define CONTROLLOGIC_H

 extern int16 goal_speed ;
 extern int16 Output_Val; 
 extern int16 speed_high;
 extern int16 speed_low;
 extern double distance_test;
extern uint8 turn_flag;
extern double angle_error;
extern uint8 selct;

 #include "zf_common_headfile.h"
void speed_logic();  // ËÙ¶ÈÂß¼­

#endif