#ifndef KEY_H
#define KEY_H

#include "zf_common_headfile.h"
extern uint8 key_delay;
extern uint8 safe;
extern uint8 page;
extern uint8 light_sec[8];
extern float kp_back;
extern float kp_forward;
extern float kd_back;
extern float kd_forward;
void key_proc(); // 按键处理函数
#endif
