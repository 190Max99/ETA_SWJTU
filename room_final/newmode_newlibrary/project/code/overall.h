#ifndef OVERALL_H_
#define OVERALL_H_

#include "zf_common_headfile.h"
extern uint8 gnss_IMU_flag;
extern uint8 gnss_dirction_flag;
extern uint8 voice_flag;
extern int16 distance_count;
extern uint8 IMU_flag;
extern float dis_flag;
extern float acc_angle;
extern float goal_angle;
extern  uint8 en_flg;
void overall_proc();

#endif /* OVERALL_H_ */
