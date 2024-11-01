#ifndef CODE_PID_H_
#define CODE_PID_H_
#include "zf_common_headfile.h"
typedef struct
{
    float Kp;  // 比例系数
    float Ki;  // 积分系数
    float Kd;  // 微分系数
    float Ek;  // 当前误差
    float Ek1; // 上一次误差
    float Ek2; // 上上次误差
    float I_out;
    float OUT; // 输出
} PID_IncTypeDef;
void PID_Inc_Init(PID_IncTypeDef *sptr, float kp, float ki, float kd); // 增量式PID初始化
float PID_Inc(float SetValue, float ActualValue, PID_IncTypeDef *PID); // 增量式PID
float PID_Pos(float SetPostion, float ActualPostion, PID_IncTypeDef *PID); // 位置式PID
void PID_Pos_Init(PID_IncTypeDef *sptr, float kp,float kd); // 位置式PID初始化
extern PID_IncTypeDef whell_pid;
extern PID_IncTypeDef servo_pid;
#endif /* CODE_PID_H_ */