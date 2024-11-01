#ifndef CODE_PID_H_
#define CODE_PID_H_
#include "zf_common_headfile.h"
typedef struct
{
    float Kp;  // ����ϵ��
    float Ki;  // ����ϵ��
    float Kd;  // ΢��ϵ��
    float Ek;  // ��ǰ���
    float Ek1; // ��һ�����
    float Ek2; // ���ϴ����
    float I_out;
    float OUT; // ���
} PID_IncTypeDef;
void PID_Inc_Init(PID_IncTypeDef *sptr, float kp, float ki, float kd); // ����ʽPID��ʼ��
float PID_Inc(float SetValue, float ActualValue, PID_IncTypeDef *PID); // ����ʽPID
float PID_Pos(float SetPostion, float ActualPostion, PID_IncTypeDef *PID); // λ��ʽPID
void PID_Pos_Init(PID_IncTypeDef *sptr, float kp,float kd); // λ��ʽPID��ʼ��
extern PID_IncTypeDef whell_pid;
extern PID_IncTypeDef servo_pid;
#endif /* CODE_PID_H_ */