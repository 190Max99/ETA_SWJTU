#include "PID.h"
PID_IncTypeDef whell_pid;
PID_IncTypeDef servo_pid;
void PID_Inc_Init(PID_IncTypeDef *sptr, float kp, float ki, float kd)
{
    sptr->Ek1 = 0; // 上次偏差值初始化
    sptr->Ek2 = 0; // 上上次偏差值初始化

    sptr->Kp = kp; // 比例常数
    sptr->Ki = ki; // 积分常数
    sptr->Kd = kd; // 微分常数
    sptr->OUT = 0;
}
void PID_Pos_Init(PID_IncTypeDef *sptr, float kp, float kd)
{
    sptr->Ek1 = 0; // 上次偏差值初始化
    sptr->Ek2 = 0; // 上上次偏差值初始化

    sptr->Kp = kp; // 比例常数
    sptr->Kd = kd; // 微分常数
    sptr->OUT = 0;
}

float PID_Inc(float SetValue, float ActualValue, PID_IncTypeDef *PID)
{

    float PIDInc; // 增量PID
    PID->Ek2 = PID->Ek1;
    PID->Ek1 = PID->Ek;
    PID->Ek = SetValue - ActualValue;
    PID->I_out=PID->Ki * PID->Ek;
    
    if( PID->I_out>=200)   PID->I_out=200;
    if( PID->I_out<=-200)  PID->I_out=-200;

    // /* 设定闭环死区 */
    // if( ( PID->Ek>-8) && ( PID->Ek<8 ) )
    // {
    //     PID->Ek = 0;
    //     PID->Ek2 = 0;
    //     PID->Ek1 = 0;
    // }
    if (PID->Ek > 5 || PID->Ek < (-5))
    {
        PIDInc = (PID->Kp * (PID->Ek - PID->Ek1)) +PID->I_out+ (PID->Kd * (PID->Ek - 2 * PID->Ek1 + PID->Ek2));
    }
    else
    {
        PIDInc = 0;
    }
    return PIDInc;
}

float PID_Pos(float SetPostion, float ActualPostion, PID_IncTypeDef *PID)
{

    //float PIDPos; // 位置式PID
    PID->Ek1 = PID->Ek;
    PID->Ek = SetPostion - ActualPostion;
    if (PID->Ek > 180)
    {
        PID->Ek = PID->Ek - 360;
    }
    else if (PID->Ek < -180)
    {
        PID->Ek = PID->Ek + 360;
    }

    PID->OUT = PID->Kp * PID->Ek + PID->Kd * (PID->Ek - PID->Ek1);

    //PID->OUT = (PID->Kp * PID->Ek)*0.5+PID->OUT*0.5;// + PID->Kd * (PID->Ek - PID->Ek1);
    return PID->OUT;
}
