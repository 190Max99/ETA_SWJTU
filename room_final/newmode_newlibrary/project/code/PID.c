#include "PID.h"
PID_IncTypeDef whell_pid;
PID_IncTypeDef servo_pid;
void PID_Inc_Init(PID_IncTypeDef *sptr, float kp, float ki, float kd)
{
    sptr->Ek1 = 0; // �ϴ�ƫ��ֵ��ʼ��
    sptr->Ek2 = 0; // ���ϴ�ƫ��ֵ��ʼ��

    sptr->Kp = kp; // ��������
    sptr->Ki = ki; // ���ֳ���
    sptr->Kd = kd; // ΢�ֳ���
    sptr->OUT = 0;
}
void PID_Pos_Init(PID_IncTypeDef *sptr, float kp, float kd)
{
    sptr->Ek1 = 0; // �ϴ�ƫ��ֵ��ʼ��
    sptr->Ek2 = 0; // ���ϴ�ƫ��ֵ��ʼ��

    sptr->Kp = kp; // ��������
    sptr->Kd = kd; // ΢�ֳ���
    sptr->OUT = 0;
}

float PID_Inc(float SetValue, float ActualValue, PID_IncTypeDef *PID)
{

    float PIDInc; // ����PID
    PID->Ek2 = PID->Ek1;
    PID->Ek1 = PID->Ek;
    PID->Ek = SetValue - ActualValue;
    PID->I_out=PID->Ki * PID->Ek;
    
    if( PID->I_out>=200)   PID->I_out=200;
    if( PID->I_out<=-200)  PID->I_out=-200;

    // /* �趨�ջ����� */
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

    //float PIDPos; // λ��ʽPID
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
