
#include "motor.h"

void motor_control(int16 Output_Val)
{
  if (Output_Val > 6000)
  {
    Output_Val = 6000;
  }
  else if (Output_Val < -6000)
  {
    Output_Val = -6000;
  }
  // ���ֵ�޷�

  if (Output_Val >= 0)
  {
    pwm_set_duty(PWM_CH3, Output_Val); // ���¶�Ӧͨ��ռ�ձ�
    pwm_set_duty(PWM_CH4, 0);
  }
  else if (Output_Val < 0)
  {
    pwm_set_duty(PWM_CH3, 0); // ���¶�Ӧͨ��ռ�ձ�
    pwm_set_duty(PWM_CH4, -Output_Val);
  }
}