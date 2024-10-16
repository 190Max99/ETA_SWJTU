#ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"



// ʹ��TIM2
#define		PWM_TIM_APBxClock_Cmd  RCC_APB1PeriphClockCmd
#define		PWM_TIM_CLK            RCC_APB1Periph_TIM2
#define		PWM_TIM                TIM2
#define		PWM_TIM_ARR            7199  //��װ��ֵ
#define		PWM_TIM_PSC            0     //Ԥ��Ƶ
// TIM2 ����Ƚ�ͨ��2�����3
#define		PWM_TIM_CH2_GPIO_CLK   RCC_APB2Periph_GPIOA
#define		PWM_TIM_CH2_PORT       GPIOA
#define		PWM_TIM_CH2_PIN        GPIO_Pin_1
// TIM2 ����Ƚ�ͨ��4�����1
#define		PWM_TIM_CH1_GPIO_CLK   RCC_APB2Periph_GPIOA
#define		PWM_TIM_CH1_PORT       GPIOA
#define		PWM_TIM_CH1_PIN        GPIO_Pin_0
void PWM_Init(void);												//PWM��ʼ��
void PWM_CarGo(u16 motor1,u16 motor3);		//С����

#endif
