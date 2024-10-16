#include "stm32f10x.h"                  // Device header

#include "pwm.h"
#include "tb6612fng.h"

//=============================================================================
//�ļ����ƣ�pwm
//���ܸ�Ҫ��PWM��IO�ڳ�ʼ��   PWM�Ķ�ʱ���ĳ�ʼ��   PWM��ʼ��
//����˵��������speed�����ٶ�
//�������أ���
//=============================================================================

//PWM��IO�ڳ�ʼ��
static void PWM_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(PWM_TIM_CH1_GPIO_CLK|PWM_TIM_CH2_GPIO_CLK,ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      //���츴��
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	
	// TIM2 ����Ƚ�ͨ��1 GPIO ��ʼ��
  GPIO_InitStructure.GPIO_Pin = PWM_TIM_CH1_PIN;
  GPIO_Init(PWM_TIM_CH1_PORT, &GPIO_InitStructure);
	// TIM2 ����Ƚ�ͨ��2 GPIO ��ʼ��
  GPIO_InitStructure.GPIO_Pin = PWM_TIM_CH2_PIN;
  GPIO_Init(PWM_TIM_CH2_PORT, &GPIO_InitStructure);


//PWM�Ķ�ʱ���ĳ�ʼ��
static void PWM_TIM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
	
	// ������ʱ��ʱ��,���ڲ�ʱ��CK_INT=72M
	PWM_TIM_APBxClock_Cmd(PWM_TIM_CLK, ENABLE);

/*--------------------ʱ���ṹ���ʼ��-------------------------*/
	// ��������
	TIM_TimeBaseStructure.TIM_Period=PWM_TIM_ARR;									// �Զ���װ�ؼĴ�����ֵ���ۼ�TIM_Period+1��Ƶ�ʺ����һ�����»����ж�
	TIM_TimeBaseStructure.TIM_Prescaler= PWM_TIM_PSC;							// ����CNT��������ʱ�� = Fck_int/(psc+1)
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;					// ʱ�ӷ�Ƶ���� ����������ʱ��ʱ��Ҫ�õ�
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;			// ����������ģʽ������Ϊ���ϼ���
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;								// �ظ���������ֵ��û�õ����ù�
	TIM_TimeBaseInit(PWM_TIM, &TIM_TimeBaseStructure);						// ��ʼ����ʱ��
	
	/*--------------------����ȽϽṹ���ʼ��-------------------*/	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;							// ����ΪPWMģʽ��PWMģʽ������ARR��Ч
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	// �Ƚ����ʹ�ܣ�ʹ�ܣ������ҪPWM
	
	// ����Ƚ�ͨ�� 1
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;			// ���ͨ����ƽ�������ã���	
	TIM_OC1Init(PWM_TIM, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(PWM_TIM, TIM_OCPreload_Enable); 					//ʹ��TIM14��CCR1�ϵ�Ԥװ�ؼĴ���
	
	// ����Ƚ�ͨ�� 2
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;			// ���ͨ����ƽ�������ã���
	TIM_OC2Init(PWM_TIM, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);					//ʹ��TIM14��CCR1�ϵ�Ԥװ�ؼĴ���
	
	
	// ʹ�ܼ�����
	TIM_Cmd(PWM_TIM, ENABLE);

}

//PWM��ʼ��
void PWM_Init(void)
{
	PWM_GPIO_Init();
	PWM_TIM_Init();
}

//С����
void PWM_CarGo(u16 motor1,u16 motor3)
{

	TIM_SetCompare2(PWM_TIM,motor3);
	TIM_SetCompare1(PWM_TIM,motor1);
}
