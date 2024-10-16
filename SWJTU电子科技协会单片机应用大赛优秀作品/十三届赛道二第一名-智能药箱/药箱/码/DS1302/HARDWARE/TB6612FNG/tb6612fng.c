#include "tb6612fng.h"

//=============================================================================
//�ļ����ƣ�tb6612fng
//���ܸ�Ҫ��MOTOR��IO�ڳ�ʼ��  ɲ��  ǰ��  ����  ˳ʱ��  ��ʱ��
//����˵��������speed�����ٶ�
//�������أ���
//=============================================================================

//MOTOR��IO�ڳ�ʼ��
void TB6612FNG_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(MOTOR1_GPIO_CLK|MOTOR3_GPIO_CLK,ENABLE);//ʱ��
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//�������
	
	//���1
	GPIO_InitStructure.GPIO_Pin = MOTOR1_GPIO_PIN1|MOTOR1_GPIO_PIN2;
	GPIO_Init(MOTOR1_GPIO_PORT, &GPIO_InitStructure);
	//���2
	GPIO_InitStructure.GPIO_Pin = MOTOR3_GPIO_PIN1|MOTOR3_GPIO_PIN2;
	GPIO_Init(MOTOR3_GPIO_PORT, &GPIO_InitStructure);
	//С��ɲ��
//	MOTOR_CarBrakeAll();
}

//С��ֹͣ0
void TB6612FNG_CarStop(void)
{
	MOTOR1_BRAKE;	//С���˶������1
	MOTOR3_BRAKE;	//С���˶������2
	
}

//С��ǰ��1
void TB6612FNG_CarTurnfront(void)
{
	MOTOR1_CCW;	//С���˶������1
	MOTOR3_CCW;	//С���˶������2
	
}

//С������2
void TB6612FNG_CarTurnback(void) 								
{
	MOTOR1_CW;	//С���˶������1
	MOTOR3_CW;	//С���˶������2

}

//С����ת3
void TB6612FNG_CarTurnleft(void)
{
	MOTOR1_CCW; //С���˶������1
	MOTOR3_CCW; //С���˶������2
	
}

//С����ת4
void TB6612FNG_CarTurnright(void)
{
	MOTOR1_CCW;	//С���˶������1
	MOTOR3_CCW;	//С���˶������2
	
}

//С����ʱ��5
void TB6612FNG_CarAnticlockwise(void)
{
	MOTOR1_CW; //С���˶������1
	MOTOR3_CCW; //С���˶������2
	
}

//С��˳ʱ��6
void TB6612FNG_CarClockwise(void)
{
	MOTOR1_CCW;	//С���˶������1
	MOTOR3_CW;	//С���˶������2
	
}
