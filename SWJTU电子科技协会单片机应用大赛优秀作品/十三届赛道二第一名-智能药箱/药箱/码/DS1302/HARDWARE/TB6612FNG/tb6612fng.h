#ifndef __TB6612FNG_H
#define __TB6612FNG_H

#include "stm32f10x.h"

/*-------------------------------------------------------------------------------
С����������
���1������ PA0 PA1		���3������ PA5 PA6
STBY1��		   PA2			STBY2��		   PA7
���2������ PA3 PA4		���4������ PB0 PB1
BrakeAll��ɲ��   Go��ǰ��   BackGo������   CW��˳ʱ��   CCW����ʱ��   BRAKE��ɲ��

---------------------------------------------------------------------------------*/

//���1
#define  MOTOR1_GPIO_CLK   RCC_APB2Periph_GPIOA
#define  MOTOR1_GPIO_PORT  GPIOA
#define  MOTOR1_GPIO_PIN1  GPIO_Pin_7
#define  MOTOR1_GPIO_PIN2  GPIO_Pin_5
#define  MOTOR1_CW				 {GPIO_ResetBits(MOTOR1_GPIO_PORT,MOTOR1_GPIO_PIN1);GPIO_SetBits(MOTOR1_GPIO_PORT,MOTOR1_GPIO_PIN2);}//˳ʱ�룺IN1=0��IN2=1
#define  MOTOR1_CCW				 {GPIO_SetBits(MOTOR1_GPIO_PORT,MOTOR1_GPIO_PIN1);GPIO_ResetBits(MOTOR1_GPIO_PORT,MOTOR1_GPIO_PIN2);}//��ʱ�룺IN1=1��IN2=0
#define  MOTOR1_BRAKE			 GPIO_ResetBits(MOTOR1_GPIO_PORT,MOTOR1_GPIO_PIN1|MOTOR1_GPIO_PIN2);//ɲ��
//STBY1
/*#define  STBY1_GPIO_CLK    RCC_APB2Periph_GPIOA
#define  STBY1_GPIO_PORT 	 GPIOA
#define  STBY1_GPIO_PIN  	 GPIO_Pin_2
#define  STBY1_High				 GPIO_SetBits(STBY1_GPIO_PORT,STBY1_GPIO_PIN);	//STBY=1
#define  STBY1_Low				 GPIO_ResetBits(STBY1_GPIO_PORT,STBY1_GPIO_PIN);//STBY=0*/
/*//���2
#define  MOTOR2_GPIO_CLK   RCC_APB2Periph_GPIOA
#define  MOTOR2_GPIO_PORT  GPIOA
#define  MOTOR2_GPIO_PIN1	 GPIO_Pin_6
#define  MOTOR2_GPIO_PIN2	 GPIO_Pin_4
#define  MOTOR2_CW				 {GPIO_ResetBits(MOTOR2_GPIO_PORT,MOTOR2_GPIO_PIN1);GPIO_SetBits(MOTOR2_GPIO_PORT,MOTOR2_GPIO_PIN2);}//˳ʱ�룺IN1=0��IN2=1
#define  MOTOR2_CCW				 {GPIO_SetBits(MOTOR2_GPIO_PORT,MOTOR2_GPIO_PIN1);GPIO_ResetBits(MOTOR2_GPIO_PORT,MOTOR2_GPIO_PIN2);}//��ʱ�룺IN1=1��IN2=0
#define  MOTOR2_BRAKE			 GPIO_ResetBits(MOTOR2_GPIO_PORT,MOTOR2_GPIO_PIN1|MOTOR2_GPIO_PIN2);*///ɲ��

//���3
#define  MOTOR3_GPIO_CLK   RCC_APB2Periph_GPIOA
#define  MOTOR3_GPIO_PORT  GPIOA
#define  MOTOR3_GPIO_PIN1	 GPIO_Pin_6
#define  MOTOR3_GPIO_PIN2	 GPIO_Pin_4
#define  MOTOR3_CW				 {GPIO_ResetBits(MOTOR3_GPIO_PORT,MOTOR3_GPIO_PIN1);GPIO_SetBits(MOTOR3_GPIO_PORT,MOTOR3_GPIO_PIN2);}//˳ʱ�룺IN1=0��IN2=1
#define  MOTOR3_CCW				 {GPIO_SetBits(MOTOR3_GPIO_PORT,MOTOR3_GPIO_PIN1);GPIO_ResetBits(MOTOR3_GPIO_PORT,MOTOR3_GPIO_PIN2);}//��ʱ�룺IN1=1��IN2=0
#define  MOTOR3_BRAKE			 GPIO_ResetBits(MOTOR3_GPIO_PORT,MOTOR3_GPIO_PIN1|MOTOR3_GPIO_PIN2);//ɲ��
/*//STBY2
#define  STBY2_GPIO_CLK    RCC_APB2Periph_GPIOA
#define  STBY2_GPIO_PORT 	 GPIOA
#define  STBY2_GPIO_PIN  	 GPIO_Pin_7
#define  STBY2_High				 GPIO_SetBits(STBY2_GPIO_PORT,STBY2_GPIO_PIN);	//STBY=1
#define  STBY2_Low				 GPIO_ResetBits(STBY2_GPIO_PORT,STBY2_GPIO_PIN);//STBY=0*/
//���4
/*#define  MOTOR4_GPIO_CLK   RCC_APB2Periph_GPIOB
#define  MOTOR4_GPIO_PORT  GPIOB
#define  MOTOR4_GPIO_PIN1  GPIO_Pin_0
#define  MOTOR4_GPIO_PIN2  GPIO_Pin_1
#define  MOTOR4_CW				 {GPIO_ResetBits(MOTOR4_GPIO_PORT,MOTOR4_GPIO_PIN1);GPIO_SetBits(MOTOR4_GPIO_PORT,MOTOR4_GPIO_PIN2);}//˳ʱ�룺IN1=0��IN2=1
#define  MOTOR4_CCW				 {GPIO_SetBits(MOTOR4_GPIO_PORT,MOTOR4_GPIO_PIN1);GPIO_ResetBits(MOTOR4_GPIO_PORT,MOTOR4_GPIO_PIN2);}//��ʱ�룺IN1=1��IN2=0
#define  MOTOR4_BRAKE			 GPIO_ResetBits(MOTOR4_GPIO_PORT,MOTOR4_GPIO_PIN1|MOTOR4_GPIO_PIN2);*///ɲ��
/**************************** �������� ****************************/

void TB6612FNG_GPIO_Init(void);     //���GPIO����

void TB6612FNG_CarStop(void);											//С��ֹͣ0
void TB6612FNG_CarTurnfront(void); 								//С��ǰ��1
void TB6612FNG_CarTurnback(void);  								//С������2
void TB6612FNG_CarTurnleft(void);									//С����ת3
void TB6612FNG_CarTurnright(void);								//С����ת4
void TB6612FNG_CarLeftaround(void);								//С������5
void TB6612FNG_CarRightaround(void);							//С������6
void TB6612FNG_CarAnticlockwise(void);						//С����ʱ��7
void TB6612FNG_CarClockwise(void);								//С��˳ʱ��8

#endif
