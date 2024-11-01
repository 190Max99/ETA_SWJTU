 #ifndef CODE_INIT_H_
#define CODE_INIT_H_

#include "zf_common_headfile.h"

#define ENCODER_DIR1 (TC_CH58_ENCODER)                 // �������������Ӧʹ�õı������ӿ�
#define ENCODER_DIR_PULSE1 (TC_CH58_ENCODER_CH1_P17_3) // PULSE ��Ӧ������
#define ENCODER_DIR_DIR1 (TC_CH58_ENCODER_CH2_P17_4)   // DIR ��Ӧ������

#define KEY8 (P19_3) // �������壬KEY1��ӦP20_0
#define KEY7 (P19_4) //  �������壬KEY2��ӦP20_1
#define KEY6 (P20_0) //  �������壬KEY3��ӦP20_2
#define KEY5 (P20_1) //  �������壬KEY4��ӦP20_3
#define KEY4 (P20_2) // �������壬KEY1��ӦP20_0
#define KEY3 (P20_3) //  �������壬KEY2��ӦP20_1
#define KEY2 (P21_5) //  �������壬KEY3��ӦP20_2
#define KEY1 (P21_6) //  �������壬KEY4��ӦP20_3

#define CHANNEL_NUMBER (5) // PWMͨ������

#define PWM_CH3 (TCPWM_CH30_P10_2)   // PWMͨ�����壬PWM_CH3��ӦP10_2
#define PWM_CH4 (TCPWM_CH31_P10_3)   // PWMͨ�����壬PWM_CH4��ӦP10_3
#define PWM_Servo (TCPWM_CH00_P03_1) // PWMͨ�����壬PWM_CH5��ӦP03_0

#define LED1 (P19_0) // LED���壬LED1��ӦP19_0

#define FLASH_SECTION_INDEX       (0)                                 // �洢�����õ�����
#define FLASH_PAGE_INDEX          (0)                                // �洢�����õ�ҳ�� ������һ��ҳ��

void Init_cm7_0(void); // ��ʼ������

#endif /* CODE_MOTOR_H_ */