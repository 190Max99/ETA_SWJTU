 #ifndef CODE_INIT_H_
#define CODE_INIT_H_

#include "zf_common_headfile.h"

#define ENCODER_DIR1 (TC_CH58_ENCODER)                 // 带方向编码器对应使用的编码器接口
#define ENCODER_DIR_PULSE1 (TC_CH58_ENCODER_CH1_P17_3) // PULSE 对应的引脚
#define ENCODER_DIR_DIR1 (TC_CH58_ENCODER_CH2_P17_4)   // DIR 对应的引脚

#define KEY8 (P19_3) // 按键定义，KEY1对应P20_0
#define KEY7 (P19_4) //  按键定义，KEY2对应P20_1
#define KEY6 (P20_0) //  按键定义，KEY3对应P20_2
#define KEY5 (P20_1) //  按键定义，KEY4对应P20_3
#define KEY4 (P20_2) // 按键定义，KEY1对应P20_0
#define KEY3 (P20_3) //  按键定义，KEY2对应P20_1
#define KEY2 (P21_5) //  按键定义，KEY3对应P20_2
#define KEY1 (P21_6) //  按键定义，KEY4对应P20_3

#define CHANNEL_NUMBER (5) // PWM通道数量

#define PWM_CH3 (TCPWM_CH30_P10_2)   // PWM通道定义，PWM_CH3对应P10_2
#define PWM_CH4 (TCPWM_CH31_P10_3)   // PWM通道定义，PWM_CH4对应P10_3
#define PWM_Servo (TCPWM_CH00_P03_1) // PWM通道定义，PWM_CH5对应P03_0

#define LED1 (P19_0) // LED定义，LED1对应P19_0

#define FLASH_SECTION_INDEX       (0)                                 // 存储数据用的扇区
#define FLASH_PAGE_INDEX          (0)                                // 存储数据用的页码 倒数第一个页码

void Init_cm7_0(void); // 初始化函数

#endif /* CODE_MOTOR_H_ */