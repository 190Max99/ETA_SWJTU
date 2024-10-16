#ifndef __BEEP_H
#define __BEEP_H
#include "sys.h"


#define BEEP  PBout(7)


#define BEEP_GPIO_PORT	 GPIOB			            /* GPIO�˿� */
#define BEEP_RCC 	     RCC_APB2Periph_GPIOB		/* GPIO�˿�ʱ�� */
#define BEEP_PIN		     GPIO_Pin_7

void BEEP_GPIO_Init(void);

#endif

