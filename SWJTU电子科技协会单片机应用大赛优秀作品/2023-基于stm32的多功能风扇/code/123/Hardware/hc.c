//#include "hc.h"
#include "stm32f10x.h"                  // Device header
#include "Delay.h"
/*
	gpiob P5,echo 10
	gpiob P6,trig 11
	ʹ����TIM2������2�����ȼ�1,1
	
*/
u8 msHcCount = 0;//ms����

static void NVIC_Config()
{
	NVIC_InitTypeDef NVIC_InitStruct;
	//�����ж���Ϊ2
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//�����ж���Դ
	NVIC_InitStruct.NVIC_IRQChannel	= TIM3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	//���������ȼ�
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority	= 1;
	//���ô����ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority				= 1;
	//��ʼ��
	NVIC_Init(&NVIC_InitStruct);
	
}

void Hcsr04Init()
{  
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;     //�������ڶ�ʱ�����õĽṹ��
    GPIO_InitTypeDef GPIO_InitStructure;
	/*����GPIOBʱ��*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
     
        //IO��ʼ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ;   //���͵�ƽ����TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//�����������
    GPIO_Init(GPIOB , &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB,GPIO_Pin_11);//һ��ʼ�͵�ƽ
    
    GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_10 ;     //���ص�ƽ����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOB, &GPIO_InitStructure);  
    GPIO_ResetBits(GPIOB,GPIO_Pin_10); //Ĭ�ϵ͵�ƽ   
    
     //��ʱ����ʼ�� ʹ�û�����ʱ��TIM2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);   //ʹ�ܶ�ӦRCCʱ��
      //���ö�ʱ�������ṹ��
    TIM_DeInit(TIM3);
     //�Զ���װ��ֵ�Ĵ�����ֵ���ۼ�TIM_Period+1��Ƶ�ʺ����һ�����»����ж�
		TIM_TimeBaseStructure.TIM_Period			= 1000;//��������Ϊ1000us
		//ʱ��Ԥ��Ƶ��
		TIM_TimeBaseStructure.TIM_Prescaler		= 72-1;//��Ƶ��72
	
		//ʱ�ӷ�Ƶ����
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
		//����������ģʽ���������ϼ�����
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
		//�ظ���������ֵ
		//	TIM_TimeBaseStructure.TIM_RepetitionCounter=0 ;
    TIM_TimeBaseInit(TIM3 , &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ         
        
   // TIM_ClearFlag(BASIC_TIM, TIM_FLAG_Update);   //��������жϣ����һ���ж����������ж�
		TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update); //��������жϣ����һ���ж����������ж�
    TIM_ITConfig(TIM3 ,TIM_IT_Update,ENABLE);    //�򿪶�ʱ�������ж�
		NVIC_Config();
   TIM_Cmd(TIM3 ,DISABLE);     
}

void initHcsr04()
{
	Hcsr04Init();
	//BASIC_TIM_NVIC_Config();
}


static void OpenTimer()        //�򿪶�ʱ��
{
        //	/*���������*/
	TIM_SetCounter(TIM3 ,0);
	msHcCount = 0;
	TIM_Cmd(TIM3 ,ENABLE);//ʹ�ܶ�ʱ��
}
 
static void CloseTimer()        //�رն�ʱ��
{
       //	/*�رռ�����ʹ��*/
	TIM_Cmd(TIM3 ,DISABLE);
}
 

//��ʱ��2�жϷ������
void TIM3_IRQHandler(void)   //TIM3�ж�
{
        //	/*�ж��ж��ַ���Ĳ���*/
	if(TIM_GetITStatus(TIM3 ,TIM_IT_Update) != RESET){
		/*��������жϱ�־λ*/
		TIM_ClearITPendingBit(TIM3 ,TIM_IT_Update);
		msHcCount++;
	}
	
}

//��ȡ��ʱ��ʱ��
u32 GetEchoTimer(void)
{
   u32 time = 0;
	/*//�������źźܳ��ǣ�����ֵ������ظ�������overCount���ж��������������*/
	time = msHcCount*1000;//overCountÿ++һ�Σ�����overCount���룬time΢��
	time += TIM_GetCounter(TIM3);//��ȡ��TIM2���Ĵ����еļ���ֵ��һ�߼�������ź�ʱ��
	TIM6->CNT = 0;  //��TIM2�����Ĵ����ļ���ֵ����
	Delay_ms(50);
	return time;
 
}
float Hcsr04GetLength(void)
{
	/*��5�����ݼ���һ��ƽ��ֵ*/
	float length = 0;
	float t = 0;
	float sum = 0;
	u16  	i = 0;
	while(i != 5){
		GPIO_SetBits(GPIOB ,GPIO_Pin_11 );//trig�����źţ������ߵ�ƽ
		Delay_us(20);//����ʱ�䳬��10us
		GPIO_ResetBits(GPIOB ,GPIO_Pin_11 );
		/*Echo�����ź� �ȴ������ź�*/
		/*���뷽����ģ����Զ�����8��40KHz�����������ͬʱ�ز����ţ�echo���˵ĵ�ƽ����0��Ϊ1��
		����ʱӦ��������ʱ����ʱ���������������ر�ģ����յ�ʱ���ز��� �Ŷ˵ĵ�ƽ����1��Ϊ0��
		����ʱӦ��ֹͣ��ʱ������������ʱ�����µ����ʱ�伴Ϊ
			�������ɷ��䵽���ص���ʱ����*/
		while(GPIO_ReadInputDataBit(GPIOB ,GPIO_Pin_10) == 0);//echo�ȴ�����
		/*������ʱ��*/
		OpenTimer();
		i = i+1; //ÿ�յ�һ�λ����ź�+1���յ�5�ξͼ����ֵ
		while(GPIO_ReadInputDataBit(GPIOB ,GPIO_Pin_10) == 1);
		/*�رն�ʱ��*/
		CloseTimer();
		/*��ȡEcho�ߵ�ƽʱ��ʱ��*/
		t = GetEchoTimer();
		length = (float)t/58;//��λʱcm
		sum += length;		
	}
	length = sum/5;//���ƽ��ֵ
	
	return length;
}

