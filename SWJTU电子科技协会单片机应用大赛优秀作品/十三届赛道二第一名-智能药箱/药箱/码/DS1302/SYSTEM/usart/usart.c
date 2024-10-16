#include "sys.h"
#include "usart.h"	  
#include "stm32f10x.h"                  // Device header

////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif

/*-------------------------------------------------------------------------------
����ͨ��ʱ��
HC05����ģ��
��������APP���̱�д����ʻ״̬+0D(�س�)+0A�����У�
00��С��ֹͣ  01��С��ǰ��  02��С������  03��С����ת  04��С����ת
05��С����ʱ��  06��С��˳ʱ��  07��С����ת  08��С����ʱ��  09��С��˳ʱ��
10��ѭ����  11�����ϴ�
---------------------------------------------------------------------------------*/

#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART->SR&0X40)==0);//ѭ������,ֱ���������   
    USART->DR = (u8) ch;      
	return ch;
}
#endif 

/*ʹ��microLib�ķ���*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART, (uint8_t) ch);

	while (USART_GetFlagStatus(USART, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART->SR & USART_FLAG_RXNE));

    return ((int)(USART->DR & 0x1FF));
}
*/
 
#if EN_USART_RX   //���ʹ���˽���
//�����жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	  
  
void uart_init(u32 bound)
{
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	USART_ClockCmd(USART_CLK, ENABLE);	//ʹ��USARTʱ��
	RCC_APB2PeriphClockCmd(USART_GPIO_CLK, ENABLE);//ʹ��GPIOBʱ��
	
	//USART_TX
  GPIO_InitStructure.GPIO_Pin = USART_GPIO_PIN_TX;//��ʼ��USART_GPIO_PIN_TX
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(USART_GPIO_PORT, &GPIO_InitStructure);//��ʼ��GPIO_PIN_TX
   
  //USART_RX
  GPIO_InitStructure.GPIO_Pin = USART_GPIO_PIN_RX;//��ʼ��USART_GPIO_PIN_RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(USART_GPIO_PORT, &GPIO_InitStructure);//��ʼ��GPIO_PIN_RX

  //USART NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART_IRQn ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART, &USART_InitStructure); //��ʼ������
  
	USART_ITConfig(USART, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART, ENABLE);                    //ʹ�ܴ��� 

}

void USART_IRQHandler(void)                	//����1�жϷ������
	{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
		{
		Res =USART_ReceiveData(USART);	//��ȡ���յ�������
		
		if((USART_RX_STA&0x8000)==0)//����δ���
			{
			if(USART_RX_STA&0x4000)//���յ���0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else USART_RX_STA|=0x8000;	//��������� 
				}
			else //��û�յ�0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
					}		 
				}
			}   		 
     } 
#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntExit();  											 
#endif
} 
#endif	



