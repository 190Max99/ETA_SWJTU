#include "init.h"
void Init_cm7_0(void)
{
  gpio_init(KEY1, GPO, GPIO_HIGH, GPI_PULL_UP); // ��ʼ�� KEY1 ���� Ĭ�ϸߵ�ƽ ��������
  gpio_init(KEY2, GPO, GPIO_HIGH, GPI_PULL_UP); // ��ʼ�� KEY2 ���� Ĭ�ϸߵ�ƽ ��������
  gpio_init(KEY3, GPO, GPIO_HIGH, GPI_PULL_UP); // ��ʼ�� KEY3 ���� Ĭ�ϸߵ�ƽ ��������
  gpio_init(KEY4, GPO, GPIO_HIGH, GPI_PULL_UP); // ��ʼ�� KEY4 ���� Ĭ�ϸߵ�ƽ ��������
  gpio_init(KEY5, GPI, GPIO_HIGH, GPI_PULL_UP); // ��ʼ�� KEY1 ���� Ĭ�ϸߵ�ƽ ��������
  gpio_init(KEY6, GPI, GPIO_HIGH, GPI_PULL_UP); // ��ʼ�� KEY2 ���� Ĭ�ϸߵ�ƽ ��������
  gpio_init(KEY7, GPI, GPIO_HIGH, GPI_PULL_UP); // ��ʼ�� KEY3 ���� Ĭ�ϸߵ�ƽ ��������
  gpio_init(KEY8, GPI, GPIO_HIGH, GPI_PULL_UP); // ��ʼ�� KEY4 ���� Ĭ�ϸߵ�ƽ ��������

  encoder_dir_init(ENCODER_DIR1, ENCODER_DIR_PULSE1, ENCODER_DIR_DIR1); // ��ʼ��������ģ�������� ����������������ģʽ

  pwm_init(PWM_CH3, 17000, 0); // ��ʼ�� PWM ͨ�� Ƶ�� 17KHz ��ʼռ�ձ� 0%
  pwm_init(PWM_CH4, 17000, 0);
  pwm_init(PWM_Servo, 50, 700); // ��ʼ�� PWM ͨ�� Ƶ�� 50Hz ��ʼռ�ձ� 700us

 gnss_init(TAU1201); // ��ʼ�� GNSS ģ��

  //  ips114_init(); // ��ʼ�� IPS1.14��Ļ

  ips200_init(IPS200_TYPE_SPI); // ��ʼ�� TFT1.8��Ļ

  flash_init(); // flash��ʼ��(����flash֮ǰ��Ҫ��ʼ��һ��)

  //  uart_init(UART_0, 115200, UART0_TX_P00_1, UART0_RX_P00_0); // ����0��ʼ����Ӧ��¼��

  uart_receiver_init(); // ң�������ڳ�ʼ��

  uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN); // ��ʼ������4
  uart_rx_interrupt(UART_INDEX, 1);                               // ���� UART_INDEX �Ľ����ж�
// gyroOffset_init();                                              // ��������Ʈ��ʼ��

  //  if(wireless_uart_init())
  // {
  //   while(1)
  //  {
  //  }
  //  }

   wireless_uart_send_string("seekfree. \r\n");
 // pit_ms_init(PIT_CH1, 2); // ��ʼ�� PIT_CH1 Ϊ�����ж� 2ms
  pit_ms_init(PIT_CH2, 5); // ��ʼ�� PIT_CH0 Ϊ�����ж� 5ms
}