#include "init.h"
void Init_cm7_0(void)
{
  gpio_init(KEY1, GPO, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY1 输入 默认高电平 上拉输入
  gpio_init(KEY2, GPO, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY2 输入 默认高电平 上拉输入
  gpio_init(KEY3, GPO, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY3 输入 默认高电平 上拉输入
  gpio_init(KEY4, GPO, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY4 输入 默认高电平 上拉输入
  gpio_init(KEY5, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY1 输入 默认高电平 上拉输入
  gpio_init(KEY6, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY2 输入 默认高电平 上拉输入
  gpio_init(KEY7, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY3 输入 默认高电平 上拉输入
  gpio_init(KEY8, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY4 输入 默认高电平 上拉输入

  encoder_dir_init(ENCODER_DIR1, ENCODER_DIR_PULSE1, ENCODER_DIR_DIR1); // 初始化编码器模块与引脚 带方向增量编码器模式

  pwm_init(PWM_CH3, 17000, 0); // 初始化 PWM 通道 频率 17KHz 初始占空比 0%
  pwm_init(PWM_CH4, 17000, 0);
  pwm_init(PWM_Servo, 50, 700); // 初始化 PWM 通道 频率 50Hz 初始占空比 700us

 gnss_init(TAU1201); // 初始化 GNSS 模块

  //  ips114_init(); // 初始化 IPS1.14屏幕

  ips200_init(IPS200_TYPE_SPI); // 初始化 TFT1.8屏幕

  flash_init(); // flash初始化(访问flash之前需要初始化一次)

  //  uart_init(UART_0, 115200, UART0_TX_P00_1, UART0_RX_P00_0); // 串口0初始化对应烧录线

  uart_receiver_init(); // 遥控器串口初始化

  uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN); // 初始化串口4
  uart_rx_interrupt(UART_INDEX, 1);                               // 开启 UART_INDEX 的接收中断
// gyroOffset_init();                                              // 陀螺仪零飘初始化

  //  if(wireless_uart_init())
  // {
  //   while(1)
  //  {
  //  }
  //  }

   wireless_uart_send_string("seekfree. \r\n");
 // pit_ms_init(PIT_CH1, 2); // 初始化 PIT_CH1 为周期中断 2ms
  pit_ms_init(PIT_CH2, 5); // 初始化 PIT_CH0 为周期中断 5ms
}