/*********************************************************************************************************************
 * CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 CYT4BB 开源库的一部分
 *
 * CYT4BB 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          main_cm7_0
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          IAR 9.40.1
 * 适用平台          CYT4BB
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2024-1-4       pudding            first version
 * 2024-1-4       Yibo Shu           second car
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************
// cw
#define M7_1_FLASH_SECTION_INDEX (0) // 存储 M7_1数据地址及长度 用的扇区
#define M7_1_FLASH_PAGE_INDEX (1)    // 存储 M7_1数据长度及长度 用的页码
volatile uint8 *m7_1_data;           // 定义一个数据指针 用于通过地址直接访问M7_1数据
uint32 m7_1_data_length;             // 定义M7_1数据的长度
//

#define DATA_LENGTH (1) // 数组数据长度

#pragma location = 0x28001000         // 将下面这个数组定义到指定的RAM地址，便于其他核心直接访问(开源库默认在0x28001000地址保留了8kb的空间用于数据交互)
__no_init uint8 m7_data[DATA_LENGTH]; // 定义M0演示数据数组 整数类型  由于该数组已经在M0核心赋值过初值，因此此处不再初始化

int16 encoder_data_dir[4] = {0}; // 编码器存储数组
int16 acc_speed1 = 0;            // 存储实际速度
int16 PWM_duty = 0;              // 存储PWM占空比
int16 count_100ms = 0;

// int16 Output_Val = 0;                  // 输出值
uint8 key_delay = 0, ips114_delay = 1; // 按键延时，屏幕刷新延时
// int16 goal_speed = 500;                // 目标速度

float acc_angle = 0;
float goal_angle = 0;
int16 smotor_duty = 0;
uint32 data_from_cm71 = 0; // 从M7_1接收到的数据
uint32 angle_from71[5] = {0};
uint8 data_en = 0;

unsigned char send[20];
// int16 speed_high = 850;
// int16 speed_low = 350;

void show_ips114screen(); // 显示屏幕函数
void show_ips200PAGE1();  // 显示屏幕函数
void show_ips200_page2(); // 显示屏幕函数
void show_ips200_test();  // 显示屏幕函数
void wireless_send();
// **************************** 主函数 *****************************

void my_ipc_callback(uint32 receive_data)
{
  data_from_cm71 = receive_data;
}

int main(void)
{
  clock_init(SYSTEM_CLOCK_250M); // 时钟配置及系统初始化<务必保留>
  debug_info_init();             // 调试串口信息初始化

  // 此处编写用户代码 例如外设初始化代码等

  SCB_DisableDCache(); // 关闭DCashe

  ipc_communicate_init(IPC_PORT_1, my_ipc_callback); // 初始化IPC模块 选择端口2 填写中断回调函数
                                                     //  ipc_communicate_init(IPC_PORT_2, my_ipc_callback2);  // 初始化IPC模块 选择端口2 填写中断回调函数

  Init_cm7_0(); // 初始化函数
  PID_Inc_Init(&whell_pid, 6, 0.2, 0);
  PID_Pos_Init(&servo_pid, 5, 0);
  /*
   system_delay_ms(500);                       // 等待M7_1将地址保存到flash  也可以使用查询flash数据的方式(此处不演示)

   flash_init();                  // flash初始化(访问flash之前需要初始化一次)
   flash_read_page_to_buffer(M7_1_FLASH_SECTION_INDEX, M7_1_FLASH_PAGE_INDEX, 2);  // 读取M7_1数据地址所在页的数据
   m7_1_data = (volatile uint8 *)flash_union_buffer[0].uint32_type;              // 保存地址和长度 使用指针直接访问
   m7_1_data_length = flash_union_buffer[1].uint32_type; */
  //
  // 此处编写用户代码 例如外设初始化代码等
  while (true)
  {
    speed_logic();
    key_proc();
    gnss_proc();                                                  // 运行并读取GPS数据
    SCB_CleanInvalidateDCache_by_Addr(&m7_data, sizeof(m7_data)); // M7_0核心有Dcache 当需要读取RAM地址数据时应该更新Dcache的内容 否则可能只是读取到Dcache而不是读取的RAM
    data_en = m7_data[0];
    // show_ips114screen();
    //  show_ips200angle_imu();
    // show_ips200_test();
    if (page == 0)
    {
      show_ips200PAGE1();
    }
    else if (page == 1)
    {
      show_ips200_page2();
    }
    Follow_Track(); // 计算方位角和距离
    overall_proc();
    remote_co(); // 遥控器控制
    //  wireless_send();
    // light_proc(); // 灯光判断
    //  pwm_set_duty(PWM_Servo, duty_servo); // 设置PWM占空比
    //  set_servo();
    // system_delay_ms(100);
    // cw
    /*   SCB_CleanInvalidateDCache();        // M7_0有Dcashe 读取数据之前需要更新DCache的数据
       printf("%.d,", m7_1_data[0]);     // 直接将数据打印到串口
   */
    //
    // 此处编写需要循环执行的代码
  }
}

void show_ips114screen()
{

  if (ips114_delay)
    return;
  ips114_delay = 1;

  ips114_show_string(0, 0, "acc_speed1:");
  ips114_show_int(100, 0, acc_speed1, 5);
  ips114_show_string(0, 20, "Output_Val:");
  ips114_show_int(100, 20, Output_Val, 5);
  ips114_show_string(0, 40, "goal_speed:");
  ips114_show_int(100, 40, goal_speed, 5);

  ips114_show_string(0, 60, "safe:");
  ips114_show_int(100, 60, safe, 5);

  ips114_show_string(0, 80, "Kp:");
  ips114_show_float(100, 80, whell_pid.Kp, 5, 2);

  ips114_show_string(0, 100, "Ki:");
  ips114_show_float(100, 100, whell_pid.Ki, 5, 2);
}

void show_ips200PAGE1()
{
  if (ips114_delay)
    return;
  ips114_delay = 1;
  // tft180_show_string(0, 0, "acc_speed1:");
  // tft180_show_int(100, 0, acc_speed1, 5);
  // tft180_show_string(0, 20, "SpeedErr:");
  // tft180_show_int(100, 20, SpeedErr, 5);
  // tft180_show_string(0, 40, "Output_Val:");
  // tft180_show_int(100, 40, Output_Val, 5);
  // tft180_show_string(0, 60, "goal_speed:");
  // tft180_show_int(100, 60, goal_speed, 5);
  // tft180_show_string(0, 80, "gyro_z:");
  // tft180_show_int(100, 80, imu963ra_gyro_z, 5);
  // tft180_show_string(0, 100, "Ki:");
  // tft180_show_float(100, 100, Ki, 5, 2);

  ips200_show_string(0, 0, "cm71:");              //
  ips200_show_uint(32, 9 * 0, data_from_cm71, 5); // 显示cm71数据
  ips200_show_string(150, 0, "light:");           //
  ips200_show_uint(200, 9 * 0, light_num, 5);     // 显示cm71数据

  ips200_show_float(0, 9 * 1, gnss.longitude, 4, 6); // 显示实时GPS经度

  ips200_show_float(150, 9 * 1, gnss.latitude, 4, 6); // 显示实时GPS经度

  ips200_show_string(0, 9 * 2, "di:");
  ips200_show_float(22, 9 * 2, gnss.direction, 4, 6); // 显示方向角

  ips200_show_string(150, 9 * 2, "n::");
  ips200_show_uint(180, 9 * 2, gnss.satellite_used, 5); // 显示方向角

  ips200_show_string(0, 9 * 3, "ds");
  ips200_show_float(22, 9 * 3, distance, 4, 6); // 显示方向角

  ips200_show_string(0, 9 * 4, "Kp:");
  ips200_show_float(30, 9 * 4, kp_back, 5, 2);

  ips200_show_string(0, 9 * 5, "Kd:");
  ips200_show_float(30, 9 * 5, kd_back, 5, 2);

  // acc_speed1

  ips200_show_string(0, 9 * 6, "spe:");
  ips200_show_float(30, 9 * 6, acc_speed1, 5, 2);
  
    ips200_show_string(0, 9 * 7, "sec:");
  ips200_show_uint(30, 9 * 7, selct, 5);

  ips200_show_string(0, 9 * 12, "z:");                 // 显示z轴角度
  ips200_show_float(30, 9 * 12, eulerAngle_yaw, 3, 6); // 显示z轴角度

  ips200_show_string(0, 9 * 13, "azi:");                             // 显示方向角
  ips200_show_float(30, 9 * 13, azimuth_new, 3, 6);                  // 显示方向角
  ips200_show_string(0, 9 * 14, "ans:");                             // 显示方向角
  ips200_show_float(30, 9 * 14, azimuth_new - eulerAngle_yaw, 3, 6); // 显示方向角

  ips200_show_uint(0, 9 * 15, gnss_IMU_flag, 5);       // 显示方向角
  ips200_show_uint(22, 9 * 15, gnss_dirction_flag, 5); // 显示方向角
  ips200_show_uint(44, 9 * 15, voice_flag, 5);         // 显示方向角

  ips200_show_int(0, 9 * 17, safe, 5); // 显示方向角

  ips200_show_string(0, 9 * 18, "SH:");       // 显示方向角
  ips200_show_int(50, 9 * 18, speed_high, 5); // 显示方向角

  ips200_show_string(0, 9 * 19, "SL:");      // 显示方向角
  ips200_show_int(50, 9 * 19, speed_low, 5); // 显示方向角

  // ips200_show_float(50, 9 * 20, data_mid, 8, 2); // 显示方向角
  ips200_show_string(0, 9 * 20, "dis_f:");
  ips200_show_float(50, 9 * 20, dis_flag, 1, 1);

  ips200_show_string(0, 9 * 21, "[9]");
  ips200_show_float(30, 9 * 21, GPS_GET_LAT[9], 4, 6);
  ips200_show_float(150, 9 * 21, GPS_GET_LOT[9], 4, 6);

  ips200_show_string(0, 9 * 22, "[10]");
  ips200_show_float(30, 9 * 22, GPS_GET_LAT[10], 4, 6);
  ips200_show_float(150, 9 * 22, GPS_GET_LOT[10], 4, 6);

  ips200_show_string(0, 9 * 23, "en");
  ips200_show_int(30, 9 * 23, m7_data[0], 5);
  ips200_show_string(0, 9 * 25, "spe");
  ips200_show_int(30, 9 * 25, goal_speed, 5);

  // show_ips200angle_gnss();

  // ips200_show_string(0, 9*9, "servo:");    //显示舵机占空比
  // ips200_show_uint(40, 9 * 9,  , 4);  //显示舵机占空比
}

void show_ips200_page2()
{
  if (ips114_delay)
    return;
  ips114_delay = 1;
  ips200_show_int(0, 0, light_sec[0], 1);
  ips200_show_int(30, 0, light_sec[1], 1);
  ips200_show_int(60, 0, light_sec[2], 1);
  ips200_show_int(90, 0, light_sec[3], 1);
  ips200_show_int(120, 0, light_sec[4], 1);
  ips200_show_int(150, 0, light_sec[5], 1);
  ips200_show_int(180, 0, light_sec[6], 1);
  ips200_show_int(210, 0, light_sec[7], 1);

  ips200_show_string(0, 10, "Kp:");
  ips200_show_float(100, 10, servo_pid.Kp, 5, 2);

  ips200_show_string(0, 20, "Kd:");
  ips200_show_float(100, 20, servo_pid.Kd, 5, 2);
}
void show_ips200_test()
{
  if (ips114_delay)
    return;
  ips114_delay = 1;

  ips200_show_string(0, 0, "acc_speed1:");
  ips200_show_int(100, 0, acc_speed1, 5);
  ips200_show_string(0, 10, "Output_Val:");
  ips200_show_int(100, 10, Output_Val, 5);
  ips200_show_string(0, 20, "goal_speed:");
  ips200_show_int(100, 20, goal_speed, 5);

  ips200_show_string(0, 30, "safe:");
  ips200_show_int(100, 30, safe, 5);

  ips200_show_string(0, 40, "Kp:");
  ips200_show_float(100, 40, whell_pid.Kp, 5, 2);

  ips200_show_string(0, 50, "Ki:");
  ips200_show_float(100, 50, whell_pid.Ki, 5, 2);

  ips200_show_string(0, 60, "servo:");
  ips200_show_int(100, 60, smotor_duty, 5);

  ips200_show_string(0, 80, "DUTY_Z:");
  ips200_show_float(100, 80, eulerAngle_yaw, 5, 5);
  show_ips200angle_imu();
}

void wireless_send()
{
  if (count_100ms)
    return;
  count_100ms = 1;

  sprintf(send, "$%d,%d\n", (unsigned int)(acc_angle - goal_angle), (unsigned int)distance);
  wireless_uart_send_buffer(send, 20);
}
// **************************** PIT中断函数 ****************************
void pit0_ch2_isr()
{
  pit_isr_flag_clear(PIT_CH2); // 清除中断标志位

  // get_IMU_data();
  // 每5毫秒读取一次编码器数值
  if (++key_delay >= 2) // 按键延时
  {
    key_delay = 0;
  }
  if (++ips114_delay >= 20) // 屏幕刷新延时
  {
    ips114_delay = 0;
    // uart_rx433_send();
  }
  if (++count_100ms >= 40)
  {
    count_100ms = 0;
  }

  /*PID控制代码*/
  acc_speed1 = accu_speed(); // 读取编码器速度
  if (safe == 1 || (safe == 2 && GPS_GET_LAT[1] == 0))
  {
    acc_angle = eulerAngle_yaw; // 读取IMU角度
    goal_angle = data_from_cm71;
    if ((angle_error > 0 && angle_error < 90) || (angle_error > 270 && angle_error < 360) || selct == 1)
    {
      servo_pid.Kp = kp_forward;
      servo_pid.Kd = kd_forward;
      smotor_duty = (int16)PID_Pos(0, data_from_cm71, &servo_pid);
    }
    else
    {
      servo_pid.Kp = kp_back;
      servo_pid.Kd = kd_back;
      //  servo_pid.Kd = 2;
      smotor_duty = -(int16)PID_Pos(180, data_from_cm71, &servo_pid);
      //  smotor_duty=-smotor_duty;
    }
  }
  else if (safe == 2)
  {
    if (light_num == light_sec[0] || light_num == light_sec[1] || light_num == light_sec[2] || light_num == light_sec[3] || light_num == light_sec[4] || light_num == light_sec[5] || light_num == light_sec[6] || light_num == light_sec[7])
    {
      if (gnss_IMU_flag)
      {
        if (IMU_flag) // 陀螺仪控制反向
        {
          servo_pid.Kp = 5;
          acc_angle = eulerAngle_yaw; // 读取IMU角度
          goal_angle = azimuth_new;
          //   distance_count+=acc_speed1;

          smotor_duty = (int16)PID_Pos(goal_angle, acc_angle, &servo_pid);
        }

        if (gnss_dirction_flag) // GNSS方向控制
        {
          servo_pid.Kp = 1.5;
          servo_pid.Kd = 0;
          acc_angle = gnss.direction; // 车的方向角
          goal_angle = azimuth_new;
          smotor_duty = (int16)PID_Pos(goal_angle, acc_angle, &servo_pid);
        }
      }
      // if (gnss_dirction_flag)
      // {
      //   acc_angle = gnss.direction; // 车的方向角
      //   goal_angle = azimuth_new;
      //   smotor_duty = (int16)PID_Pos(goal_angle, acc_angle, &servo_pid);
      // }

      if (voice_flag)
      {
        servo_pid.Kp = 1.8;
        acc_angle = eulerAngle_yaw; // 读取IMU角度
        goal_angle = data_from_cm71;
        smotor_duty = (int16)PID_Pos(0, data_from_cm71, &servo_pid);
      }
    }
    else
    {
      if (gnss_IMU_flag)
      {
        if (IMU_flag) // 陀螺仪控制反向
        {
          servo_pid.Kp = 5;
          acc_angle = eulerAngle_yaw; // 读取IMU角度
          goal_angle = azimuth_new;
          //   distance_count+=acc_speed1;

          smotor_duty = (int16)PID_Pos(goal_angle, acc_angle, &servo_pid);
        }

        if (gnss_dirction_flag) // GNSS方向控制
        {
          servo_pid.Kp = 1.5;
          acc_angle = gnss.direction; // 车的方向角
          goal_angle = azimuth_new;
          smotor_duty = (int16)PID_Pos(goal_angle, acc_angle, &servo_pid);
        }
      }
    }
  }
  if(light_num!=light_old)
  {
    light_old=light_num;
    Output_Val=0;
   // turn_flag=1;
  }
  Output_Val += PID_Inc(goal_speed, acc_speed1, &whell_pid); // PID控制
  smotor_duty = (int16)limit(smotor_duty, 140);
  duty_servo = 700 - smotor_duty;
  if (safe)
  {
    pwm_set_duty(PWM_Servo, duty_servo); // 设置PWM占空比
  }

  //  printf("zhiguoxin:%d, %d\n", acc_speed1, Output_Val); // 打印速度
  motor_control(Output_Val); // 电机控制
}
