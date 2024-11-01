/*********************************************************************************************************************
 * CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
 * Copyright (c) 2022 SEEKFREE ��ɿƼ�
 *
 * ���ļ��� CYT4BB ��Դ���һ����
 *
 * CYT4BB ��Դ�� ��������
 * �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
 * �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
 *
 * ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
 * ����û�������������Ի��ʺ��ض���;�ı�֤
 * ����ϸ����μ� GPL
 *
 * ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
 * ���û�У������<https://www.gnu.org/licenses/>
 *
 * ����ע����
 * ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
 * �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
 * ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
 * ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
 *
 * �ļ�����          main_cm7_0
 * ��˾����          �ɶ���ɿƼ����޹�˾
 * �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
 * ��������          IAR 9.40.1
 * ����ƽ̨          CYT4BB
 * ��������          https://seekfree.taobao.com/
 *
 * �޸ļ�¼
 * ����              ����                ��ע
 * 2024-1-4       pudding            first version
 * 2024-1-4       Yibo Shu           second car
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������

// **************************** �������� ****************************
// cw
#define M7_1_FLASH_SECTION_INDEX (0) // �洢 M7_1���ݵ�ַ������ �õ�����
#define M7_1_FLASH_PAGE_INDEX (1)    // �洢 M7_1���ݳ��ȼ����� �õ�ҳ��
volatile uint8 *m7_1_data;           // ����һ������ָ�� ����ͨ����ֱַ�ӷ���M7_1����
uint32 m7_1_data_length;             // ����M7_1���ݵĳ���
//

#define DATA_LENGTH (1) // �������ݳ���

#pragma location = 0x28001000         // ������������鶨�嵽ָ����RAM��ַ��������������ֱ�ӷ���(��Դ��Ĭ����0x28001000��ַ������8kb�Ŀռ��������ݽ���)
__no_init uint8 m7_data[DATA_LENGTH]; // ����M0��ʾ�������� ��������  ���ڸ������Ѿ���M0���ĸ�ֵ����ֵ����˴˴����ٳ�ʼ��

int16 encoder_data_dir[4] = {0}; // �������洢����
int16 acc_speed1 = 0;            // �洢ʵ���ٶ�
int16 PWM_duty = 0;              // �洢PWMռ�ձ�
int16 count_100ms = 0;

// int16 Output_Val = 0;                  // ���ֵ
uint8 key_delay = 0, ips114_delay = 1; // ������ʱ����Ļˢ����ʱ
// int16 goal_speed = 500;                // Ŀ���ٶ�

float acc_angle = 0;
float goal_angle = 0;
int16 smotor_duty = 0;
uint32 data_from_cm71 = 0; // ��M7_1���յ�������
uint32 angle_from71[5] = {0};
uint8 data_en = 0;

unsigned char send[20];
// int16 speed_high = 850;
// int16 speed_low = 350;

void show_ips114screen(); // ��ʾ��Ļ����
void show_ips200PAGE1();  // ��ʾ��Ļ����
void show_ips200_page2(); // ��ʾ��Ļ����
void show_ips200_test();  // ��ʾ��Ļ����
void wireless_send();
// **************************** ������ *****************************

void my_ipc_callback(uint32 receive_data)
{
  data_from_cm71 = receive_data;
}

int main(void)
{
  clock_init(SYSTEM_CLOCK_250M); // ʱ�����ü�ϵͳ��ʼ��<��ر���>
  debug_info_init();             // ���Դ�����Ϣ��ʼ��

  // �˴���д�û����� ���������ʼ�������

  SCB_DisableDCache(); // �ر�DCashe

  ipc_communicate_init(IPC_PORT_1, my_ipc_callback); // ��ʼ��IPCģ�� ѡ��˿�2 ��д�жϻص�����
                                                     //  ipc_communicate_init(IPC_PORT_2, my_ipc_callback2);  // ��ʼ��IPCģ�� ѡ��˿�2 ��д�жϻص�����

  Init_cm7_0(); // ��ʼ������
  PID_Inc_Init(&whell_pid, 6, 0.2, 0);
  PID_Pos_Init(&servo_pid, 5, 0);
  /*
   system_delay_ms(500);                       // �ȴ�M7_1����ַ���浽flash  Ҳ����ʹ�ò�ѯflash���ݵķ�ʽ(�˴�����ʾ)

   flash_init();                  // flash��ʼ��(����flash֮ǰ��Ҫ��ʼ��һ��)
   flash_read_page_to_buffer(M7_1_FLASH_SECTION_INDEX, M7_1_FLASH_PAGE_INDEX, 2);  // ��ȡM7_1���ݵ�ַ����ҳ������
   m7_1_data = (volatile uint8 *)flash_union_buffer[0].uint32_type;              // �����ַ�ͳ��� ʹ��ָ��ֱ�ӷ���
   m7_1_data_length = flash_union_buffer[1].uint32_type; */
  //
  // �˴���д�û����� ���������ʼ�������
  while (true)
  {
    speed_logic();
    key_proc();
    gnss_proc();                                                  // ���в���ȡGPS����
    SCB_CleanInvalidateDCache_by_Addr(&m7_data, sizeof(m7_data)); // M7_0������Dcache ����Ҫ��ȡRAM��ַ����ʱӦ�ø���Dcache������ �������ֻ�Ƕ�ȡ��Dcache�����Ƕ�ȡ��RAM
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
    Follow_Track(); // ���㷽λ�Ǻ;���
    overall_proc();
    remote_co(); // ң��������
    //  wireless_send();
    // light_proc(); // �ƹ��ж�
    //  pwm_set_duty(PWM_Servo, duty_servo); // ����PWMռ�ձ�
    //  set_servo();
    // system_delay_ms(100);
    // cw
    /*   SCB_CleanInvalidateDCache();        // M7_0��Dcashe ��ȡ����֮ǰ��Ҫ����DCache������
       printf("%.d,", m7_1_data[0]);     // ֱ�ӽ����ݴ�ӡ������
   */
    //
    // �˴���д��Ҫѭ��ִ�еĴ���
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
  ips200_show_uint(32, 9 * 0, data_from_cm71, 5); // ��ʾcm71����
  ips200_show_string(150, 0, "light:");           //
  ips200_show_uint(200, 9 * 0, light_num, 5);     // ��ʾcm71����

  ips200_show_float(0, 9 * 1, gnss.longitude, 4, 6); // ��ʾʵʱGPS����

  ips200_show_float(150, 9 * 1, gnss.latitude, 4, 6); // ��ʾʵʱGPS����

  ips200_show_string(0, 9 * 2, "di:");
  ips200_show_float(22, 9 * 2, gnss.direction, 4, 6); // ��ʾ�����

  ips200_show_string(150, 9 * 2, "n::");
  ips200_show_uint(180, 9 * 2, gnss.satellite_used, 5); // ��ʾ�����

  ips200_show_string(0, 9 * 3, "ds");
  ips200_show_float(22, 9 * 3, distance, 4, 6); // ��ʾ�����

  ips200_show_string(0, 9 * 4, "Kp:");
  ips200_show_float(30, 9 * 4, kp_back, 5, 2);

  ips200_show_string(0, 9 * 5, "Kd:");
  ips200_show_float(30, 9 * 5, kd_back, 5, 2);

  // acc_speed1

  ips200_show_string(0, 9 * 6, "spe:");
  ips200_show_float(30, 9 * 6, acc_speed1, 5, 2);
  
    ips200_show_string(0, 9 * 7, "sec:");
  ips200_show_uint(30, 9 * 7, selct, 5);

  ips200_show_string(0, 9 * 12, "z:");                 // ��ʾz��Ƕ�
  ips200_show_float(30, 9 * 12, eulerAngle_yaw, 3, 6); // ��ʾz��Ƕ�

  ips200_show_string(0, 9 * 13, "azi:");                             // ��ʾ�����
  ips200_show_float(30, 9 * 13, azimuth_new, 3, 6);                  // ��ʾ�����
  ips200_show_string(0, 9 * 14, "ans:");                             // ��ʾ�����
  ips200_show_float(30, 9 * 14, azimuth_new - eulerAngle_yaw, 3, 6); // ��ʾ�����

  ips200_show_uint(0, 9 * 15, gnss_IMU_flag, 5);       // ��ʾ�����
  ips200_show_uint(22, 9 * 15, gnss_dirction_flag, 5); // ��ʾ�����
  ips200_show_uint(44, 9 * 15, voice_flag, 5);         // ��ʾ�����

  ips200_show_int(0, 9 * 17, safe, 5); // ��ʾ�����

  ips200_show_string(0, 9 * 18, "SH:");       // ��ʾ�����
  ips200_show_int(50, 9 * 18, speed_high, 5); // ��ʾ�����

  ips200_show_string(0, 9 * 19, "SL:");      // ��ʾ�����
  ips200_show_int(50, 9 * 19, speed_low, 5); // ��ʾ�����

  // ips200_show_float(50, 9 * 20, data_mid, 8, 2); // ��ʾ�����
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

  // ips200_show_string(0, 9*9, "servo:");    //��ʾ���ռ�ձ�
  // ips200_show_uint(40, 9 * 9,  , 4);  //��ʾ���ռ�ձ�
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
// **************************** PIT�жϺ��� ****************************
void pit0_ch2_isr()
{
  pit_isr_flag_clear(PIT_CH2); // ����жϱ�־λ

  // get_IMU_data();
  // ÿ5�����ȡһ�α�������ֵ
  if (++key_delay >= 2) // ������ʱ
  {
    key_delay = 0;
  }
  if (++ips114_delay >= 20) // ��Ļˢ����ʱ
  {
    ips114_delay = 0;
    // uart_rx433_send();
  }
  if (++count_100ms >= 40)
  {
    count_100ms = 0;
  }

  /*PID���ƴ���*/
  acc_speed1 = accu_speed(); // ��ȡ�������ٶ�
  if (safe == 1 || (safe == 2 && GPS_GET_LAT[1] == 0))
  {
    acc_angle = eulerAngle_yaw; // ��ȡIMU�Ƕ�
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
        if (IMU_flag) // �����ǿ��Ʒ���
        {
          servo_pid.Kp = 5;
          acc_angle = eulerAngle_yaw; // ��ȡIMU�Ƕ�
          goal_angle = azimuth_new;
          //   distance_count+=acc_speed1;

          smotor_duty = (int16)PID_Pos(goal_angle, acc_angle, &servo_pid);
        }

        if (gnss_dirction_flag) // GNSS�������
        {
          servo_pid.Kp = 1.5;
          servo_pid.Kd = 0;
          acc_angle = gnss.direction; // ���ķ����
          goal_angle = azimuth_new;
          smotor_duty = (int16)PID_Pos(goal_angle, acc_angle, &servo_pid);
        }
      }
      // if (gnss_dirction_flag)
      // {
      //   acc_angle = gnss.direction; // ���ķ����
      //   goal_angle = azimuth_new;
      //   smotor_duty = (int16)PID_Pos(goal_angle, acc_angle, &servo_pid);
      // }

      if (voice_flag)
      {
        servo_pid.Kp = 1.8;
        acc_angle = eulerAngle_yaw; // ��ȡIMU�Ƕ�
        goal_angle = data_from_cm71;
        smotor_duty = (int16)PID_Pos(0, data_from_cm71, &servo_pid);
      }
    }
    else
    {
      if (gnss_IMU_flag)
      {
        if (IMU_flag) // �����ǿ��Ʒ���
        {
          servo_pid.Kp = 5;
          acc_angle = eulerAngle_yaw; // ��ȡIMU�Ƕ�
          goal_angle = azimuth_new;
          //   distance_count+=acc_speed1;

          smotor_duty = (int16)PID_Pos(goal_angle, acc_angle, &servo_pid);
        }

        if (gnss_dirction_flag) // GNSS�������
        {
          servo_pid.Kp = 1.5;
          acc_angle = gnss.direction; // ���ķ����
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
  Output_Val += PID_Inc(goal_speed, acc_speed1, &whell_pid); // PID����
  smotor_duty = (int16)limit(smotor_duty, 140);
  duty_servo = 700 - smotor_duty;
  if (safe)
  {
    pwm_set_duty(PWM_Servo, duty_servo); // ����PWMռ�ձ�
  }

  //  printf("zhiguoxin:%d, %d\n", acc_speed1, Output_Val); // ��ӡ�ٶ�
  motor_control(Output_Val); // �������
}
