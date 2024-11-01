#include "zf_common_headfile.h"

uint8 gnss_IMU_flag = 0;
uint8 IMU_flag = 0;
uint8 gnss_dirction_flag = 0;
uint8 voice_flag = 0;
int16 distance_count = 0;
// uint8 light_old = 0;
float dis_flag = 4.5;
uint8 en_flg = 0;

void overall_proc()
{
   if (distance > dis_flag) //&& //en_flg == 0)
   {
      // light_old = light_num;
      //  distance_count = 0;
      gnss_IMU_flag = 1;
      //  gnss_dirction_flag = 0;
      voice_flag = 0;
   }

   // if (distance > 1 && distance_count > 40000)
   // {
   //    gnss_IMU_flag = 0;
   //    gnss_dirction_flag = 1;
   //    voice_flag = 0;
   // }

   if (distance <= dis_flag)
   {
      gnss_IMU_flag = 0;
      IMU_flag = 0;
      gnss_dirction_flag = 0;
      voice_flag = 1;
      en_flg = 1;
   }
}
