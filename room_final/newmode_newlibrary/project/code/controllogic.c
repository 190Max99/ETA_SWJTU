#include "zf_common_headfile.h"

int16 Output_Val = 0;   // 输出值
int16 goal_speed = 500; // 目标速度
int16 speed_high = 500;
int16 speed_low = 350;
uint8 flag_mid = 0;
uint8 turn_flag = 0;
double distance_test = 6;
double angle_error = 0;
float disance_speed_reduce = 7;
uint8 selct = 0;
extern uint8 data_en;

uint8 empty_flag = 0;
uint8 empty_en = 1;

void speed_logic()
{
    if (safe == 0)
    {
        goal_speed = 0;
        Output_Val = 0;
    }
    else if ((safe == 1 && (light_num == light_sec[0] || light_num == light_sec[1] || light_num == light_sec[2] || light_num == light_sec[3] || light_num == light_sec[4] || light_num == light_sec[5] || light_num == light_sec[6] || light_num == light_sec[7])) || (safe == 2 && GPS_GET_LAT[1] == 0 && light_num > 0 && (light_num == light_sec[0] || light_num == light_sec[1] || light_num == light_sec[2] || light_num == light_sec[3] || light_num == light_sec[4] || light_num == light_sec[5] || light_num == light_sec[6] || light_num == light_sec[7])))
    {
        if (selct == 1)
        {
             angle_error = goal_angle;
            if (angle_error > 360)
            {
                angle_error -= 360;
            }
            if (angle_error < -360)
            {
                angle_error += 360;
            }
            if ((angle_error > 340) || (angle_error < 20)) // 陀螺仪方向与目标方向相差小于20度，切换为GNSS方向
            {
                goal_speed = speed_high;
            }
            else
            {
                goal_speed = speed_low;
            }
        }
        else if (selct == 0)
        {
            angle_error = goal_angle;
            if (angle_error > 360)
            {
                angle_error -= 360;
            }
            if (angle_error < -360)
            {
                angle_error += 360;
            }

            if ((angle_error > 0 && angle_error < 90) || (angle_error > 270 && angle_error < 360))
            {
                 if ((angle_error > 340) || (angle_error < 20)) // 陀螺仪方向与目标方向相差小于20度，切换为GNSS方向
                {
                  kp_forward = 0.5;
                    kd_forward = 0;
                    goal_speed = speed_high;
                }
                else
                {
                  kp_forward = 2;
                    kd_forward = 0;
                    goal_speed = speed_low;
                }
            }

            if (angle_error > 90 && angle_error < 270)
            {
                if ((angle_error > 160) && (angle_error < 200)) // 陀螺仪方向与目标方向相差小于20度，切换为GNSS方向
                {
                    kp_back = 0.6;
                    kd_back = 0;
                    
                    if ((angle_error > 170) && (angle_error < 190)) // 陀螺仪方向与目标方向相差小于20度，切换为GNSS方向
                {
                   kp_back = 0.3;
                    kd_back = 0;
                }
                    goal_speed = -speed_high;
                }
                else
                {
                    kp_back = 2;
                    kd_back = 1;
                    goal_speed = -speed_low;
                }
            }
        }
    }
    else if (safe == 2 && (light_num == light_sec[0] || light_num == light_sec[1] || light_num == light_sec[2] || light_num == light_sec[3] || light_num == light_sec[4] || light_num == light_sec[5] || light_num == light_sec[6] || light_num == light_sec[7]) && GPS_GET_LAT[1] != 0)
    {
        angle_error = azimuth_new - eulerAngle_yaw;
        if (angle_error > 360)
        {
            angle_error -= 360;
        }
        if (angle_error < -360)
        {
            angle_error += 360;
        }
        // 默认对北发车
        if ((angle_error > -15) && (angle_error < 15) && turn_flag == 1) // 陀螺仪方向与目标方向相差小于10度，切换为GNSS方向
        {
            turn_flag = 0;
            if (gnss_IMU_flag)
            {
                gnss_dirction_flag = 1;
                IMU_flag = 0;
            }
        }
        if (light_num != light_old && turn_flag == 0)
        {
            light_old = light_num;
            en_flg = 0;
            turn_flag = 1;
        }
        if (turn_flag) // 急转弯，用陀螺仪判断方向
        {
            goal_speed = 600;
            if (gnss_IMU_flag)
            {
                gnss_dirction_flag = 0;
                IMU_flag = 1;
            }
        }
        else if (turn_flag == 0)
        {
            empty_en = 1;
            if (distance >= disance_speed_reduce)
            {
                whell_pid.Kp = 7.5;
                whell_pid.Ki = 0.15;
                goal_speed = speed_high;
            }
            if (distance < disance_speed_reduce && distance > dis_flag)
            {
                goal_speed = distance * (speed_high - speed_low) / (disance_speed_reduce - dis_flag) + speed_high - disance_speed_reduce * (speed_high - speed_low) / (disance_speed_reduce - dis_flag);
                whell_pid.Kp = 5;
                whell_pid.Ki = 0.25;
            }
            else if (voice_flag)
            {
                angle_error = goal_angle;
                if (angle_error > 360)
                {
                    angle_error -= 360;
                }
                if (angle_error < -360)
                {
                    angle_error += 360;
                }
                if ((angle_error > 340) || (angle_error < 20)) // 陀螺仪方向与目标方向相差小于10度，切换为GNSS方向
                {
                    goal_speed = speed_low + 400;
                }
                else
                {
                    goal_speed = speed_low;
                }
            }
            else if (voice_flag)
            {
                angle_error = goal_angle;
                if (angle_error > 360)
                {
                    angle_error -= 360;
                }
                if (angle_error < -360)
                {
                    angle_error += 360;
                }
                if ((angle_error > 340) || (angle_error < 20)) // 陀螺仪方向与目标方向相差小于10度，切换为GNSS方向
                {
                    goal_speed = speed_low + 400;
                }
                else
                {
                    goal_speed = speed_low;
                }
            }
        }
        else if (voice_flag)
        {
            goal_speed = speed_low;
        }
    }
    else if (safe == 2 && light_num > 0 && GPS_GET_LAT[1] != 0)

    {
        angle_error = azimuth_new - eulerAngle_yaw;
        if (angle_error > 360)
        {
            angle_error -= 360;
        }
        if (angle_error < -360)
        {
            angle_error += 360;
        }
        // 默认对北发车
        if ((angle_error > -10) && (angle_error < 10) && turn_flag == 1) // 陀螺仪方向与目标方向相差小于10度，切换为GNSS方向
        {
            turn_flag = 0;
            if (gnss_IMU_flag)
            {
                gnss_dirction_flag = 1;
                IMU_flag = 0;
            }
        }
        if (light_num != light_old && turn_flag == 0)
        {
            light_old = light_num;
            en_flg = 0;
            turn_flag = 1;
        }
        if (turn_flag) // 急转弯，用陀螺仪判断方向
        {
            if (empty_en)
            {
                goal_speed = 600;
            }
            if (gnss_IMU_flag)
            {
                gnss_dirction_flag = 0;
                IMU_flag = 1;
            }
        }
        else if (turn_flag == 0)
        {
            if (distance > 3.5 && empty_en == 1)
            {
                empty_flag = 1;
                empty_en = 0;
                light_old = light_num;
            }
            if (distance < 3.5 && empty_flag)
            {
                empty_flag = 0;
                //  empty_en = 1;
            }

            if (empty_flag == 1 && empty_en == 0)
            {
                if (distance >= disance_speed_reduce)
                {
                    goal_speed = speed_high;
                }
                if (distance < disance_speed_reduce && distance > dis_flag)
                {
                    goal_speed = distance * (speed_high - speed_low) / (disance_speed_reduce - dis_flag) + speed_high - disance_speed_reduce * (speed_high - speed_low) / (disance_speed_reduce - dis_flag);
                }
            }
            else
            {
                goal_speed = 0;
                empty_flag = 0;
            }
        }
    }
    else
    {
        goal_speed = 0;
    }
}

void angle_logic()
{
}