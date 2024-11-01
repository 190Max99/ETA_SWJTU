#include "IMU.h"

gyro_param_t Gyro_Offset; // 陀螺仪零飘结构体
IMU_param_t IMU_Data;     // 减去零飘数据结构体

/**
 * @brief 陀螺仪零漂初始化
 * 通过采集一定数据求均值计算陀螺仪零点偏移值。
 * 后续 陀螺仪读取的数据 - 零飘值，即可去除零点偏移量。
 */

int GL_IMU_Flag = 0;
float gyro_Offset_flag;
float Daty_Z;
float T_N;
float T_M;

float IMU_gyro_Offset_Init()
{
    Gyro_Offset.Xdata = 0;
    Gyro_Offset.Ydata = 0;
    Gyro_Offset.Zdata = 0;
    for (uint16_t i = 0; i < 1000; i++)
    {
        //        Gyro_Offset.Xdata += imu660ra_gyro_x;
        //        Gyro_Offset.Ydata += imu660ra_gyro_y;
        //        Gyro_Offset.Zdata += imu660ra_gyro_z;

        Gyro_Offset.Xdata += imu963ra_gyro_x;
        Gyro_Offset.Ydata += imu963ra_gyro_y;
        Gyro_Offset.Zdata += imu963ra_gyro_z;
        system_delay_ms(2); // 最大 1Khz
    }

    Gyro_Offset.Xdata /= 1000;
    Gyro_Offset.Ydata /= 1000;
    Gyro_Offset.Zdata /= 1000;

    return gyro_Offset_flag = 1;
}

/**
 * @brief 将采集的数值转化为实际物理值, 并对陀螺仪进行去零漂处理
 * 加速度计初始化配置 -> 测量范围: ±8g        对应灵敏度: 4096 LSB/g
 * 陀螺仪初始化配置   -> 测量范围: ±2000 dps  对应灵敏度: 16.4 LSB/dps   (degree per second)
 * @tips: gyro = (gyro_val / 16.4) °/s = ((gyro_val / 16.4) * PI / 180) rad/s
 */
void IMU_GetValues() // 将采集的数值转化为实际物理值, 并对陀螺仪进行去零漂处理
{
    // 2000dps:IMU660--16.4
    // 2000dps:IMU963--14.3

    //! 陀螺仪角速度必须转换为弧度制角速度: deg/s -> rad/s
    // IMU_Data.gyro_x = ((float) imu660ra_gyro_x - Gyro_Offset.Xdata) * PI / 180 / 16.4f;
    // IMU_Data.gyro_y = ((float) imu660ra_gyro_y - Gyro_Offset.Ydata) * PI / 180 / 16.4f;
    // IMU_Data.gyro_z = ((float) imu660ra_gyro_z - Gyro_Offset.Zdata) * PI / 180 / 16.4f;

    IMU_Data.gyro_x = ((float)imu963ra_gyro_x - Gyro_Offset.Xdata) * PI / 180 / 14.3f;
    IMU_Data.gyro_y = ((float)imu963ra_gyro_y - Gyro_Offset.Ydata) * PI / 180 / 14.3f;
    IMU_Data.gyro_z = ((float)imu963ra_gyro_z - Gyro_Offset.Zdata) * PI / 180 / 14.3f;
}

void IMU_init() // IMU初始化
{
    while (1)
    {
        if (imu963ra_init()) // 初始化MPU9250
        {
            printf("imu963ra init failed\n");
        }
        else
        {
            break;
        }
    }
    pit_ms_init(PIT_CH1, 5); // (IMU)初始化 PIT_CH1 为周期中断 5ms 周期,IMU660频率为200HZ
    IMU_gyro_Offset_Init();  // 陀螺仪零漂初始化
}

void IMU_Handle_180()
{
    Daty_Z -= RAD_TO_ANGLE(IMU_Data.gyro_z * 0.005); //(积分过程)本来是逆时针为正,现在改为顺时针为正

    if ((Daty_Z > 0 && Daty_Z <= 180) || (Daty_Z < 0 && Daty_Z >= (-180))) // 顺时针
    {
        Daty_Z = +Daty_Z;
    }
    else if (Daty_Z > 180 && Daty_Z <= 360)
    {
        Daty_Z -= 360;
    }
    else if (Daty_Z < (-180) && Daty_Z >= (-360))
    {
        Daty_Z += 360;
    }
}

void IMU_Handle_360()
{
    T_M -= RAD_TO_ANGLE(IMU_Data.gyro_z * 0.005); //(积分过程)本来是逆时针为正,现在改为顺时针为正

    if (T_M > 360 || T_M < -360) // 限制IMU的数值在0-360之间
    {
        T_M = 0;
    }
}

void IMU_Handle_0()
{
    T_N -= RAD_TO_ANGLE(IMU_Data.gyro_z * 0.005); //(积分过程)本来是逆时针为正,现在改为顺时针为正
}

void IMU_YAW_integral() // 对角速度进行积分
{

    IMU_GetValues(); // 获取陀螺仪数据
    //    Daty_X+=RAD_TO_ANGLE(IMU_Data.gyro_x*0.005);
    //    Daty_Y+=RAD_TO_ANGLE(IMU_Data.gyro_y*0.005);
    //    if(IMU_Data.gyro_z<0.0045&&IMU_Data.gyro_z>-0.0045)
    if (IMU_Data.gyro_z < 0.015 && IMU_Data.gyro_z > -0.015) // 滤波
    {
        Daty_Z -= 0;
    }
    else
    {
        IMU_Handle_180(); // 规划为0-180和0-(-180)   Daty_Z
        IMU_Handle_360(); // 规划为0-360和0-(-360)   T_M
        IMU_Handle_0();   // 规划为0-正无穷和0-负无穷  T_N
    }
}

void IMU_SHOW()
{

    ips114_show_string(0, 0, "Yaw:");
    ips114_show_float(100, 0, Daty_Z, 3, 6);
    ips114_show_string(0, 20, "T_M:");
    ips114_show_float(100, 20, T_M, 3, 6);
    ips114_show_string(0, 40, "T_N:");
    ips114_show_float(100, 40, T_N, 3, 6);
}
