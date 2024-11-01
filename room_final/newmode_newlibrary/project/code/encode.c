#include "encode.h"
int16 accu_speed()
{
    int16 encoder_data_dir[2] = {0};                       // 用于存放编码器的数据
    int16 acc_speed1 = 0;                                  // 用于存放实际速度
    encoder_data_dir[0] = encoder_get_count(ENCODER_DIR1); // 获取编码器数据
    encoder_clear_count(ENCODER_DIR1);                     // 清除编码器数据
    acc_speed1 = -encoder_data_dir[0] * 2;                 // 计算实际速度
    return acc_speed1;                                     // 返回实际速度
}