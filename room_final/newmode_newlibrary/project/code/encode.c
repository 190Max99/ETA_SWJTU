#include "encode.h"
int16 accu_speed()
{
    int16 encoder_data_dir[2] = {0};                       // ���ڴ�ű�����������
    int16 acc_speed1 = 0;                                  // ���ڴ��ʵ���ٶ�
    encoder_data_dir[0] = encoder_get_count(ENCODER_DIR1); // ��ȡ����������
    encoder_clear_count(ENCODER_DIR1);                     // �������������
    acc_speed1 = -encoder_data_dir[0] * 2;                 // ����ʵ���ٶ�
    return acc_speed1;                                     // ����ʵ���ٶ�
}