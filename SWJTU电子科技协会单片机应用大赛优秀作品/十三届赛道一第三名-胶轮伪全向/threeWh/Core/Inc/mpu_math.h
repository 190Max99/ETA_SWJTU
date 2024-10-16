#ifndef __MPU_MATH_H
#define __MPU_MATH_H

#include "arm_math.h"
#include "Kalman_Filter.h"

#define Kp 100.0f                        // ��������֧�������������ٶȼ�/��ǿ��
#define Ki 0.002f                // ��������֧���ʵ�������ƫ�����ν�
#define halfT 0.02f                // �������ڵ�һ��

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);

#endif