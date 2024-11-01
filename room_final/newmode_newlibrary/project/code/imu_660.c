/*
 * imu.c
 *
 *  Created on: 2022��12��16��
 *      Author: paper
 */

#include "imu_660.h"
 //#include "math.h"
#define delta_T 0.002f // 2ms����һ��
#define alpha 0.3f
#define M_PI 3.1415926f

float GyroOffset_Xdata = 0, icm_data_acc_x = 0, icm_data_gyro_x = 0;
float GyroOffset_Ydata = 0, icm_data_acc_y = 0, icm_data_gyro_y = 0;
float GyroOffset_Zdata = 0, icm_data_acc_z = 0, icm_data_gyro_z = 0;
float Q_info_q0 = 1, Q_info_q1 = 0, Q_info_q2 = 0, Q_info_q3 = 0;
float param_Kp = 0.17;  // ���ٶȼƵ��������ʱ�������
float param_Ki = 0.004; // �������������ʵĻ������� 0.004
float eulerAngle_yaw = 0, eulerAngle_pitch = 0, eulerAngle_roll = 0, eulerAngle_yaw_total = 0, eulerAngle_yaw_old = 0;
float eulerAngle_yaw_q = 0; // ��Ԫ������ľ�yaw�ǣ����ڽ�����Ԫ������Ư
float offset = 0;        // ��Ԫ������Ư
float offset_sum = 0;    // ��Ԫ������Ư�ۻ�
uint16 offset_count = 0; // ��Ԫ������Ưʱ���ۼ�
float offset_final = 0;  // ��Ԫ������Ư����ֵ
float I_ex, I_ey, I_ez;  // ������
float test = 1;
float test1 = 1;
float test2 = 1;
float test3 = 1;
float fast_sqrt(float num) {
    float halfx = 0.5f * num;
    float y = num;
    long i = *(long*)&y;
    i = 0x5f375a86 - (i >> 1);

    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
    // float y = sqrtf(num);
    // return y;
}

void gyroOffset_init(void) { /////////��������Ʈ��ʼ��
     while (1)
    {
        if (imu660ra_init()) // ��ʼ��MPU9250
        {
            printf("imu660ra init failed\n");
        }
        else
        {
            break;
        }
    }
    GyroOffset_Xdata = 0;
    GyroOffset_Ydata = 0;
    GyroOffset_Zdata = 0;
    for (uint16_t i = 0; i < 1000; i++)
    {
        imu660ra_get_gyro();
        GyroOffset_Xdata += imu660ra_gyro_x;
        GyroOffset_Ydata += imu660ra_gyro_y;
        GyroOffset_Zdata += imu660ra_gyro_z;
        system_delay_ms(5);
    }
    GyroOffset_Xdata /= 1000;
    GyroOffset_Ydata /= 1000;
    GyroOffset_Zdata /= 1000;
    // GyroOffset_Xdata = 0.2511;
    // GyroOffset_Ydata = -1.1646;
    // GyroOffset_Zdata = 1.1402;
}

// ת��Ϊʵ������ֵ
void ICM_getValues() {
    // һ�׵�ͨ�˲�����λg/s
    icm_data_acc_x = (((float)imu660ra_acc_x) * alpha) + icm_data_acc_x * (1 - alpha);
    icm_data_acc_y = (((float)imu660ra_acc_y) * alpha) + icm_data_acc_y * (1 - alpha);
    icm_data_acc_z = (((float)imu660ra_acc_z) * alpha) + icm_data_acc_z * (1 - alpha);
    // �����ǽ��ٶ�ת����
    icm_data_gyro_x = ((float)imu660ra_gyro_x - GyroOffset_Xdata) * M_PI / 180 / 16.4f;
    icm_data_gyro_y = ((float)imu660ra_gyro_y - GyroOffset_Ydata) * M_PI / 180 / 16.4f;
    icm_data_gyro_z = ((float)imu660ra_gyro_z - GyroOffset_Zdata) * M_PI / 180 / 16.4f;
}

// �����˲�
void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az) {
    float halfT = 0.5 * delta_T;
    float vx, vy, vz; // ��ǰ�Ļ�������ϵ�ϵ�������λ����
    float ex, ey, ez; // ��Ԫ������ֵ����ٶȼƲ���ֵ�����
    float q0 = Q_info_q0;
    float q1 = Q_info_q1;
    float q2 = Q_info_q2;
    float q3 = Q_info_q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    //float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    //float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    // float delta_2 = 0;

    // �Լ��ٶ����ݽ��й�һ�� �õ���λ���ٶ�
    float norm = fast_sqrt(ax * ax + ay * ay + az * az);

    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    // ���ݵ�ǰ��Ԫ������ֵ̬����������������������ںͼ��ټ�ʵ�ʲ��������ĸ������������жԱȣ��Ӷ�ʵ�ֶ�������̬������
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    // vz = (q0*q0-0.5f+q3 * q3) * 2;

    // �������������������ʵ�ʲ�����������������������֮�����
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    // �ò���������PI����������ƫ��
    // ͨ������ param_Kp��param_Ki ����������
    // ���Կ��Ƽ��ٶȼ����������ǻ�����̬���ٶȡ�
    I_ex += halfT * ex; // integral error scaled by Ki
    I_ey += halfT * ey;
    I_ez += halfT * ez;

    gx = gx + param_Kp * ex + param_Ki * I_ex;
    gy = gy + param_Kp * ey + param_Ki * I_ey;
    gz = gz + param_Kp * ez + param_Ki * I_ez;

    /*����������ɣ���������Ԫ��΢�ַ���*/

    // ��Ԫ��΢�ַ��̣�����halfTΪ�������ڵ�1/2��gx gy gzΪ�����ǽ��ٶȣ����¶�����֪��������ʹ����һ��������������Ԫ��΢�ַ���
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
    //    delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);
    //    ������Ԫ����    ��Ԫ��΢�ַ���  ��Ԫ�������㷨�����ױϿ���
    //    q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    //    q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    //    q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    //    q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT

    // normalise quaternion
    norm = fast_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q_info_q0 = q0 * norm;
    Q_info_q1 = q1 * norm;
    Q_info_q2 = q2 * norm;
    Q_info_q3 = q3 * norm;

}

// ��ȡ������̬
void ICM_getEulerianAngles(void) {
    // �ɼ�����������
    imu660ra_get_gyro();
    imu660ra_get_acc();
    imu660ra_acc_x = imu660ra_acc_transition(imu660ra_acc_x);
    imu660ra_acc_y = imu660ra_acc_transition(imu660ra_acc_y);
    imu660ra_acc_z = imu660ra_acc_transition(imu660ra_acc_z);
    ICM_getValues();
    ICM_AHRSupdate(icm_data_gyro_x, icm_data_gyro_y, icm_data_gyro_z, icm_data_acc_x, icm_data_acc_y, icm_data_acc_z);
    // ICM_AHRSupdate(icm_data_gyro_y, icm_data_gyro_z, icm_data_gyro_x, icm_data_acc_y, icm_data_acc_z, icm_data_acc_x);//�޸�������λ�ú󣬸������̬����Ͳ��������

     // ZX XY  YZ
     // xy  yz zx
    float q0 = Q_info_q0;
    float q1 = Q_info_q1;
    float q2 = Q_info_q2;
    float q3 = Q_info_q3;
    // ��Ԫ������ŷ����---ԭʼ
    eulerAngle_roll = -asin(-2 * q1 * q3 + 2 * q0 * q2) * 180 / M_PI;                                // pitch
    eulerAngle_pitch = -atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / M_PI; // roll
    eulerAngle_yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / M_PI;  // yaw
    float i = eulerAngle_yaw - eulerAngle_yaw_old;
    if (i < -180) {
        i += 360;
    }
    else if (i > 180) {
        i -= 360;
    }
    eulerAngle_yaw_total += i;
    eulerAngle_yaw_old = eulerAngle_yaw;






    //��̬����

    // if (eulerAngle_yaw > 360) {
    //     eulerAngle_yaw -= 360;
    // }
    // else if (eulerAngle_yaw > 0) {
    //     eulerAngle_yaw += 360;
    // }

}


//void IMU_quater_readoffset()
// {
//     offset = eulerAngle_yaw - eulerAngle_yaw_q;
//     eulerAngle_yaw_q = eulerAngle_yaw;
//     offset_sum += offset;
// }