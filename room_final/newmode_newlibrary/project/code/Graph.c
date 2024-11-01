#include "Graph.h"

float Daty_Z_old = 0;    // ��һ��IMU�Ƕ�
float azimuth_old = 0;   // ��һ��GNSS�Ƕ�
float direction_old = 0; // ��һ��Ŀ��Ƕ�

void show_ips200angle_imu()
{
    if (eulerAngle_yaw != Daty_Z_old)
    {
        ips200_clear_region(60, 100, 80, 80);
    }
    ips200_draw_line(100, 140, 100 + 40 * sin(eulerAngle_yaw * 3.14159 / 180), 140 - 40 * cos(eulerAngle_yaw * 3.14159 / 180), RGB565_RED);
    Daty_Z_old = eulerAngle_yaw;
}

void show_ips200angle_gnss()
{
    if (azimuth_new != azimuth_old || eulerAngle_yaw != Daty_Z_old)
    {
        ips200_clear_region(100, 200, 80, 80);
    }
    ips200_draw_line(140, 240, 140 + 40 * sin((azimuth_new - eulerAngle_yaw) * 3.14159 / 180), 240 - 40 * cos((azimuth_new - eulerAngle_yaw) * 3.14159 / 180), RGB565_RED);
    azimuth_old = azimuth_new;   // ������һ�νǶ�
    Daty_Z_old = eulerAngle_yaw; // ������һ�νǶ�
}

void show_ips200angle_godirection()
{
    if (gnss.direction != direction_old || azimuth_new != azimuth_old)
    {
        ips200_clear_region(40, 80, 80, 80); // �����һ�νǶ�
    }
    ips200_draw_line(80, 120, 80 + 40 * sin((azimuth_new - gnss.direction) * 3.14159 / 180), 120 - 40 * cos((azimuth_new - gnss.direction) * 3.14159 / 180), RGB565_RED);
    azimuth_old = azimuth_new;      // ������һ�νǶ�
    direction_old = gnss.direction; // ������һ�νǶ�
}