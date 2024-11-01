#include "zf_common_headfile.h"

// 1.ȷ�������
// 2.������������Ƕȼ��㺯�� ��������>�õ������
// 3.IMU��������������>�õ������
// 4.�����-�����=ƫ���

int GL_Point = 40; // �ж��ٸ���
int GL_NUM = 0;
int i = 0;
float GPS_GET_LAT[40]; // ����һ�����飬�������γ������
float GPS_GET_LOT[40]; // ����һ�����飬������ž�������
uint8 gps_nember=0;
uint8 light_num=0;
extern uint8 key_down;
double azimuth = 0;      // �����
double azimuth_new = 0;  // �����
double IMU_Z = 0;        // �����
double distance = 0;     // ����
uint8 light_old = 0;          // ��ȫ��־
uint8 no_data_count = 0; // GNSS�����޸��¼���

void gnss_proc()
{
    if(gnss_flag == 1)
    {
        gnss_flag = 0;
       gnss_data_parse();
    }
    else 
    {
        no_data_count++;
        if(no_data_count > 3)
        {
        }
    
    }
}

void GPS_GL_GET()
{
    i++;
    GL_NUM++;
    GPS_GET_LAT[i] = gnss.latitude;  // ��ȡγ��
    GPS_GET_LOT[i] = gnss.longitude; // ��ȡ����
     gps_nember=gnss.satellite_used;
    if (GL_NUM > 40)                 // ����ɼ���������ڹ涨���������ֵ������������1
    {
        GL_NUM = 1;
    }
}

void clear_counttime()
{
     i=0;
}

/*���㷽λ�Ǻ;���*/
void Follow_Track()
{
  //  azimuth = get_two_points_azimuth(GPS_GET_LAT[GL_NUM - 1], GPS_GET_LOT[GL_NUM - 1], GPS_GET_LAT[GL_NUM], GPS_GET_LOT[GL_NUM]);   // ��������֮��ķ����
     //  azimuth = get_two_points_azimuth(GPS_GET_LAT[GL_NUM - 1], GPS_GET_LOT[GL_NUM - 1], GPS_GET_LAT[GL_NUM], GPS_GET_LOT[GL_NUM]);   // ��������֮��ķ����
  if (safe == 2 && (light_num == light_sec[0] || light_num == light_sec[1] || light_num == light_sec[2] || light_num == light_sec[3] || light_num == light_sec[4] || light_num == light_sec[5] || light_num == light_sec[6] || light_num == light_sec[7]))
    {
    distance = get_two_points_distance(gnss.latitude, gnss.longitude, GPS_GET_LAT[light_num], GPS_GET_LOT[light_num]); // ��������֮��ľ���
    azimuth_new = get_two_points_azimuth(gnss.latitude, gnss.longitude, GPS_GET_LAT[light_num], GPS_GET_LOT[light_num]);          // ��������֮��ķ����
    // Ĭ�϶Ա�����
    }
     else if (safe == 2 && light_num > 0)
     {
    distance = get_two_points_distance(gnss.latitude, gnss.longitude, GPS_GET_LAT[10], GPS_GET_LOT[10]); // ��������֮��ľ���
    azimuth_new = get_two_points_azimuth(gnss.latitude, gnss.longitude, GPS_GET_LAT[10], GPS_GET_LOT[10]);          // ��������֮��ķ����
    // Ĭ�϶Ա�����
     }
}
