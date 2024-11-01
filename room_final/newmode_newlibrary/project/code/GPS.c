#include "zf_common_headfile.h"

// 1.确定坐标点
// 2.利用坐标点代入角度计算函数 ————>得到方向角
// 3.IMU———————>得到航向角
// 4.航向角-方向角=偏差角

int GL_Point = 40; // 有多少个点
int GL_NUM = 0;
int i = 0;
float GPS_GET_LAT[40]; // 定义一个数组，用来存放纬度数据
float GPS_GET_LOT[40]; // 定义一个数组，用来存放经度数据
uint8 gps_nember=0;
uint8 light_num=0;
extern uint8 key_down;
double azimuth = 0;      // 方向角
double azimuth_new = 0;  // 方向角
double IMU_Z = 0;        // 航向角
double distance = 0;     // 距离
uint8 light_old = 0;          // 安全标志
uint8 no_data_count = 0; // GNSS数据无更新计数

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
    GPS_GET_LAT[i] = gnss.latitude;  // 获取纬度
    GPS_GET_LOT[i] = gnss.longitude; // 获取经度
     gps_nember=gnss.satellite_used;
    if (GL_NUM > 40)                 // 如果采集点次数大于规定次数的最大值，则让他等于1
    {
        GL_NUM = 1;
    }
}

void clear_counttime()
{
     i=0;
}

/*计算方位角和距离*/
void Follow_Track()
{
  //  azimuth = get_two_points_azimuth(GPS_GET_LAT[GL_NUM - 1], GPS_GET_LOT[GL_NUM - 1], GPS_GET_LAT[GL_NUM], GPS_GET_LOT[GL_NUM]);   // 计算两点之间的方向角
     //  azimuth = get_two_points_azimuth(GPS_GET_LAT[GL_NUM - 1], GPS_GET_LOT[GL_NUM - 1], GPS_GET_LAT[GL_NUM], GPS_GET_LOT[GL_NUM]);   // 计算两点之间的方向角
  if (safe == 2 && (light_num == light_sec[0] || light_num == light_sec[1] || light_num == light_sec[2] || light_num == light_sec[3] || light_num == light_sec[4] || light_num == light_sec[5] || light_num == light_sec[6] || light_num == light_sec[7]))
    {
    distance = get_two_points_distance(gnss.latitude, gnss.longitude, GPS_GET_LAT[light_num], GPS_GET_LOT[light_num]); // 计算两点之间的距离
    azimuth_new = get_two_points_azimuth(gnss.latitude, gnss.longitude, GPS_GET_LAT[light_num], GPS_GET_LOT[light_num]);          // 计算两点之间的方向角
    // 默认对北发车
    }
     else if (safe == 2 && light_num > 0)
     {
    distance = get_two_points_distance(gnss.latitude, gnss.longitude, GPS_GET_LAT[10], GPS_GET_LOT[10]); // 计算两点之间的距离
    azimuth_new = get_two_points_azimuth(gnss.latitude, gnss.longitude, GPS_GET_LAT[10], GPS_GET_LOT[10]);          // 计算两点之间的方向角
    // 默认对北发车
     }
}
