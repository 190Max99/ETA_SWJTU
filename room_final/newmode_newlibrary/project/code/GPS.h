 #ifndef CODE_GPS_H_
#define CODE_GPS_H_

#include "zf_common_headfile.h"

extern float GPS_GET_LAT[40]; // 定义一个数组，用来存放纬度数据
extern float GPS_GET_LOT[40]; // 定义一个数组，用来存放经度数据
extern double azimuth;         // 方向角
extern double IMU_Z;           // 航向角
extern double azimuth_new;     // 方向角
extern uint8 gps_nember;             // 卫星数量
extern uint8 light_num;               // 灯编号
extern double distance;     // 距离
extern uint8 light_old;     // 灯
void GPS_GL_GET();   // 获取GPS数据
void Follow_Track(); // 跟踪轨迹
void gnss_proc();
void clear_counttime();
#endif /* CODE_GPS_H_ */