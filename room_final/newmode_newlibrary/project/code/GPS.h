 #ifndef CODE_GPS_H_
#define CODE_GPS_H_

#include "zf_common_headfile.h"

extern float GPS_GET_LAT[40]; // ����һ�����飬�������γ������
extern float GPS_GET_LOT[40]; // ����һ�����飬������ž�������
extern double azimuth;         // �����
extern double IMU_Z;           // �����
extern double azimuth_new;     // �����
extern uint8 gps_nember;             // ��������
extern uint8 light_num;               // �Ʊ��
extern double distance;     // ����
extern uint8 light_old;     // ��
void GPS_GL_GET();   // ��ȡGPS����
void Follow_Track(); // ���ٹ켣
void gnss_proc();
void clear_counttime();
#endif /* CODE_GPS_H_ */