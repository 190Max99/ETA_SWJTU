#include "zf_common_headfile.h"

extern float gyro_Offset_flag;
extern float Daty_Z;
extern float T_M;
extern float T_N;
extern int GL_IMU_Flag;

typedef struct
{
    float Xdata;
    float Ydata;
    float Zdata;
} gyro_param_t;

typedef struct
{
    float acc_x; // 加速度计x轴数据
    float acc_y; // 加速度计y轴数据
    float acc_z; // 加速度计z轴数据

    float gyro_x; // 欧拉角x轴数据
    float gyro_y; //  欧拉角y轴数据
    float gyro_z; //  欧拉角z轴数据
} IMU_param_t;

void IMU_init(void);
void IMU_GetValues(void);
float IMU_gyro_Offset_Init(void);
void IMU_YAW_integral(void);
void IMU_PITCH_integral(void);
void IMU_SHOW();
