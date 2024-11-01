/*********************************************************************************************************************
 * CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 CYT4BB 开源库的一部分
 *
 * CYT4BB 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          main_cm7_1
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          IAR 9.40.1
 * 适用平台          CYT4BB
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2024-1-4       pudding            first version
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "arm_math.h"
#include "math.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// **************************** 代码区域 ****************************
/***定时器中断***/
uint8 ShowFlag = 0; // 屏幕显示标志位

/***声音信号采集***/
#define ADC_CHANNEL1 (ADC1_CH09_P12_5) // 宏定义P12.5为ADC_CHANNEL1
#define ADC_CHANNEL2 (ADC1_CH08_P12_4) // 宏定义P12.4为ADC_CHANNEL2
#define ADC_CHANNEL3 (ADC1_CH25_P14_5) // 宏定义P14.5为ADC_CHANNEL3
#define ADC_CHANNEL4 (ADC1_CH00_P10_4) // 宏定义P10.4为ADC_CHANNEL4

/***互相关算法***/
#define FFT_SIZE 2048                  // FFT计算长度为2048
#define MIC_RAW_DATA_LEN 2500          // 用于采集硅麦的循环数组长度（需大于FFT_SIZE，防止数据异常）
int16 mic1_raw_data[MIC_RAW_DATA_LEN]; // 采集硅麦1数据的循环数组
int16 mic2_raw_data[MIC_RAW_DATA_LEN]; // 采集硅麦2数据的循环数组
int16 mic3_raw_data[MIC_RAW_DATA_LEN]; // 采集硅麦3数据的循环数组
int16 mic4_raw_data[MIC_RAW_DATA_LEN]; // 采集硅麦4数据的循环数组
int16 mic_raw_data_count = 0;          // 存储循环数组计数位
int16 mic_raw_data_count_save = 0;     // 存储循环数组计数位的当前值（用于取出最近的FFT_SIZE个数据）
float Mic1_fftIn[FFT_SIZE * 2];        // 麦克风Mic1参与FFT的时域输入数组/经过FFT之后得到的频域输出数组（输入信号为复数，所以长度为 FFT_SIZE * 2）
float Mic2_fftIn[FFT_SIZE * 2];        // 麦克风Mic2参与FFT的时域输入数组/经过FFT之后得到的频域输出数组（输入信号为复数，所以长度为 FFT_SIZE * 2）
float Mic3_fftIn[FFT_SIZE * 2];        // 麦克风Mic3参与FFT的时域输入数组/经过FFT之后得到的频域输出数组（输入信号为复数，所以长度为 FFT_SIZE * 2）
float Mic4_fftIn[FFT_SIZE * 2];        // 麦克风Mic4参与FFT的时域输入数组/经过FFT之后得到的频域输出数组（输入信号为复数，所以长度为 FFT_SIZE * 2）
float Mic13_fft_ConjMul[FFT_SIZE * 2]; // 麦克风Mic1和麦克风Mic3频域共轭乘积数组
float Mic24_fft_ConjMul[FFT_SIZE * 2]; // 麦克风Mic2和麦克风Mic4频域共轭乘积数组
float Mic13_Correlation[FFT_SIZE];     // 麦克风Mic1和麦克风Mic3互相关运算结果
float Mic24_Correlation[FFT_SIZE];     // 麦克风Mic2和麦克风Mic4互相关运算结果
float Mic13_max = 0;                   // 存储互相关运算得到的最大值
float Mic24_max = 0;                   // 存储互相关运算得到的最大值
int16 Mic13_Max_Array_Num = 0;         // 互相关运算得到的最大值对应的数组下标
int16 Mic24_Max_Array_Num = 0;         // 互相关运算得到的最大值对应的数组下标
int16 Mic13_Delay_Temp = 0;            // Mic1和Mic3的时延差（含无效数据）
int16 Mic24_Delay_Temp = 0;            // Mic2和Mic4的时延差（含无效数据）
int16 Last_Mic13_Delay = 0;            // 上一次Mic1和Mic3的有效时延差
int16 Last_Mic24_Delay = 0;            // 上一次Mic2和Mic4的有效时延差
int16 Mic13_Delay = 0;                 // Mic1和Mic3的有效时延差
int16 Mic24_Delay = 0;                 // Mic2和Mic4的有效时延差
uint8 Mic1_Chirp_Flag = 0;             // Mic1的Chirp信号有效标志位
uint8 Mic2_Chirp_Flag = 0;             // Mic1的Chirp信号有效标志位
uint8 Mic3_Chirp_Flag = 0;             // Mic1的Chirp信号有效标志位
uint8 Mic4_Chirp_Flag = 0;             // Mic1的Chirp信号有效标志位
uint8 ChirpFlag = 0;                   // Chirp信号有效标志位

uint8 start_count = 0;  // 记时标志位
uint16 time_count1 = 0; // 记录时间
uint16 time_count2 = 0; // 显示时间
uint32 data_to_cm70 = 0;
uint32 sound_en=0;

uint8 flag_m1=0;
uint8 flag_m2=0;
uint8 flag_m3=0;
uint8 flag_m4=0;


/***声源定位***/
float Angle_rad = 0;
float Angle_deg = 0;

/***核间通信***/
#define FLASH_SECTION_INDEX (0) // 存储 M7_1 数据地址及长度 用的扇区
#define FLASH_PAGE_INDEX (1)    // 存储 M7_1 数据长度及长度 用的页码
#define DATA_LENGTH (1)         // 数组数据长度
uint8 m7_1_data[DATA_LENGTH];   // 定义 M7_1数据数组

#define DATA_LENGTH               (1)                                           // 数组数据长度

#pragma location = 0x28001000                                                   // 将下面这个数组定义到指定的RAM地址，便于其他核心直接访问(开源库默认在 0x28001000 地址保留了8kb的空间用于数据交互)
                                                                                // 此处为0x28001014的原因是前面放了一个M0的数组
uint8 m7_data[DATA_LENGTH] = {10};                        // 定义 M7_1 演示数据数组 浮点数类型


unsigned char send[15];

void my_ipc_callback(uint32 receive_data)
{
    // printf("receive M7_0 data:%d\r\n", receive_data);        // 将接收到的数据打印到串口
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); // 时钟配置及系统初始化<务必保留>
                                   /* debug串口由核心M7_0初始化，此处不需要再次初始化debug串口 */

    // 此处编写用户代码 例如外设初始化代码等
    /***核间通信初始化***/
    /*
    flash_init();            // flash初始化(访问flash之前需要初始化一次)

    flash_union_buffer[0].uint32_type = (uint32)&m7_1_data[0];      // 取数组首地址
    flash_union_buffer[1].uint32_type = DATA_LENGTH;                // 取数组长度

    flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);                    // 写入flash之前需要擦除当前页 否则可能导致数据错误
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, 2);     // 将地址和长度信息写入flash
    */

    SCB_DisableDCache(); // 关闭DCashe

    ipc_communicate_init(IPC_PORT_2, my_ipc_callback); // 初始化IPC模块 选择端口2 填写中断回调函数

    /***定时器中断初始化***/
    pit_us_init(PIT_CH0, 100); // 初始化CCU6_0_CH0为周期中断100us周期，即ADC采样频率为10kHz(Chirp声音信号的频率为250~2000Hz)
    // pit_ms_init(PIT_CH1, 100); // 初始化 CCU6_0_CH1 为周期中断 200ms 周期
    /***屏幕初始化***/
    // tft180_init(); // 初始化2寸屏幕
    /***ADC初始化***/
    adc_init(ADC_CHANNEL1, ADC_12BIT); //  初始化对应 ADC 通道为对应精度（12位精度）
    adc_init(ADC_CHANNEL2, ADC_12BIT); //  初始化对应 ADC 通道为对应精度（12位精度）
    adc_init(ADC_CHANNEL3, ADC_12BIT); //  初始化对应 ADC 通道为对应精度（12位精度）
    adc_init(ADC_CHANNEL4, ADC_12BIT); //  初始化对应 ADC 通道为对应精度（12位精度）
 //   if (wireless_uart_init())
  //  {
  //      while (1)
  //      {
  //      }
  //  }
    /***逐飞虚拟示波器初始化***/
    //  seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART); // 初始化逐飞串口助手，使用DEBUG_UART进行数据收发
    //  seekfree_assistant_oscilloscope_struct oscilloscope_data;         // 实例化虚拟示波器结构体
    //  oscilloscope_data.channel_num = 1;                                // 设置虚拟示波器的通道为1

    /***测试***/
    /*
    float signal[FFT_SIZE];
    for(int i = 0; i < FFT_SIZE; i++)
    {
        signal[i] = arm_sin_f32(PI*i/10);
    }
    for(int i = 0; i < FFT_SIZE; i++)
    {
        Mic1_fftIn[i*2] = signal[i+5];
        Mic1_fftIn[i*2 + 1] = 0;
        Mic3_fftIn[i*2] = signal[i];
        Mic3_fftIn[i*2 + 1] = 0;

        Mic2_fftIn[i*2] = signal[i+5];
        Mic2_fftIn[i*2 + 1] = 0;
        Mic4_fftIn[i*2] = signal[i];
        Mic4_fftIn[i*2 + 1] = 0;
    }
    */

    // 此处编写用户代码 例如外设初始化代码等
    while (true)
    {
        // 此处编写需要循环执行的代码
        /***屏幕显示***/
        // if (ShowFlag == 1)
        // {
        //     ShowFlag = 0; // 屏幕显示标志位清0

        //     tft180_show_int(0, 0, Mic13_Max_Array_Num, 5);  // 显示时延差数据
        //     tft180_show_int(0, 20, Mic24_Max_Array_Num, 5); // 显示时延差数据
        //     tft180_show_int(0, 40, time_count2, 5);      // 结算一次的时间
        //     tft180_show_float(0, 60, Angle_deg, 3, 2);      // 显示方位角数据
        //     tft180_show_int(0, 80, Mic13_Delay, 5);         // 显示时延差数据
        //     tft180_show_int(0, 100, Mic24_Delay, 5);        // 显示时延差数据
        //     // tft180_show_int(0, 120, Last_Mic13_Delay, 5);   // 显示时延差数据
        //     // tft180_show_int(0, 140, Last_Mic24_Delay, 5);   // 显示时延差数据
        // }

        /***互相关计算及声源定位***/
        // 准备FFT计算所需的数据
        start_count = 1;
        uint16 mic_raw_data_count_temp = 0;
        uint16 mic_data_count = 0;

        mic_raw_data_count_save = mic_raw_data_count; // 保存当前时刻的循环数组计数位（用于取出最近的FFT_SIZE个数据）

        if (mic_raw_data_count_save < FFT_SIZE) // 保存当前时刻的循环数组计数位小于FFT_SIZE，则需从循环数组的两端取出最近的FFT_SIZE个数据
        {
            mic_raw_data_count_temp = MIC_RAW_DATA_LEN - (FFT_SIZE - mic_raw_data_count_save); // 计算循环数组后端复制开始位置

            // 循环数组后端
            for (int16 i = mic_raw_data_count_temp; i < MIC_RAW_DATA_LEN; i++)
            {
                Mic1_fftIn[mic_data_count * 2] = mic1_raw_data[i]; // 复制麦克风1的AD值存到实部，虚部为0
                Mic1_fftIn[mic_data_count * 2 + 1] = 0;
                Mic2_fftIn[mic_data_count * 2] = mic2_raw_data[i]; // 复制麦克风2的AD值存到实部，虚部为0
                Mic2_fftIn[mic_data_count * 2 + 1] = 0;
                Mic3_fftIn[mic_data_count * 2] = mic3_raw_data[i]; // 复制麦克风3的AD值存到实部，虚部为0
                Mic3_fftIn[mic_data_count * 2 + 1] = 0;
                Mic4_fftIn[mic_data_count * 2] = mic4_raw_data[i]; // 复制麦克风4的AD值存到实部，虚部为0
                Mic4_fftIn[mic_data_count * 2 + 1] = 0;
                mic_data_count++;
            }
            // 循环数组前端
            for (int16 i = 0; i < mic_raw_data_count_save; i++)
            {
                Mic1_fftIn[mic_data_count * 2] = mic1_raw_data[i]; // 复制麦克风1的AD值存到实部，虚部为0
                Mic1_fftIn[mic_data_count * 2 + 1] = 0;
                Mic2_fftIn[mic_data_count * 2] = mic2_raw_data[i]; // 复制麦克风2的AD值存到实部，虚部为0
                Mic2_fftIn[mic_data_count * 2 + 1] = 0;
                Mic3_fftIn[mic_data_count * 2] = mic3_raw_data[i]; // 复制麦克风3的AD值存到实部，虚部为0
                Mic3_fftIn[mic_data_count * 2 + 1] = 0;
                Mic4_fftIn[mic_data_count * 2] = mic4_raw_data[i]; // 复制麦克风4的AD值存到实部，虚部为0
                Mic4_fftIn[mic_data_count * 2 + 1] = 0;
                mic_data_count++;
            }
        }
        else // 保存当前时刻的循环数组计数位大于等于FFT_SIZE，则取出循环数组的前FFT_SIZE个数据
        {
            for (int16 i = 0; i < FFT_SIZE; i++)
            {
                Mic1_fftIn[i * 2] = mic1_raw_data[i]; // 复制麦克风1的AD值存到实部，虚部为0
                Mic1_fftIn[i * 2 + 1] = 0;
                Mic2_fftIn[i * 2] = mic2_raw_data[i]; // 复制麦克风2的AD值存到实部，虚部为0
                Mic2_fftIn[i * 2 + 1] = 0;
                Mic3_fftIn[i * 2] = mic3_raw_data[i]; // 复制麦克风3的AD值存到实部，虚部为0
                Mic3_fftIn[i * 2 + 1] = 0;
                Mic4_fftIn[i * 2] = mic4_raw_data[i]; // 复制麦克风4的AD值存到实部，虚部为0
                Mic4_fftIn[i * 2 + 1] = 0;
            }
        }

        // FFT运算
        arm_cfft_instance_f32 arm_cfft_instance_f32_len_FFT_SIZE;            // 定义FFT对象
        arm_cfft_init_f32(&arm_cfft_instance_f32_len_FFT_SIZE, FFT_SIZE);    // 初始化FFT对象，赋予计算长度
        arm_cfft_f32(&arm_cfft_instance_f32_len_FFT_SIZE, Mic1_fftIn, 0, 1); // 麦克风1的32位浮点FFT运算，输出倒位序
        arm_cfft_f32(&arm_cfft_instance_f32_len_FFT_SIZE, Mic2_fftIn, 0, 1); // 麦克风2的32位浮点FFT运算，输出倒位序
        arm_cfft_f32(&arm_cfft_instance_f32_len_FFT_SIZE, Mic3_fftIn, 0, 1); // 麦克风3的32位浮点FFT运算，输出倒位序
        arm_cfft_f32(&arm_cfft_instance_f32_len_FFT_SIZE, Mic4_fftIn, 0, 1); // 麦克风4的32位浮点FFT运算，输出倒位序

        // for (int i = 0; i < FFT_SIZE*2; i++)
        // {
        //     // oscilloscope_data.data[0] = Mic13_Correlation[i];
        //     // seekfree_assistant_oscilloscope_send(&oscilloscope_data); // 给逐飞虚拟示波器发送数据
        //     // if(i==0)
        //     // {
        //     //     sprintf(send,"$:%d\n",100000);
        //     //      wireless_uart_send_buffer(send,10);
        //     // }
        //     // else{
        //     sprintf(send,"q:%d,%d\n",(unsigned int)Mic1_fftIn[i],i);
        //     wireless_uart_send_buffer(send,15);
        //   //  }
        // }

        // 噪声滤波（理想带通）
        for (int i = 0; i < 250; i++)
        {
            Mic1_fftIn[i] = 0;
            Mic2_fftIn[i] = 0;
            Mic3_fftIn[i] = 0;
            Mic4_fftIn[i] = 0;
        }
        for (int i = 600; i < 3596; i++)
        {
            Mic1_fftIn[i] = 0;
            Mic2_fftIn[i] = 0;
            Mic3_fftIn[i] = 0;
            Mic4_fftIn[i] = 0;
        }
        for (int i = 3900; i < FFT_SIZE * 2; i++)
        {
            Mic1_fftIn[i] = 0;
            Mic2_fftIn[i] = 0;
            Mic3_fftIn[i] = 0;
            Mic4_fftIn[i] = 0;
        }

        //  for (int i = 0; i < FFT_SIZE*2; i++)
        // {
        //     // oscilloscope_data.data[0] = Mic13_Correlation[i];
        //     // seekfree_assistant_oscilloscope_send(&oscilloscope_data); // 给逐飞虚拟示波器发送数据
        //     // if(i==0)
        //     // {
        //     //     sprintf(send,"$:%d\n",100000);
        //     //      wireless_uart_send_buffer(send,10);
        //     // }
        //     // else{
        //    sprintf(send,"q:%d,%d\n",(unsigned int)Mic1_fftIn[i],i);
        //     wireless_uart_send_buffer(send,15);
        //   //  }
        // }

        Mic1_Chirp_Flag = 0; // 变量清0
        Mic2_Chirp_Flag = 0;
        Mic3_Chirp_Flag = 0;
        Mic4_Chirp_Flag = 0;
        flag_m1=0;
        flag_m2=0;
        flag_m3=0;
        flag_m4=0;
        ChirpFlag = 0;

        for (int i = 250; i < 599; i++)
        {
            if (Mic1_fftIn[i] >= 8000)
            {
                Mic1_Chirp_Flag = 1;
                Mic2_Chirp_Flag = 1;
                Mic3_Chirp_Flag = 1;
                Mic4_Chirp_Flag = 1;
                flag_m1=1;
              //  m7_data[0]=1;
            }
            if (Mic2_fftIn[i] >= 8000)
            {
                Mic1_Chirp_Flag = 1;
                Mic2_Chirp_Flag = 1;
                Mic3_Chirp_Flag = 1;
                Mic4_Chirp_Flag = 1;
                //  m7_data[0]=1;
                flag_m2=1;
                 
            }
            if (Mic3_fftIn[i] >= 8000)
            {
                Mic1_Chirp_Flag = 1;
                Mic2_Chirp_Flag = 1;
                Mic3_Chirp_Flag = 1;
                Mic4_Chirp_Flag = 1;
                flag_m3=1;
              //    m7_data[0]=1;
            }
            if (Mic4_fftIn[i] >= 8000)
            {
                Mic1_Chirp_Flag = 1;
                Mic2_Chirp_Flag = 1;
                Mic3_Chirp_Flag = 1;
                Mic4_Chirp_Flag = 1;
                flag_m4=1;
              //    m7_data[0]=1;
            }
           
        }
        if ((Mic1_Chirp_Flag == 1) && (Mic2_Chirp_Flag == 1) && (Mic3_Chirp_Flag == 1) && (Mic4_Chirp_Flag == 1))
        {
            ChirpFlag = 1;
             m7_data[0]=1;
        }
        else 
        {
          m7_data[0]=10;
        }
        

        // 逐飞虚拟示波器
        // for (int i = 0; i < FFT_SIZE * 2; i++)
        // {
        //     oscilloscope_data.data[0] = Mic1_fftIn[i];
        //     seekfree_assistant_oscilloscope_send(&oscilloscope_data); // 给逐飞虚拟示波器发送数据
        // }

        // 麦克风1频域与麦克风3频域共轭的乘积，麦克风2频域与麦克风4频域共轭的乘积
        for (int i = 0; i < FFT_SIZE; i++)
        {
            Mic3_fftIn[2 * i + 1] = -Mic3_fftIn[2 * i + 1]; // 对麦克风3频域进行共轭
            Mic4_fftIn[2 * i + 1] = -Mic4_fftIn[2 * i + 1]; // 对麦克风4频域进行共轭
        }
        arm_cmplx_mult_cmplx_f32(Mic1_fftIn, Mic3_fftIn, Mic13_fft_ConjMul, FFT_SIZE);
        arm_cmplx_mult_cmplx_f32(Mic2_fftIn, Mic4_fftIn, Mic24_fft_ConjMul, FFT_SIZE);

        // 共轭乘积的32位浮点IFFT运算，输出倒位序
        arm_cfft_f32(&arm_cfft_instance_f32_len_FFT_SIZE, Mic13_fft_ConjMul, 1, 1);
        arm_cfft_f32(&arm_cfft_instance_f32_len_FFT_SIZE, Mic24_fft_ConjMul, 1, 1);

        // 计算幅度谱
        arm_cmplx_mag_f32(Mic13_fft_ConjMul, Mic13_Correlation, FFT_SIZE);
        arm_cmplx_mag_f32(Mic24_fft_ConjMul, Mic24_Correlation, FFT_SIZE);

        // 交换位置
        // float DataTemp1 = 0;
        // float DataTemp2 = 0;
        // for (int i = 0; i < FFT_SIZE; i++)
        // {
        //     DataTemp1 = Mic13_Correlation[i];
        //     Mic13_Correlation[i] = Mic13_Correlation[FFT_SIZE / 2 + i];
        //     Mic13_Correlation[FFT_SIZE / 2 + i] = DataTemp1;

        //     DataTemp2 = Mic24_Correlation[i];
        //     Mic24_Correlation[i] = Mic24_Correlation[FFT_SIZE / 2 + i];
        //     Mic24_Correlation[FFT_SIZE / 2 + i] = DataTemp2;
        // }

        // 最大值清0
        Mic13_max = 0;
        Mic24_max = 0;

        // 找出相关性最好的点对应的数组下标
        for (int i = 0; i < FFT_SIZE; i++)
        {
            if (Mic13_Correlation[i] > Mic13_max)
            {
                Mic13_max = Mic13_Correlation[i]; // 找出幅值最大的点,此即为相关性最好的点
                Mic13_Max_Array_Num = i;          // 相关性最好的点对应的数组下标
            }

            if (Mic24_Correlation[i] > Mic24_max)
            {
                Mic24_max = Mic24_Correlation[i]; // 找出幅值最大的点,此即为相关性最好的点
                Mic24_Max_Array_Num = i;          // 相关性最好的点对应的数组下标
            }
        }
        /*根据先接收到信号的顺序，最大峰值分别为靠近0和靠近2048*/

        if (Mic13_Max_Array_Num > 11) // 如果靠近2048则减去2048
        {
            Mic13_Delay = Mic13_Max_Array_Num - 2048;
        }
        else
        {
            Mic13_Delay = Mic13_Max_Array_Num;
        }

        if (Mic24_Max_Array_Num > 11)
        {
            Mic24_Delay = Mic24_Max_Array_Num - 2048;
        }
        else
        {
            Mic24_Delay = Mic24_Max_Array_Num;
        }

        /*滤除无效数据(几乎不存在无效数据)*/
        if ((Mic13_Delay <= 11) && (Mic13_Delay >= -11) && (Mic24_Delay <= 11) && (Mic24_Delay >= -11) && (ChirpFlag == 1))
        {
            Last_Mic13_Delay = Mic13_Delay;
            Last_Mic24_Delay = Mic24_Delay;
        }
        else
        {
            Mic13_Delay = Last_Mic13_Delay;
            Mic24_Delay = Last_Mic24_Delay;
        }
        start_count = 0; // 停止计数

        // 过滤出有效时延差
        // if ((Mic13_Delay_Temp <= 10) && (Mic13_Delay_Temp >= -10) && (Mic24_Delay_Temp <= 10) && (Mic24_Delay_Temp >= -10))
        // {
        //     Mic13_Delay = Mic13_Delay_Temp;
        //     Mic24_Delay = Mic24_Delay_Temp;
        //     Last_Mic13_Delay = Mic13_Delay;
        //     Last_Mic24_Delay = Mic24_Delay;
        // }
        // else if ((Mic13_Delay_Temp <= 10))
        // {
        //     Mic13_Delay = Last_Mic13_Delay;
        //     Mic24_Delay = Last_Mic24_Delay;
        // }

        // 计算方位角
        // C语言里float atan2f(float, float)返回的是原点至点(x,y)的方位角，即与x轴的夹角。返回值的单位为弧度，取值范围为（-π, π]。
        // 结果为正表示从 X 轴逆时针旋转的角度，结果为负表示从 X 轴顺时针旋转的角度。
        // 若要用度表示反正切值，请将结果再乘以 180/π。另外要注意的是，函数atan2f(y,x)中参数的顺序是倒置的，atan2f(y,x)计算的值相当于点(x,y)的角度值。
        Angle_rad = atan2f((float)Mic24_Delay, (float)Mic13_Delay);
        Angle_deg = Angle_rad * 180 / PI;
        if (Angle_deg < 0)
            Angle_deg += 360; // 将负数转换为正数
        data_to_cm70 = Angle_deg;
        ipc_send_data(data_to_cm70); // 发送数据给核心M7_0
        time_count2 = time_count1;   // 记录将解算一次的结果的时间
        // 核间通信
        // m7_1_data[0] = WeightMax_Array_Num;     //将数据存入数据数组
        //  M7_1核心有Dcache 当数据有变化时应该更新Dcache的内容 否则数据无法同步到RAM(其他核心访问的RAM地址也就无法读取到数据)
        // SCB_CleanInvalidateDCache();
        // wireless_uart_send_string(("zhiguoxin:%d, %d\n", acc_speed1, goal_speed);
        // 逐飞虚拟示波器
        // for (int i = 0; i < FFT_SIZE; i++)
        // {
        //     // oscilloscope_data.data[0] = Mic13_Correlation[i];
        //     // seekfree_assistant_oscilloscope_send(&oscilloscope_data); // 给逐飞虚拟示波器发送数据
        //     sprintf(send,"$:%f\n",Mic13_Correlation[i]);
        //     wireless_uart_send_buffer(send,10);
        // }

        // 此处编写需要循环执行的代码
    }
}

/***周期中断0服务程序（读取麦克风的AD值存到循环数组中）***/
void pit0_ch0_isr()
{
    pit_isr_flag_clear(PIT_CH0); // 清除中断标志

    mic1_raw_data[mic_raw_data_count] = adc_convert(ADC_CHANNEL1) - 2000; // 读取麦克风1的AD值并减去直流偏量
    mic2_raw_data[mic_raw_data_count] = adc_convert(ADC_CHANNEL2) - 2000; // 读取麦克风2的AD值并减去直流偏量
    mic3_raw_data[mic_raw_data_count] = adc_convert(ADC_CHANNEL3) - 2000; // 读取麦克风3的AD值并减去直流偏量
    mic4_raw_data[mic_raw_data_count] = adc_convert(ADC_CHANNEL4) - 2000; // 读取麦克风4的AD值并减去直流偏量

    mic_raw_data_count++; // 采集点数加1

    if (mic_raw_data_count >= 2500) // 为下一轮ADC采样做准备
    {
        mic_raw_data_count = 0;
    }
    if (start_count)
    {
        time_count1++;
    } // 记录解算时长
    else
        time_count1 = 0;
}

// /***周期中断1服务程序（屏幕显示标志位置位）***/
// void pit0_ch1_isr()
// {
//     pit_isr_flag_clear(PIT_CH1); // 清除中断标志
//     ShowFlag = 1;                // 屏幕显示标志位置1
// }

// /***周期中断2服务程序***/
// void pit0_ch2_isr()
// {
//     pit_isr_flag_clear(PIT_CH2); // 清除中断标志
// }

// **************************** 代码区域 ****************************