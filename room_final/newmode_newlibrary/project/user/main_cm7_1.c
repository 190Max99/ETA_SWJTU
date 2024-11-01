/*********************************************************************************************************************
 * CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
 * Copyright (c) 2022 SEEKFREE ��ɿƼ�
 *
 * ���ļ��� CYT4BB ��Դ���һ����
 *
 * CYT4BB ��Դ�� ��������
 * �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
 * �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
 *
 * ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
 * ����û�������������Ի��ʺ��ض���;�ı�֤
 * ����ϸ����μ� GPL
 *
 * ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
 * ���û�У������<https://www.gnu.org/licenses/>
 *
 * ����ע����
 * ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
 * �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
 * ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
 * ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
 *
 * �ļ�����          main_cm7_1
 * ��˾����          �ɶ���ɿƼ����޹�˾
 * �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
 * ��������          IAR 9.40.1
 * ����ƽ̨          CYT4BB
 * ��������          https://seekfree.taobao.com/
 *
 * �޸ļ�¼
 * ����              ����                ��ע
 * 2024-1-4       pudding            first version
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "arm_math.h"
#include "math.h"

// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������

// **************************** �������� ****************************
/***��ʱ���ж�***/
uint8 ShowFlag = 0; // ��Ļ��ʾ��־λ

/***�����źŲɼ�***/
#define ADC_CHANNEL1 (ADC1_CH09_P12_5) // �궨��P12.5ΪADC_CHANNEL1
#define ADC_CHANNEL2 (ADC1_CH08_P12_4) // �궨��P12.4ΪADC_CHANNEL2
#define ADC_CHANNEL3 (ADC1_CH25_P14_5) // �궨��P14.5ΪADC_CHANNEL3
#define ADC_CHANNEL4 (ADC1_CH00_P10_4) // �궨��P10.4ΪADC_CHANNEL4

/***������㷨***/
#define FFT_SIZE 2048                  // FFT���㳤��Ϊ2048
#define MIC_RAW_DATA_LEN 2500          // ���ڲɼ������ѭ�����鳤�ȣ������FFT_SIZE����ֹ�����쳣��
int16 mic1_raw_data[MIC_RAW_DATA_LEN]; // �ɼ�����1���ݵ�ѭ������
int16 mic2_raw_data[MIC_RAW_DATA_LEN]; // �ɼ�����2���ݵ�ѭ������
int16 mic3_raw_data[MIC_RAW_DATA_LEN]; // �ɼ�����3���ݵ�ѭ������
int16 mic4_raw_data[MIC_RAW_DATA_LEN]; // �ɼ�����4���ݵ�ѭ������
int16 mic_raw_data_count = 0;          // �洢ѭ���������λ
int16 mic_raw_data_count_save = 0;     // �洢ѭ���������λ�ĵ�ǰֵ������ȡ�������FFT_SIZE�����ݣ�
float Mic1_fftIn[FFT_SIZE * 2];        // ��˷�Mic1����FFT��ʱ����������/����FFT֮��õ���Ƶ��������飨�����ź�Ϊ���������Գ���Ϊ FFT_SIZE * 2��
float Mic2_fftIn[FFT_SIZE * 2];        // ��˷�Mic2����FFT��ʱ����������/����FFT֮��õ���Ƶ��������飨�����ź�Ϊ���������Գ���Ϊ FFT_SIZE * 2��
float Mic3_fftIn[FFT_SIZE * 2];        // ��˷�Mic3����FFT��ʱ����������/����FFT֮��õ���Ƶ��������飨�����ź�Ϊ���������Գ���Ϊ FFT_SIZE * 2��
float Mic4_fftIn[FFT_SIZE * 2];        // ��˷�Mic4����FFT��ʱ����������/����FFT֮��õ���Ƶ��������飨�����ź�Ϊ���������Գ���Ϊ FFT_SIZE * 2��
float Mic13_fft_ConjMul[FFT_SIZE * 2]; // ��˷�Mic1����˷�Mic3Ƶ����˻�����
float Mic24_fft_ConjMul[FFT_SIZE * 2]; // ��˷�Mic2����˷�Mic4Ƶ����˻�����
float Mic13_Correlation[FFT_SIZE];     // ��˷�Mic1����˷�Mic3�����������
float Mic24_Correlation[FFT_SIZE];     // ��˷�Mic2����˷�Mic4�����������
float Mic13_max = 0;                   // �洢���������õ������ֵ
float Mic24_max = 0;                   // �洢���������õ������ֵ
int16 Mic13_Max_Array_Num = 0;         // ���������õ������ֵ��Ӧ�������±�
int16 Mic24_Max_Array_Num = 0;         // ���������õ������ֵ��Ӧ�������±�
int16 Mic13_Delay_Temp = 0;            // Mic1��Mic3��ʱ�Ӳ����Ч���ݣ�
int16 Mic24_Delay_Temp = 0;            // Mic2��Mic4��ʱ�Ӳ����Ч���ݣ�
int16 Last_Mic13_Delay = 0;            // ��һ��Mic1��Mic3����Чʱ�Ӳ�
int16 Last_Mic24_Delay = 0;            // ��һ��Mic2��Mic4����Чʱ�Ӳ�
int16 Mic13_Delay = 0;                 // Mic1��Mic3����Чʱ�Ӳ�
int16 Mic24_Delay = 0;                 // Mic2��Mic4����Чʱ�Ӳ�
uint8 Mic1_Chirp_Flag = 0;             // Mic1��Chirp�ź���Ч��־λ
uint8 Mic2_Chirp_Flag = 0;             // Mic1��Chirp�ź���Ч��־λ
uint8 Mic3_Chirp_Flag = 0;             // Mic1��Chirp�ź���Ч��־λ
uint8 Mic4_Chirp_Flag = 0;             // Mic1��Chirp�ź���Ч��־λ
uint8 ChirpFlag = 0;                   // Chirp�ź���Ч��־λ

uint8 start_count = 0;  // ��ʱ��־λ
uint16 time_count1 = 0; // ��¼ʱ��
uint16 time_count2 = 0; // ��ʾʱ��
uint32 data_to_cm70 = 0;
uint32 sound_en=0;

uint8 flag_m1=0;
uint8 flag_m2=0;
uint8 flag_m3=0;
uint8 flag_m4=0;


/***��Դ��λ***/
float Angle_rad = 0;
float Angle_deg = 0;

/***�˼�ͨ��***/
#define FLASH_SECTION_INDEX (0) // �洢 M7_1 ���ݵ�ַ������ �õ�����
#define FLASH_PAGE_INDEX (1)    // �洢 M7_1 ���ݳ��ȼ����� �õ�ҳ��
#define DATA_LENGTH (1)         // �������ݳ���
uint8 m7_1_data[DATA_LENGTH];   // ���� M7_1��������

#define DATA_LENGTH               (1)                                           // �������ݳ���

#pragma location = 0x28001000                                                   // ������������鶨�嵽ָ����RAM��ַ��������������ֱ�ӷ���(��Դ��Ĭ���� 0x28001000 ��ַ������8kb�Ŀռ��������ݽ���)
                                                                                // �˴�Ϊ0x28001014��ԭ����ǰ�����һ��M0������
uint8 m7_data[DATA_LENGTH] = {10};                        // ���� M7_1 ��ʾ�������� ����������


unsigned char send[15];

void my_ipc_callback(uint32 receive_data)
{
    // printf("receive M7_0 data:%d\r\n", receive_data);        // �����յ������ݴ�ӡ������
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); // ʱ�����ü�ϵͳ��ʼ��<��ر���>
                                   /* debug�����ɺ���M7_0��ʼ�����˴�����Ҫ�ٴγ�ʼ��debug���� */

    // �˴���д�û����� ���������ʼ�������
    /***�˼�ͨ�ų�ʼ��***/
    /*
    flash_init();            // flash��ʼ��(����flash֮ǰ��Ҫ��ʼ��һ��)

    flash_union_buffer[0].uint32_type = (uint32)&m7_1_data[0];      // ȡ�����׵�ַ
    flash_union_buffer[1].uint32_type = DATA_LENGTH;                // ȡ���鳤��

    flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);                    // д��flash֮ǰ��Ҫ������ǰҳ ������ܵ������ݴ���
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, 2);     // ����ַ�ͳ�����Ϣд��flash
    */

    SCB_DisableDCache(); // �ر�DCashe

    ipc_communicate_init(IPC_PORT_2, my_ipc_callback); // ��ʼ��IPCģ�� ѡ��˿�2 ��д�жϻص�����

    /***��ʱ���жϳ�ʼ��***/
    pit_us_init(PIT_CH0, 100); // ��ʼ��CCU6_0_CH0Ϊ�����ж�100us���ڣ���ADC����Ƶ��Ϊ10kHz(Chirp�����źŵ�Ƶ��Ϊ250~2000Hz)
    // pit_ms_init(PIT_CH1, 100); // ��ʼ�� CCU6_0_CH1 Ϊ�����ж� 200ms ����
    /***��Ļ��ʼ��***/
    // tft180_init(); // ��ʼ��2����Ļ
    /***ADC��ʼ��***/
    adc_init(ADC_CHANNEL1, ADC_12BIT); //  ��ʼ����Ӧ ADC ͨ��Ϊ��Ӧ���ȣ�12λ���ȣ�
    adc_init(ADC_CHANNEL2, ADC_12BIT); //  ��ʼ����Ӧ ADC ͨ��Ϊ��Ӧ���ȣ�12λ���ȣ�
    adc_init(ADC_CHANNEL3, ADC_12BIT); //  ��ʼ����Ӧ ADC ͨ��Ϊ��Ӧ���ȣ�12λ���ȣ�
    adc_init(ADC_CHANNEL4, ADC_12BIT); //  ��ʼ����Ӧ ADC ͨ��Ϊ��Ӧ���ȣ�12λ���ȣ�
 //   if (wireless_uart_init())
  //  {
  //      while (1)
  //      {
  //      }
  //  }
    /***�������ʾ������ʼ��***/
    //  seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART); // ��ʼ����ɴ������֣�ʹ��DEBUG_UART���������շ�
    //  seekfree_assistant_oscilloscope_struct oscilloscope_data;         // ʵ��������ʾ�����ṹ��
    //  oscilloscope_data.channel_num = 1;                                // ��������ʾ������ͨ��Ϊ1

    /***����***/
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

    // �˴���д�û����� ���������ʼ�������
    while (true)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���
        /***��Ļ��ʾ***/
        // if (ShowFlag == 1)
        // {
        //     ShowFlag = 0; // ��Ļ��ʾ��־λ��0

        //     tft180_show_int(0, 0, Mic13_Max_Array_Num, 5);  // ��ʾʱ�Ӳ�����
        //     tft180_show_int(0, 20, Mic24_Max_Array_Num, 5); // ��ʾʱ�Ӳ�����
        //     tft180_show_int(0, 40, time_count2, 5);      // ����һ�ε�ʱ��
        //     tft180_show_float(0, 60, Angle_deg, 3, 2);      // ��ʾ��λ������
        //     tft180_show_int(0, 80, Mic13_Delay, 5);         // ��ʾʱ�Ӳ�����
        //     tft180_show_int(0, 100, Mic24_Delay, 5);        // ��ʾʱ�Ӳ�����
        //     // tft180_show_int(0, 120, Last_Mic13_Delay, 5);   // ��ʾʱ�Ӳ�����
        //     // tft180_show_int(0, 140, Last_Mic24_Delay, 5);   // ��ʾʱ�Ӳ�����
        // }

        /***����ؼ��㼰��Դ��λ***/
        // ׼��FFT�������������
        start_count = 1;
        uint16 mic_raw_data_count_temp = 0;
        uint16 mic_data_count = 0;

        mic_raw_data_count_save = mic_raw_data_count; // ���浱ǰʱ�̵�ѭ���������λ������ȡ�������FFT_SIZE�����ݣ�

        if (mic_raw_data_count_save < FFT_SIZE) // ���浱ǰʱ�̵�ѭ���������λС��FFT_SIZE�������ѭ�����������ȡ�������FFT_SIZE������
        {
            mic_raw_data_count_temp = MIC_RAW_DATA_LEN - (FFT_SIZE - mic_raw_data_count_save); // ����ѭ�������˸��ƿ�ʼλ��

            // ѭ��������
            for (int16 i = mic_raw_data_count_temp; i < MIC_RAW_DATA_LEN; i++)
            {
                Mic1_fftIn[mic_data_count * 2] = mic1_raw_data[i]; // ������˷�1��ADֵ�浽ʵ�����鲿Ϊ0
                Mic1_fftIn[mic_data_count * 2 + 1] = 0;
                Mic2_fftIn[mic_data_count * 2] = mic2_raw_data[i]; // ������˷�2��ADֵ�浽ʵ�����鲿Ϊ0
                Mic2_fftIn[mic_data_count * 2 + 1] = 0;
                Mic3_fftIn[mic_data_count * 2] = mic3_raw_data[i]; // ������˷�3��ADֵ�浽ʵ�����鲿Ϊ0
                Mic3_fftIn[mic_data_count * 2 + 1] = 0;
                Mic4_fftIn[mic_data_count * 2] = mic4_raw_data[i]; // ������˷�4��ADֵ�浽ʵ�����鲿Ϊ0
                Mic4_fftIn[mic_data_count * 2 + 1] = 0;
                mic_data_count++;
            }
            // ѭ������ǰ��
            for (int16 i = 0; i < mic_raw_data_count_save; i++)
            {
                Mic1_fftIn[mic_data_count * 2] = mic1_raw_data[i]; // ������˷�1��ADֵ�浽ʵ�����鲿Ϊ0
                Mic1_fftIn[mic_data_count * 2 + 1] = 0;
                Mic2_fftIn[mic_data_count * 2] = mic2_raw_data[i]; // ������˷�2��ADֵ�浽ʵ�����鲿Ϊ0
                Mic2_fftIn[mic_data_count * 2 + 1] = 0;
                Mic3_fftIn[mic_data_count * 2] = mic3_raw_data[i]; // ������˷�3��ADֵ�浽ʵ�����鲿Ϊ0
                Mic3_fftIn[mic_data_count * 2 + 1] = 0;
                Mic4_fftIn[mic_data_count * 2] = mic4_raw_data[i]; // ������˷�4��ADֵ�浽ʵ�����鲿Ϊ0
                Mic4_fftIn[mic_data_count * 2 + 1] = 0;
                mic_data_count++;
            }
        }
        else // ���浱ǰʱ�̵�ѭ���������λ���ڵ���FFT_SIZE����ȡ��ѭ�������ǰFFT_SIZE������
        {
            for (int16 i = 0; i < FFT_SIZE; i++)
            {
                Mic1_fftIn[i * 2] = mic1_raw_data[i]; // ������˷�1��ADֵ�浽ʵ�����鲿Ϊ0
                Mic1_fftIn[i * 2 + 1] = 0;
                Mic2_fftIn[i * 2] = mic2_raw_data[i]; // ������˷�2��ADֵ�浽ʵ�����鲿Ϊ0
                Mic2_fftIn[i * 2 + 1] = 0;
                Mic3_fftIn[i * 2] = mic3_raw_data[i]; // ������˷�3��ADֵ�浽ʵ�����鲿Ϊ0
                Mic3_fftIn[i * 2 + 1] = 0;
                Mic4_fftIn[i * 2] = mic4_raw_data[i]; // ������˷�4��ADֵ�浽ʵ�����鲿Ϊ0
                Mic4_fftIn[i * 2 + 1] = 0;
            }
        }

        // FFT����
        arm_cfft_instance_f32 arm_cfft_instance_f32_len_FFT_SIZE;            // ����FFT����
        arm_cfft_init_f32(&arm_cfft_instance_f32_len_FFT_SIZE, FFT_SIZE);    // ��ʼ��FFT���󣬸�����㳤��
        arm_cfft_f32(&arm_cfft_instance_f32_len_FFT_SIZE, Mic1_fftIn, 0, 1); // ��˷�1��32λ����FFT���㣬�����λ��
        arm_cfft_f32(&arm_cfft_instance_f32_len_FFT_SIZE, Mic2_fftIn, 0, 1); // ��˷�2��32λ����FFT���㣬�����λ��
        arm_cfft_f32(&arm_cfft_instance_f32_len_FFT_SIZE, Mic3_fftIn, 0, 1); // ��˷�3��32λ����FFT���㣬�����λ��
        arm_cfft_f32(&arm_cfft_instance_f32_len_FFT_SIZE, Mic4_fftIn, 0, 1); // ��˷�4��32λ����FFT���㣬�����λ��

        // for (int i = 0; i < FFT_SIZE*2; i++)
        // {
        //     // oscilloscope_data.data[0] = Mic13_Correlation[i];
        //     // seekfree_assistant_oscilloscope_send(&oscilloscope_data); // ���������ʾ������������
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

        // �����˲��������ͨ��
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
        //     // seekfree_assistant_oscilloscope_send(&oscilloscope_data); // ���������ʾ������������
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

        Mic1_Chirp_Flag = 0; // ������0
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
        

        // �������ʾ����
        // for (int i = 0; i < FFT_SIZE * 2; i++)
        // {
        //     oscilloscope_data.data[0] = Mic1_fftIn[i];
        //     seekfree_assistant_oscilloscope_send(&oscilloscope_data); // ���������ʾ������������
        // }

        // ��˷�1Ƶ������˷�3Ƶ����ĳ˻�����˷�2Ƶ������˷�4Ƶ����ĳ˻�
        for (int i = 0; i < FFT_SIZE; i++)
        {
            Mic3_fftIn[2 * i + 1] = -Mic3_fftIn[2 * i + 1]; // ����˷�3Ƶ����й���
            Mic4_fftIn[2 * i + 1] = -Mic4_fftIn[2 * i + 1]; // ����˷�4Ƶ����й���
        }
        arm_cmplx_mult_cmplx_f32(Mic1_fftIn, Mic3_fftIn, Mic13_fft_ConjMul, FFT_SIZE);
        arm_cmplx_mult_cmplx_f32(Mic2_fftIn, Mic4_fftIn, Mic24_fft_ConjMul, FFT_SIZE);

        // ����˻���32λ����IFFT���㣬�����λ��
        arm_cfft_f32(&arm_cfft_instance_f32_len_FFT_SIZE, Mic13_fft_ConjMul, 1, 1);
        arm_cfft_f32(&arm_cfft_instance_f32_len_FFT_SIZE, Mic24_fft_ConjMul, 1, 1);

        // ���������
        arm_cmplx_mag_f32(Mic13_fft_ConjMul, Mic13_Correlation, FFT_SIZE);
        arm_cmplx_mag_f32(Mic24_fft_ConjMul, Mic24_Correlation, FFT_SIZE);

        // ����λ��
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

        // ���ֵ��0
        Mic13_max = 0;
        Mic24_max = 0;

        // �ҳ��������õĵ��Ӧ�������±�
        for (int i = 0; i < FFT_SIZE; i++)
        {
            if (Mic13_Correlation[i] > Mic13_max)
            {
                Mic13_max = Mic13_Correlation[i]; // �ҳ���ֵ���ĵ�,�˼�Ϊ�������õĵ�
                Mic13_Max_Array_Num = i;          // �������õĵ��Ӧ�������±�
            }

            if (Mic24_Correlation[i] > Mic24_max)
            {
                Mic24_max = Mic24_Correlation[i]; // �ҳ���ֵ���ĵ�,�˼�Ϊ�������õĵ�
                Mic24_Max_Array_Num = i;          // �������õĵ��Ӧ�������±�
            }
        }
        /*�����Ƚ��յ��źŵ�˳������ֵ�ֱ�Ϊ����0�Ϳ���2048*/

        if (Mic13_Max_Array_Num > 11) // �������2048���ȥ2048
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

        /*�˳���Ч����(������������Ч����)*/
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
        start_count = 0; // ֹͣ����

        // ���˳���Чʱ�Ӳ�
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

        // ���㷽λ��
        // C������float atan2f(float, float)���ص���ԭ������(x,y)�ķ�λ�ǣ�����x��ļнǡ�����ֵ�ĵ�λΪ���ȣ�ȡֵ��ΧΪ��-��, ��]��
        // ���Ϊ����ʾ�� X ����ʱ����ת�ĽǶȣ����Ϊ����ʾ�� X ��˳ʱ����ת�ĽǶȡ�
        // ��Ҫ�öȱ�ʾ������ֵ���뽫����ٳ��� 180/�С�����Ҫע����ǣ�����atan2f(y,x)�в�����˳���ǵ��õģ�atan2f(y,x)�����ֵ�൱�ڵ�(x,y)�ĽǶ�ֵ��
        Angle_rad = atan2f((float)Mic24_Delay, (float)Mic13_Delay);
        Angle_deg = Angle_rad * 180 / PI;
        if (Angle_deg < 0)
            Angle_deg += 360; // ������ת��Ϊ����
        data_to_cm70 = Angle_deg;
        ipc_send_data(data_to_cm70); // �������ݸ�����M7_0
        time_count2 = time_count1;   // ��¼������һ�εĽ����ʱ��
        // �˼�ͨ��
        // m7_1_data[0] = WeightMax_Array_Num;     //�����ݴ�����������
        //  M7_1������Dcache �������б仯ʱӦ�ø���Dcache������ ���������޷�ͬ����RAM(�������ķ��ʵ�RAM��ַҲ���޷���ȡ������)
        // SCB_CleanInvalidateDCache();
        // wireless_uart_send_string(("zhiguoxin:%d, %d\n", acc_speed1, goal_speed);
        // �������ʾ����
        // for (int i = 0; i < FFT_SIZE; i++)
        // {
        //     // oscilloscope_data.data[0] = Mic13_Correlation[i];
        //     // seekfree_assistant_oscilloscope_send(&oscilloscope_data); // ���������ʾ������������
        //     sprintf(send,"$:%f\n",Mic13_Correlation[i]);
        //     wireless_uart_send_buffer(send,10);
        // }

        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}

/***�����ж�0������򣨶�ȡ��˷��ADֵ�浽ѭ�������У�***/
void pit0_ch0_isr()
{
    pit_isr_flag_clear(PIT_CH0); // ����жϱ�־

    mic1_raw_data[mic_raw_data_count] = adc_convert(ADC_CHANNEL1) - 2000; // ��ȡ��˷�1��ADֵ����ȥֱ��ƫ��
    mic2_raw_data[mic_raw_data_count] = adc_convert(ADC_CHANNEL2) - 2000; // ��ȡ��˷�2��ADֵ����ȥֱ��ƫ��
    mic3_raw_data[mic_raw_data_count] = adc_convert(ADC_CHANNEL3) - 2000; // ��ȡ��˷�3��ADֵ����ȥֱ��ƫ��
    mic4_raw_data[mic_raw_data_count] = adc_convert(ADC_CHANNEL4) - 2000; // ��ȡ��˷�4��ADֵ����ȥֱ��ƫ��

    mic_raw_data_count++; // �ɼ�������1

    if (mic_raw_data_count >= 2500) // Ϊ��һ��ADC������׼��
    {
        mic_raw_data_count = 0;
    }
    if (start_count)
    {
        time_count1++;
    } // ��¼����ʱ��
    else
        time_count1 = 0;
}

// /***�����ж�1���������Ļ��ʾ��־λ��λ��***/
// void pit0_ch1_isr()
// {
//     pit_isr_flag_clear(PIT_CH1); // ����жϱ�־
//     ShowFlag = 1;                // ��Ļ��ʾ��־λ��1
// }

// /***�����ж�2�������***/
// void pit0_ch2_isr()
// {
//     pit_isr_flag_clear(PIT_CH2); // ����жϱ�־
// }

// **************************** �������� ****************************