#include "remote.h"
int16 duty_servo = 750; // 上限890 下限610
int16 GOAL_SPEE_ST = 0;
uint8 safe = 0; // 安全开关
uint8 z = 0;
uint8 j = 0;
uint8 go_flag = 0;
uint8 send_count = 0;

uint8 read_data[10]; // 串口接收数据缓冲区
uint8 get_data = 0;  // 接收数据变量

uint8 read_data2[10];       // 串口接收数据缓冲区
uint8 get_data2 = 0;        // 接收数据变量
uint32 fifo_data_count = 0; // fifo 数据

double data_mid_2 = 0;
double data_mid = 0;
uint8 data_tran_GPS[10];
uint8 read_data_LA[10];
double LA_data[10];
double LA_data_mid = 0;
uint8 flag_LA = 0;

void remote_co(void)
{
    if (1 == uart_receiver.finsh_flag) // 帧完成标志判断
    {
        if (1 == uart_receiver.state) // 遥控器失控状态判断
        {
            // printf("CH1-CH6 data: ");
            // for(int i = 0; i < 6; i++)
            // {
            //     printf("%d ", uart_receiver.channel[i]);         // 串口输出6个通道数据
            // }
            // printf("\r\n");

            GOAL_SPEE_ST = uart_receiver.channel[1] - 1000;
            // duty_servo = 750 + (uart_receiver.channel[0] - 1000) / 8;
            if (uart_receiver.channel[3] <= 500)
            {
                safe = 0;
            }
            else if (uart_receiver.channel[3] >= 500 && uart_receiver.channel[3] <= 1000)
            {
                safe = 1;
            }
            else if (uart_receiver.channel[3] >= 1000)
            {
                safe = 2;
            }
        }
        else
        {
            //   printf("Remote control has been disconnected.\r\n"); // 串口输出失控提示
        }
        uart_receiver.finsh_flag = 0; // 帧完成标志复位
    }
}

void uart_rx433_read_fromcontrol(void)
{
    //    get_data = uart_read_byte(UART_INDEX);                                      // 接收数据 while 等待式 不建议在中断使用
    if (uart_query_byte(UART_INDEX, &get_data)) // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    {
        if (get_data == 0x66 && z == 0)
        {
            read_data[z] = get_data;
            z = 1;
        }
        else if (z == 1)
        {
            read_data[z] = get_data;
            z = 2;
        }
        else if ((get_data == read_data[1] ^ 0xff) && z == 2) /*get_data==(~read_data[1])&&*/
        {
            read_data[z] = get_data;
            z = 3;
        }
        else if (get_data == 0x88 && z == 3)
        {
            read_data[z] = get_data;
            light_num = read_data[1];
            z = 0;
            for (int w = 0; w < 10; w++)
            {
                read_data[w] = 0;
            }
        }
        else
        {
            z = 0;
            for (int t = 0; t < 10; t++)
            {
                read_data[t] = 0;
            }
        }
    }
}

void uart_rx433_send_fromcar1()
{
    if (uart_query_byte(UART_INDEX, &get_data2)) // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    {
        if (get_data2 == 0x44 && j == 0)
        {
            read_data2[z] = get_data2;
            j = 1;
        }
        else if (j == 1)
        {
            read_data2[z] = get_data2;
            j = 2;
        }
        else if (get_data2 == 0xbe && j == 2)
        {
            go_flag = read_data2[1];
            j = 0;
            for (int k = 0; k < 10; k++)
            {
                read_data2[k] = 0;
            }
        }
        else
        {
            j = 0;
            for (int t = 0; t < 10; t++)
            {
                read_data2[t] = 0;
            }
        }
    }
}

// void uart_rx433_send()
// {
//     if (start_send == 1)
//     {
//         uart_write_buffer(UART_INDEX, (uint8 *)"\x88", 1);
//         uart_write_byte(UART_INDEX, distance);
//         uart_write_byte(UART_INDEX, 0xcc);
//         send_count++;

//         if (send_count > 10)
//         {
//             start_send = 0;
//             send_count = 0;
//         }
//     }
// }

void send_GPStocar1_LA()
{
    uart_write_byte(UART_INDEX, 0xAA);
    // data_tran_GPS[0]=data_mid/(1000000000);
    for (int k = 1; k < 8; k++)
    {
        data_mid_2 = GPS_GET_LAT[k];
        data_mid = data_mid_2 * (1000000);
        data_mid = GPS_GET_LAT[k] * (1000000);
        uart_write_byte(UART_INDEX, 0xcc);
        uart_write_byte(UART_INDEX, k);

        data_tran_GPS[0] = (uint32)data_mid % (1000000000) / (100000000);
        data_tran_GPS[1] = (uint32)data_mid % (100000000) / (10000000);
        data_tran_GPS[2] = (uint32)data_mid % (10000000) / (1000000);
        data_tran_GPS[3] = (uint32)data_mid % (1000000) / (100000);
        data_tran_GPS[4] = (uint32)data_mid % (100000) / (10000);
        data_tran_GPS[5] = (uint32)data_mid % (10000) / (1000);
        data_tran_GPS[6] = (uint32)data_mid % (1000) / (100);
        data_tran_GPS[7] = (uint32)data_mid % (100) / 10;
        data_tran_GPS[8] = (uint32)data_mid % 10;

        uart_write_buffer(UART_INDEX, data_tran_GPS, 9);

        uart_write_byte(UART_INDEX, 0xdd);
    }
}

void send_GPStocar1_LO()
{
    uart_write_byte(UART_INDEX, 0x00);
    // data_tran_GPS[0]=data_mid/(1000000000);
    for (int k = 1; k < 8; k++)
    {
        data_mid_2 = GPS_GET_LAT[k];
        data_mid = data_mid_2 * (1000000);
        data_mid = GPS_GET_LOT[k] * 1000000;
        uart_write_byte(UART_INDEX, 0xcc);
        uart_write_byte(UART_INDEX, k);

        data_tran_GPS[0] = (uint32)data_mid % (1000000000) / (100000000);
        data_tran_GPS[1] = (uint32)data_mid % (100000000) / (10000000);
        data_tran_GPS[2] = (uint32)data_mid % (10000000) / (1000000);
        data_tran_GPS[3] = (uint32)data_mid % (1000000) / (100000);
        data_tran_GPS[4] = (uint32)data_mid % (100000) / (10000);
        data_tran_GPS[5] = (uint32)data_mid % (10000) / (1000);
        data_tran_GPS[6] = (uint32)data_mid % (1000) / (100);
        data_tran_GPS[7] = (uint32)data_mid % (100) / 10;
        data_tran_GPS[8] = (uint32)data_mid % 10;

        uart_write_buffer(UART_INDEX, data_tran_GPS, 9);

        uart_write_byte(UART_INDEX, 0xdd);
    }
}

void receive_data_GPS_LA()
{
    if (uart_query_byte(UART_INDEX, &get_data2)) // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    {
        if (get_data2 == 0xaa && j == 0)
        {
            j = 1;
        }
        else if (get_data2 == 0xcc && j == 1)
        {
            j = 2;
        }
        else if (j == 2)
        {
            flag_LA = get_data2;
            j = 3;
        }
        else if (j == 3)
        {
            read_data_LA[0] = get_data2;
            j = 4;
        }
        else if (j == 4)
        {
            read_data_LA[1] = get_data2;
            j = 5;
        }
        else if (j == 5)
        {
            read_data_LA[2] = get_data2;
            j = 6;
        }
        else if (j == 6)
        {
            read_data_LA[3] = get_data2;
            j = 7;
        }
        else if (j == 7)
        {
            read_data_LA[4] = get_data2;
            j = 8;
        }
        else if (j == 8)
        {
            read_data_LA[5] = get_data2;
            j = 9;
        }
        else if (j == 9)
        {
            read_data_LA[6] = get_data2;
            j = 10;
        }
        else if (j == 10)
        {
            read_data_LA[7] = get_data2;
            j = 11;
        }
        else if (j == 11)
        {
            read_data_LA[8] = get_data2;
            j = 12;
        }
        else if (j == 12 && get_data2 == 0xdd)
        {
            LA_data_mid = read_data_LA[0] * 100000000 + read_data_LA[1] * 10000000 + read_data_LA[2] * 1000000 + read_data_LA[3] * 100000 + read_data_LA[4] * 10000 + read_data_LA[5] * 1000 + read_data_LA[6] * 100 + read_data_LA[7] * 10 + read_data_LA[8];
            LA_data[flag_LA] = LA_data_mid / 1000000;
            GPS_GET_LAT[flag_LA] = LA_data[flag_LA];
            flash_union_buffer[flag_LA-1].float_type=GPS_GET_LAT[flag_LA];
            j = 0;
        }
        else
        {
            j=0;
        }
    }
}


void receive_data_GPS_LO()
{
    if (uart_query_byte(UART_INDEX, &get_data2)) // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    {
        if (get_data2 == 0x00 && j == 0)
        {
            j = 1;
        }
        else if (get_data2 == 0xcc && j == 1)
        {
            j = 2;
        }
        else if (j == 2)
        {
            flag_LA = get_data2;
            j = 3;
        }
        else if (j == 3)
        {
            read_data_LA[0] = get_data2;
            j = 4;
        }
        else if (j == 4)
        {
            read_data_LA[1] = get_data2;
            j = 5;
        }
        else if (j == 5)
        {
            read_data_LA[2] = get_data2;
            j = 6;
        }
        else if (j == 6)
        {
            read_data_LA[3] = get_data2;
            j = 7;
        }
        else if (j == 7)
        {
            read_data_LA[4] = get_data2;
            j = 8;
        }
        else if (j == 8)
        {
            read_data_LA[5] = get_data2;
            j = 9;
        }
        else if (j == 9)
        {
            read_data_LA[6] = get_data2;
            j = 10;
        }
        else if (j == 10)
        {
            read_data_LA[7] = get_data2;
            j = 11;
        }
        else if (j == 11)
        {
            read_data_LA[8] = get_data2;
            j = 12;
        }
        else if (j == 12 && get_data2 == 0xdd)
        {
            LA_data_mid = read_data_LA[0] * 100000000 + read_data_LA[1] * 10000000 + read_data_LA[2] * 1000000 + read_data_LA[3] * 100000 + read_data_LA[4] * 10000 + read_data_LA[5] * 1000 + read_data_LA[6] * 100 + read_data_LA[7] * 10 + read_data_LA[8];
            LA_data[flag_LA] = LA_data_mid / 1000000;
            GPS_GET_LOT[flag_LA] = LA_data[flag_LA];
            flash_union_buffer[flag_LA+9].float_type=GPS_GET_LOT[flag_LA];
            j = 0;
        }
        else
        {
            j=0;
        }
    }
}