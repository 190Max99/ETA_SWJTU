#ifndef _REMOTE_H
#define _REMOTE_H
#include "zf_common_headfile.h"
#define UART_INDEX              (UART_4)                           // Ĭ�� UART_0
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)                           // Ĭ�� 115200
#define UART_TX_PIN             (UART4_TX_P14_1 )                           // Ĭ�� UART0_TX_P00_1
#define UART_RX_PIN             (UART4_RX_P14_0 )                           // Ĭ�� UART0_RX_P00_

extern int16 duty_servo;
extern int16 GOAL_SPEE_ST;
extern uint8 safe;
extern uint8 send_count;
extern uint8 go_flag;
extern double data_mid;
void remote_co(void);
void uart_rx433_read(void);
void uart_rx433_send(void);
void send_GPStocar1_LA();
void send_GPStocar1_LO();
void receive_data_GPS_LA();
void receive_data_GPS_LO();
#endif