#ifndef _bluetooth_h_
#define _bluetooth_h_

#define BT_UART_INDEX   (UART_2)
#define BT_UART_BAUDRATE    (115200)
#define BT_UART_TX_PIN    (UART2_TX_P10_5 )
#define BT_UART_RX_PIN    (UART2_RX_P10_6)
extern uint8 bluetooth_uart_get_data[64];                                                        // 串口接收数据缓冲区
extern uint8 bluetooth_fifo_get_data[64];                                                        // fifo 输出读出缓冲区

extern uint8  bluetooth_get_data;                                                            // 接收数据变量
extern uint32 bluetooth_fifo_data_count;                                                     // fifo 数据个数

extern fifo_struct bluetooth_uart_data_fifo;

void BlueTooth_Init(void);

#endif
