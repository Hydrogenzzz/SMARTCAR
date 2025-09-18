#include "global.h"
#include "BlueTooth.h"
#include "zf_common_headfile.h"

uint8 bluetooth_uart_get_data[64];                                                        // 串口接收数据缓冲区
uint8 bluetooth_fifo_get_data[64];                                                        // fifo 输出读出缓冲区

uint8  bluetooth_get_data = 0;                                                            // 接收数据变量
uint32 bluetooth_fifo_data_count = 0;                                                     // fifo 数据个数

fifo_struct bluetooth_uart_data_fifo;

/*
函数名称：BlueTooth_Init
功能说明：初始化蓝牙通信模块
参数说明：无
函数返回：无
*/
void BlueTooth_Init(void)
{
    fifo_init(&bluetooth_uart_data_fifo, FIFO_DATA_8BIT, bluetooth_uart_get_data, 64);  // 初始化数据缓冲区FIFO
    uart_init(BT_UART_INDEX, BT_UART_BAUDRATE, BT_UART_TX_PIN, BT_UART_RX_PIN);         // 初始化UART通信端口
    uart_rx_interrupt(BT_UART_INDEX, 1);                                                // 使能UART接收中断
}
