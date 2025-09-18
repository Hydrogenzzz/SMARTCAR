#include "global.h"
#include "BlueTooth.h"
#include "zf_common_headfile.h"

uint8 bluetooth_uart_get_data[64];                                                        // 串口接收数据缓冲区
uint8 bluetooth_fifo_get_data[64];                                                        // fifo 输出读出缓冲区

uint8  bluetooth_get_data = 0;                                                            // 接收数据变量
uint32 bluetooth_fifo_data_count = 0;                                                     // fifo 数据个数

fifo_struct bluetooth_uart_data_fifo;

void BlueTooth_Init(void)
{
    fifo_init(&bluetooth_uart_data_fifo, FIFO_DATA_8BIT, bluetooth_uart_get_data, 64);
    uart_init(BT_UART_INDEX, BT_UART_BAUDRATE, BT_UART_TX_PIN, BT_UART_RX_PIN);
    uart_rx_interrupt(BT_UART_INDEX, 1);
}
