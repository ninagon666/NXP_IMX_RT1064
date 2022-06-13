#include "openartmini.h"

extern rt_sem_t next_control_sem;

uint8 test_rx_buffer;
lpuart_transfer_t   test_receivexfer;
lpuart_handle_t     test_g_lpuartHandle;

uint8 test_data;
int8_t rx_array[5] = {0, -1};

void example_uart_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
    rt_enter_critical();
    if (kStatus_LPUART_RxIdle == status) {
        static int rx_num = -1;
      
        //将结果存入rx_array
        if (test_rx_buffer == 0XFF) {
            rx_num = 0;
            rx_array[rx_num] = 0XFF;
            rx_num++;
        } 
        else if (rx_num < 5 && rx_num > 0) {
            rx_array[rx_num] = test_rx_buffer;
            rx_num++;
        }
        
        if (rx_num == 5) {
          rx_num = -1;
          rt_sem_release(next_control_sem);
        }
     }
    
    handle->rxDataSize = test_receivexfer.dataSize;  //还原缓冲区长度
    handle->rxData = test_receivexfer.data;          //还原缓冲区地址
    rt_exit_critical();
}

void openart_mini(void) 
{
    uart_init(USART_1, 115200,UART1_TX_B12,UART1_RX_B13);	
    NVIC_SetPriority(LPUART1_IRQn,10);         //设置串口中断优先级 范围0-15 越小优先级越高
    uart_rx_irq(USART_1,1);
    //配置串口接收的缓冲区及缓冲区长度
    test_receivexfer.dataSize = 1;
    test_receivexfer.data = &test_rx_buffer;
    //设置中断函数及其参数
    uart_set_handle(USART_1, &test_g_lpuartHandle, example_uart_callback, NULL, 0, test_receivexfer.data, 1);
}