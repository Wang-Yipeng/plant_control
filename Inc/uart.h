#ifndef __UART_H
#define __UART_H 	

#include "stm32f4xx_hal.h"
#include <string.h>


void uart_enc(void);
void CopeSerial1Data(unsigned char ucData1);//控制指令串口接收数据解析
void CopeSerial2Data(unsigned char ucData2);//POS数据串口接收数据解析

#define buff_size 100//buff容量,500条，以100Hz计算的话为5s的数据量

typedef struct buff
{
    uint16_t front;
    uint16_t rear;
    uint16_t lenth;
    double ring_buff[buff_size];
}queue;


uint8_t write_buff(queue ringbuff, double data);

uint8_t read_buff(queue ringbuff, uint8_t *rdata);

void init_buff(queue ringbuff);
void queen_init(void);



#endif
