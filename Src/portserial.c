/*
 * FreeModbus Libary: RT-Thread Port
 * Copyright (C) 2013 Armink <armink.ztl@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.60 2013/08/13 15:07:05 Armink $
 */

#include "port.h"
#include "stm32f4xx_hal.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
//#include "rtdevice.h"
//#include "board.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;  //

/* ----------------------- Static variables ---------------------------------*/
#define UART_BAUD_RATE          115200
#define UART_BAUD_CALC(UART_BAUD_RATE,F_OSC) \
    ( ( F_OSC ) / ( ( UART_BAUD_RATE ) * 16UL ) - 1 )

//#define UART_UCSRB  UCSR0B
static uint8_t recv_byte;


void set_rs485_mode(rs485_mode_t mode)
{
//	if(mode == RS485_TX) {
//		HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
//		HAL_Delay(5);	
//	} else {
//		HAL_Delay(10);
//		HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);		
//	}
}

void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
	if(xRxEnable == TRUE) {
		__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
		HAL_UART_Receive_IT(&huart6, &recv_byte, 1);
		
		printf("__HAL_UART_ENABLE_IT \r\n");
		
		
	} else {
		__HAL_UART_DISABLE_IT(&huart6, UART_IT_RXNE);
		printf("__HAL_UART_DISABLE_IT \r\n");
		
	}
	
	if(xTxEnable == TRUE) {
		__HAL_UART_ENABLE_IT(&huart6, UART_IT_TC);
		set_rs485_mode(RS485_TX);
	} else {
		__HAL_UART_DISABLE_IT(&huart6, UART_IT_TC);	
	}
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
	UART_HandleTypeDef *cur_uart;
	
	if(ucPORT == 3) {
		cur_uart = &huart3;
	} else if(ucPORT == 6) {
		cur_uart = &huart6;
	} else if(ucPORT == 2) {
		cur_uart = &huart2;
	}	else {
		return FALSE;
	}			

	if(ucDataBits == 8) {
		cur_uart->Init.WordLength = UART_WORDLENGTH_8B;
	} else if(ucDataBits == 9) {
		cur_uart->Init.WordLength = UART_WORDLENGTH_9B;
	} else {
		return FALSE;
	}
	
	if(eParity == MB_PAR_NONE) {
		cur_uart->Init.Parity = UART_PARITY_NONE;
	} else if(eParity == MB_PAR_ODD) {
		cur_uart->Init.Parity = UART_PARITY_ODD;
	} else if(eParity == MB_PAR_EVEN) {
		cur_uart->Init.Parity = UART_PARITY_EVEN;
	} else {
		return FALSE;
	}
	
	cur_uart->Init.BaudRate = ulBaudRate;

  if (HAL_UART_Init(cur_uart) != HAL_OK)
  {
    return FALSE;
  }
	
	return TRUE;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
	printf("===send:%02x\r\n", ucByte);
	
	HAL_UART_Transmit_IT(&huart6, (uint8_t *)&ucByte, 1);
	
	return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
	*pucByte = recv_byte;
	//HAL_UART_Receive(&huart2, &recv_byte, 1, 100);
	
	//recv_byte=(unsigned char)USART1->DR;
	//printf("x \r\n");
	HAL_UART_Receive_IT(&huart6, &recv_byte, 1);
	
	return TRUE;
}
