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
 * File: $Id: porttimer.c,v 1.60 2013/08/13 15:07:05 Armink $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/

extern TIM_HandleTypeDef htim7;

//static struct rt_timer timer;
//static void prvvTIMERExpiredISR(void);
//static void timer_timeout_ind(void* parameter);

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
    htim7.Init.Period = 50 * usTim1Timerout50us;
	//htim7.Init.Period = 100000;
	//htim7.Init.Period = 1800;
	
	
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
	  printf(" HAL_TIM_Base_Init(&htim7) != HAL_OK");
    //_Error_Handler(__FILE__, __LINE__);
  }

  return TRUE;
}

inline void
vMBPortTimersEnable(  )
{
	htim7.Instance->CNT = 0;
	HAL_TIM_Base_Start_IT(&htim7);
}

inline void
vMBPortTimersDisable(  )
{
	HAL_TIM_Base_Stop_IT(&htim7);
}
