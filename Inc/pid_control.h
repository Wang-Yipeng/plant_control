#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H 	

#include "stm32f4xx_hal.h"


void platform_ini(void);
void GoHome(void);

void pid_control(int32_t pos_control,int32_t PID_K1,int32_t PID_K2, int32_t PID_DK);
void plate_control(unsigned char ucData);
void plate_model(unsigned char ucData);
void plate_run(void);


#endif
