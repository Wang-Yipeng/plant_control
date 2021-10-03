#include "pid_control.h"
#include "string.h"

float PID_P=2,PID_I=1,PID_D=5;

extern int32_t PID_K1,PID_K2,PID_DK;
extern uint32_t pos_home;


extern TIM_HandleTypeDef htim2;

extern struct SAngle stcAngle;
extern double HomeHeading; 
extern short FloatHomeHeading;
extern double SetHeading; 

extern DAC_HandleTypeDef hdac;


extern unsigned char Ini_Flag;
extern unsigned char Home_Flag;
extern 	unsigned char plate_state;
extern 	unsigned char sensordata[8];
extern void uart_enc();

void platform_ini(){
	Ini_Flag=0;
	
	printf("开始初始化右转 \r\n");	
	
		if (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13)==GPIO_PIN_SET){printf("1 \r\n");Ini_Flag=1;};
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET); 		//打开转台电机使能
		while(Ini_Flag==0){
			
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,4000);
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,0);
			

		}
	
	printf("到达右极限 \r\n");	
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);	
	uart_enc();
	__HAL_TIM_SET_COUNTER(&htim2, 0);	
	while(Ini_Flag==1){
		
	  	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0);
	  	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,1000);

	}
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET); 		//关闭转台电机使能
		uart_enc();


}


void GoHome(){
	while(Home_Flag==0){
		int32_t abs_num=0;
		abs_num=pos_home-(uint32_t)(__HAL_TIM_GET_COUNTER(&htim2));
		if(abs_num>=0){
		}else{
			abs_num=-abs_num;
		}
			
	 pid_control(pos_home, PID_K1, PID_K2, PID_DK);
		if( abs_num<100){
			Home_Flag=1;
			//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET); //LED0亮
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0);
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,0);
			
			printf("到达零位 \r\n");
		}; 
		
	}
	

}

void pid_control(int32_t pos_control, int32_t PID_K1,int32_t PID_K2, int32_t PID_DK){	
	
	    int32_t PID_control=0;
		  static int32_t err=0,err_next=0,err_last=0;
			uint32_t enc4;
			enc4=(uint32_t)(__HAL_TIM_GET_COUNTER(&htim2));//获取定时器的值
			PID_K1=pos_control-enc4;			
			PID_DK=PID_K1+PID_K2;
			PID_control=   PID_P  *  PID_K1  + PID_D  *  PID_DK;
	
//			err=pos_control-enc4;	
//			PID_control=   PID_P  *  err  + PID_I  * (err+err_next+err_last)+ PID_D *(err-2*err_next+err_last);
	
	
	
		if (PID_control>0){
			
			if (PID_control>4040){PID_control=4040;};
					
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET); //LED0亮			

			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0);
		  
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,PID_control);
		
		} else if (PID_control<=0){
			
			if (PID_control<-4040){PID_control=-4040;};
					
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET); //LED0亮			

			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,-PID_control);
		  
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,0);
		
		}		
			//uart_enc();	  
			  
			PID_K2=PID_K1;
			err_last=err_next;
	        err_next=err;
		
  }

void plate_control(unsigned char ucData) {
	
			switch(ucData)//1判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据  
		//0x01 稳定平台停止运行
		//0x02 稳定平台开始运行
		//0x03 稳定平台回零位
		//0x04 稳定平台自检
		{
			case 0x01:	{
				    plate_state=plate_state&0x7F;
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET); 		//关闭转台电机使能
					HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0);    //关闭控制信号
					HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,0);	//关闭控制信号			

					printf("\r\n 收到指令，稳定平台停止运行,平台状态为%x \r\n" ,plate_state);
					break;
				}

			
			case 0x02:	{
					if((plate_state&0x80)==0x00){
						plate_state|= 1 << 7;	
						platform_ini();
						plate_state=plate_state&0x7F;
					//	HomeHeading=(float)(sensordata[6]*256+sensordata[7])/32768*180;
						
						
						
						plate_state|= 1 << 1;
						
						printf("\r\n 收到指令,稳定平台初始化完成,平台状态为 %x \r\n" ,plate_state);
						
					//	printf("\r\n HomeHeading为%f \r\n" ,HomeHeading);
						break;
					}
					else{
						printf("\r\n 收到开启平台指令，但未在停止状态，平台状态为%x \r\n" ,plate_state);
					break;
					}
				}
			
			
			case 0x03:	{
				
				if((plate_state&0x80)==0x00){
						plate_state|= 1 << 7;
						if ((plate_state&0x02)==0x02){
						Home_Flag=0;
					    printf("\r\n GoHome %x \r\n" ,plate_state);
						GoHome();
							
						memcpy(&FloatHomeHeading,&sensordata[4],2);	
						plate_state=plate_state&0x7F;
						plate_state|= 1 << 0;
						printf("\r\n 收到指令，稳定平台回零位,平台状态为%x \r\n" ,plate_state);
						break;
						}
						else{
						printf("\r\n 收到GoHome指令，但未初始化,平台状态为%x \r\n" ,plate_state);
						
						}
					}
					else{
						printf("\r\n 收到回零指令，但未在停止状态，平台状态为%x \r\n" ,plate_state);
						break;
					}

				}
		
			
			case 0x04:	{
					plate_state|= 1 << 7;
					printf("\r\n 收到指令，稳定平台开启,平台状态为%x \r\n" ,plate_state);
			
					break;
				}

		}

}

void plate_model(unsigned char ucData){

					if((plate_state&0x80)==0x00){
						plate_state&=0x0F;
						switch(ucData)
						{
							case 0x00:	plate_state|= 1 << 4;break;//按设定航向
							case 0x01:	plate_state|= 1 << 5;break;//按启动航向	
							case 0x02:	plate_state|= 1 << 6;break;//按飞行航向
							case 0x03:	plate_state|= 1 << 3;break;//跟随调试模式

						}
						printf("\r\n 设置稳定模式：%x \r\n" ,plate_state);
					}

}

void plate_run(){



}








