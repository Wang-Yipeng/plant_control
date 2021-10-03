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
	
	printf("��ʼ��ʼ����ת \r\n");	
	
		if (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13)==GPIO_PIN_SET){printf("1 \r\n");Ini_Flag=1;};
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET); 		//��ת̨���ʹ��
		while(Ini_Flag==0){
			
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,4000);
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,0);
			

		}
	
	printf("�����Ҽ��� \r\n");	
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);	
	uart_enc();
	__HAL_TIM_SET_COUNTER(&htim2, 0);	
	while(Ini_Flag==1){
		
	  	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0);
	  	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,1000);

	}
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET); 		//�ر�ת̨���ʹ��
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
			//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET); //LED0��
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0);
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,0);
			
			printf("������λ \r\n");
		}; 
		
	}
	

}

void pid_control(int32_t pos_control, int32_t PID_K1,int32_t PID_K2, int32_t PID_DK){	
	
	    int32_t PID_control=0;
		  static int32_t err=0,err_next=0,err_last=0;
			uint32_t enc4;
			enc4=(uint32_t)(__HAL_TIM_GET_COUNTER(&htim2));//��ȡ��ʱ����ֵ
			PID_K1=pos_control-enc4;			
			PID_DK=PID_K1+PID_K2;
			PID_control=   PID_P  *  PID_K1  + PID_D  *  PID_DK;
	
//			err=pos_control-enc4;	
//			PID_control=   PID_P  *  err  + PID_I  * (err+err_next+err_last)+ PID_D *(err-2*err_next+err_last);
	
	
	
		if (PID_control>0){
			
			if (PID_control>4040){PID_control=4040;};
					
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET); //LED0��			

			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0);
		  
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,PID_control);
		
		} else if (PID_control<=0){
			
			if (PID_control<-4040){PID_control=-4040;};
					
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET); //LED0��			

			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,-PID_control);
		  
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,0);
		
		}		
			//uart_enc();	  
			  
			PID_K2=PID_K1;
			err_last=err_next;
	        err_next=err;
		
  }

void plate_control(unsigned char ucData) {
	
			switch(ucData)//1�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������  
		//0x01 �ȶ�ƽֹ̨ͣ����
		//0x02 �ȶ�ƽ̨��ʼ����
		//0x03 �ȶ�ƽ̨����λ
		//0x04 �ȶ�ƽ̨�Լ�
		{
			case 0x01:	{
				    plate_state=plate_state&0x7F;
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET); 		//�ر�ת̨���ʹ��
					HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0);    //�رտ����ź�
					HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,0);	//�رտ����ź�			

					printf("\r\n �յ�ָ��ȶ�ƽֹ̨ͣ����,ƽ̨״̬Ϊ%x \r\n" ,plate_state);
					break;
				}

			
			case 0x02:	{
					if((plate_state&0x80)==0x00){
						plate_state|= 1 << 7;	
						platform_ini();
						plate_state=plate_state&0x7F;
					//	HomeHeading=(float)(sensordata[6]*256+sensordata[7])/32768*180;
						
						
						
						plate_state|= 1 << 1;
						
						printf("\r\n �յ�ָ��,�ȶ�ƽ̨��ʼ�����,ƽ̨״̬Ϊ %x \r\n" ,plate_state);
						
					//	printf("\r\n HomeHeadingΪ%f \r\n" ,HomeHeading);
						break;
					}
					else{
						printf("\r\n �յ�����ƽָ̨���δ��ֹͣ״̬��ƽ̨״̬Ϊ%x \r\n" ,plate_state);
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
						printf("\r\n �յ�ָ��ȶ�ƽ̨����λ,ƽ̨״̬Ϊ%x \r\n" ,plate_state);
						break;
						}
						else{
						printf("\r\n �յ�GoHomeָ���δ��ʼ��,ƽ̨״̬Ϊ%x \r\n" ,plate_state);
						
						}
					}
					else{
						printf("\r\n �յ�����ָ���δ��ֹͣ״̬��ƽ̨״̬Ϊ%x \r\n" ,plate_state);
						break;
					}

				}
		
			
			case 0x04:	{
					plate_state|= 1 << 7;
					printf("\r\n �յ�ָ��ȶ�ƽ̨����,ƽ̨״̬Ϊ%x \r\n" ,plate_state);
			
					break;
				}

		}

}

void plate_model(unsigned char ucData){

					if((plate_state&0x80)==0x00){
						plate_state&=0x0F;
						switch(ucData)
						{
							case 0x00:	plate_state|= 1 << 4;break;//���趨����
							case 0x01:	plate_state|= 1 << 5;break;//����������	
							case 0x02:	plate_state|= 1 << 6;break;//�����к���
							case 0x03:	plate_state|= 1 << 3;break;//�������ģʽ

						}
						printf("\r\n �����ȶ�ģʽ��%x \r\n" ,plate_state);
					}

}

void plate_run(){



}








