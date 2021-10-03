#include "uart.h"
#include "math.h"
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern struct SAngle stcAngle;

extern void plate_control(unsigned char plate_controldata);
extern void plate_model(unsigned char plate_modeldata);
extern float PID_P,PID_I,PID_D;

void USART_BRR_Configuration(UART_HandleTypeDef *huart, uint32_t BaudRate);

extern unsigned char sensordata[8];
extern double SetHeading;

extern double diffheading;
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
	USART2->DR = (uint8_t) ch;      
	return ch;
}
#endif 

queue North_v_queen;
queue East_v_queen;

queue Azimuth_queue;

double rad2deg(double x) 
{ double y;
	y= x*180/3.141592653; 
	return y;
} 


uint8_t write_buff(queue ringbuff,double data)//入队
{
    if((ringbuff.rear+1)%buff_size==ringbuff.front)
    {
        return 0;
    }
    ringbuff.ring_buff[ringbuff.rear]=data;
    ringbuff.rear=(ringbuff.rear+1)%buff_size;
    ringbuff.lenth++;
	return 1;
}

uint8_t read_buff(queue ringbuff, uint8_t *rdata)//出队
{
    if(ringbuff.rear==ringbuff.front)
    {
        return 0;
    }
    *rdata=ringbuff.ring_buff[ringbuff.front];
    ringbuff.front=(ringbuff.front+1)%buff_size;
    ringbuff.lenth--;
	return 1;
}

void init_buff(queue ringbuff)//初始化队列
{
    ringbuff.front=0;
    ringbuff.rear=0;
    ringbuff.lenth=0;
}

void queen_init(void){
	
	init_buff(North_v_queen);
	init_buff(East_v_queen);
	init_buff(Azimuth_queue);

}



void USART_BRR_Configuration(UART_HandleTypeDef *huart, uint32_t BaudRate)
{
	uint32_t pclk;
	huart->Init.BaudRate = BaudRate;
	if (huart->Init.OverSampling == UART_OVERSAMPLING_8)
  	{
  		if ((huart->Instance == USART1) || (huart->Instance == USART6))
	    {
	      pclk = HAL_RCC_GetPCLK2Freq();
	      huart->Instance->BRR = UART_BRR_SAMPLING8(pclk, huart->Init.BaudRate);
	    }
	    else
	    {
	      pclk = HAL_RCC_GetPCLK1Freq();
	      huart->Instance->BRR = UART_BRR_SAMPLING8(pclk, huart->Init.BaudRate);
	    }
	}
	else
	{
		if ((huart->Instance == USART1) || (huart->Instance == USART6))
	    {
	      pclk = HAL_RCC_GetPCLK2Freq();
	      huart->Instance->BRR = UART_BRR_SAMPLING16(pclk, huart->Init.BaudRate);
	    }
	    else
	    {
	      pclk = HAL_RCC_GetPCLK1Freq();
	      huart->Instance->BRR = UART_BRR_SAMPLING16(pclk, huart->Init.BaudRate);
	    }
    }
}


////CopeSerial1Data为串口2中断调用函数，串口每收到一个数据，调用一次这个函数。
//void CopeSerial1Data(unsigned char ucData1)
//{
//		static unsigned char controlRxBuffer[20];
//		static unsigned char uc5RxCnt = 0;

//	//	if (uc5RxCnt>12){ uc5RxCnt=0;}
//	
//		controlRxBuffer[uc5RxCnt++]=ucData1;	//将收到的数据存入缓冲区中
//		if (controlRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
//		{
//			uc5RxCnt=0;

//			return;
//		} 
//		
//		if(uc5RxCnt<10){return;}//数据不满10个，则返回
//		else {
//			//sum校验	if(uc1RxBuffer[9]==(uc1RxBuffer[0]+uc1RxBuffer[1]+uc1RxBuffer[2]+uc1RxBuffer[3]+uc1RxBuffer[4]+uc1RxBuffer[5]+uc1RxBuffer[6]+uc1RxBuffer[7]+uc1RxBuffer[8]))
//		
//			HAL_UART_Transmit(&huart2, controlRxBuffer, 10, 1);
//		//	HAL_UART_Transmit(&huart3, controlRxBuffer, 10, 1);
//			switch(controlRxBuffer[2])//1判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据  
//			//0x51 稳定平台运动控制
//			//0x52 稳定平台跟踪模式设定
//			//0x53 稳定平台PID参数设定
//			{
//				case 0x51:	plate_control(controlRxBuffer[3]);break;
//				case 0x52:	plate_model(controlRxBuffer[3]);break;
//				case 0x53:	{	
//					short PPP,III,DDD;
//					memcpy(&PPP,&controlRxBuffer[3],2);
//					memcpy(&III,&controlRxBuffer[5],2);
//					memcpy(&DDD,&controlRxBuffer[7],2);
//					
//					printf("\r\n PPP %x  DDD %x  \r\n" ,PPP,DDD);
//					
//					PID_P=(float)(PPP)/10000;
//					PID_I=(float)(III)/10000;
//					PID_D=(float)(DDD)/10000;
//					printf("\r\n 收到setPID指令，PID_P：%f  PID_I：%f  PID_D：%f \r\n" ,PID_P,PID_I,PID_P);
//					break;}
//				case 0x54:	{ //设置串口2的波特率
//					switch(controlRxBuffer[3])
//					{
//						case 0x01:	USART_BRR_Configuration(&huart2, 460800);break;
//						case 0x02:	USART_BRR_Configuration(&huart2, 230400);break;
//						case 0x03:	USART_BRR_Configuration(&huart2, 115200);break;
//						case 0x04:	USART_BRR_Configuration(&huart2, 76800);break;
//						case 0x05:	USART_BRR_Configuration(&huart2, 57600);break;
//					}
//			
//					break;}
//				case 0x55:	{//设定航向
//					
//						SetHeading=(float)((uint32_t)(controlRxBuffer[3]*16777216+controlRxBuffer[4]*65535+controlRxBuffer[5]*256+controlRxBuffer[6])/4294967295*360);
//						printf("\r\n 设置航向：%f \r\n" ,SetHeading);
//						break;}

//			
//			}
//			uc5RxCnt=0;//清空缓存区
//		}

//}



//CopeSerial2Data为串口2中断调用函数，串口每收到一个数据，调用一次这个函数。
	
//	AA 44 13					//同步数据头
//	58							//数据长度              
//	FC 01						//消息ID
//	7C							//消息类型
//	08							//端口地址
//	68 6E						//消息长度
//	EC 0F						//Sequence
//	7C							//Idle Time
//	08							//Time Status
//	00 00						//Week
//	00 00 00 00					//ms
//	44 4E 10 41					//Receiver Status
//	00 00 00 00 00 00 00 00
//	00 00 00 00 00 00 00 00
//	00 00 00 00 00 00 00 00
//	00 00 00 00 00 00 00 00
//	00 00 00 00 00 00 00 00
//	00 00 00 00 00 00 00 00
//	00 00 00 00 00 00 00 00
//	00 00 00 00 00 00 00 00
//	00 00 00 00 00 00 00 00
//	00 00 00 00
//	58 1C 01 5C 
	



//AA 44 13
//58
//FC 01
//7C
//08
//38 BC
//0A 10
//7C
//08
//00 00
//00 00 00 00
//4C 6D 10 41
//24 BC 45 DD 8F 05 37 40
//23 EF 4D 8D DA 4E 5C 40
//00 00 88 CF 49 45 29 40
//48 D5 59 71 34 39 A4 3F
//78 E3 A1 C9 5B 66 74 BF
//4C E5 26 5C 57 3E B5 BF
//49 90 A0 15 2B C1 E1 BF
//E5 FC 7B D8 CC 3B DB BF
//00 00 00 00 00 00 00 80
//0A 00 00 00
//3C C3 1D 33 


void CopeSerial2Data(unsigned char ucData2)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;
	
	//HAL_UART_Transmit(&huart2, &ucData2, 1, 1);
//	LED_REVERSE();					//接收到数据，LED灯闪烁一下
	ucRxBuffer[ucRxCnt++]=ucData2;	//将收到的数据存入缓冲区中

	
	if (ucRxBuffer[0]!=0xAA) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	} 
 
	if (ucRxCnt<100) {return;}//数据不满11个，则返回
	
	
	else if(ucRxBuffer[0]==0xAA && ucRxBuffer[1]==0x44)
	{   
		//HAL_UART_Transmit(&huart2, ucRxBuffer, 10, 1);
		//HAL_UART_Transmit(&huart2, ucRxBuffer, 10, 1);
		
//		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
//		{
//			
////			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
////			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
////			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
//				case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);memcpy(&sensordata,&ucRxBuffer[2],8);	break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
////			case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);break;
////			case 0x55:	memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
////			case 0x56:	memcpy(&stcPress,&ucRxBuffer[2],8);break;
////			case 0x57:	memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
////			case 0x58:	memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
////			case 0x59:	memcpy(&stcQ,&ucRxBuffer[2],8);break;
//		}
		
		double latitude,longtitude,height,north_v,east_v,up_v,roll,pitch,azimuth;

		memcpy(&latitude,&ucRxBuffer[24],8);
		memcpy(&longtitude,&ucRxBuffer[32],8);
		memcpy(&height,&ucRxBuffer[40],8);
		
		memcpy(&north_v,&ucRxBuffer[48],8);
		memcpy(&east_v,&ucRxBuffer[56],8);
		memcpy(&up_v,&ucRxBuffer[64],8);
		
		memcpy(&roll,&ucRxBuffer[72],8);
		memcpy(&pitch,&ucRxBuffer[80],8);
		memcpy(&azimuth,&ucRxBuffer[88],8);
		
		uint8_t Status;
		memcpy(&Status,&ucRxBuffer[96],1);
		
		
		
		static double north_v_arr[buff_size],east_v_arr[buff_size],azimuth_arr[buff_size];
		static uint16_t arr_num = 0;
		if(Status==3){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET); //LED0亮
			//printf("\r\n ..................................................... \r\n");	
			if(arr_num<buff_size){			
				north_v_arr[arr_num]=north_v;
				east_v_arr[arr_num]=east_v;
				azimuth_arr[arr_num]=azimuth;
				
//			printf("\r\n latitude_arrA: %f %f %f %f %f  \r\n",latitude_arr[0],latitude_arr[1], 
//				latitude_arr[2],latitude_arr[3],latitude_arr[4]);
				
			arr_num++;
			}
			else{
				uint16_t i=0;
				while(i!=buff_size-1){
					north_v_arr[i]=north_v_arr[i+1];
					east_v_arr[i]=east_v_arr[i+1];
					azimuth_arr[i]=azimuth_arr[i+1];
					i++;					
				}
				north_v_arr[buff_size-1]=north_v;
				east_v_arr[buff_size-1]=east_v;
				azimuth_arr[buff_size-1]=azimuth;
				
//			printf("\r\n latitude_arrA: %f %f %f %f %f  \r\n",latitude_arr[0],latitude_arr[1], 
//				latitude_arr[2],latitude_arr[3],latitude_arr[4]);
				
			if((fabs(north_v)+fabs(east_v))>1){
				
				double North_Velocitys=0,East_Velocitys=0,Azimuths=0,North_Velocity=0,East_Velocity=0,Azimuth=0;
				double heading=0;
				for(int i=0; i<100; i++)
					{
						North_Velocitys=North_Velocitys+north_v_arr[i];
						East_Velocitys=East_Velocitys+east_v_arr[i];
						Azimuths=Azimuths+azimuth_arr[i];
						 }
					North_Velocity  = North_Velocitys/100;
					East_Velocity   = East_Velocitys/100;
						 Azimuth    = Azimuths/100;
						 
			    if (East_Velocity<0 && North_Velocity>=0){
					heading=270+rad2deg(atan(fabs(North_Velocity/East_Velocity)));
				}else if(East_Velocity==0 && North_Velocity>=0){
					heading=270;				
				}else
				
				if (East_Velocity<=0 && North_Velocity<0){
					heading=180+rad2deg(atan(fabs(East_Velocity/North_Velocity)));
				}
				
				if (East_Velocity>0 && North_Velocity<=0){
					heading=90+rad2deg(atan(fabs(North_Velocity/East_Velocity)));
				}else if(East_Velocity==0 && North_Velocity<=0){
					heading=90;				
				}else
				
				if (East_Velocity>=0 && North_Velocity>0){
					heading=rad2deg(atan(fabs(East_Velocity/North_Velocity)));
				}
				
				if(Azimuth-heading>=180){
				diffheading=Azimuth-heading-360;
				
				}else if(Azimuth-heading<=-180){
					diffheading=Azimuth-heading+360;
				}else{
								
					diffheading=Azimuth-heading;
				}

				
				//printf("\r\n East_Velocity: %f   \r\n",East_Velocity);
				//printf("\r\n North_Velocity: %f   \r\n",North_Velocity);
				//printf("\r\n Azimuth: %f   \r\n",Azimuth);
				//printf("\r\n heading: %f   \r\n",heading);
				//printf("\r\n diffheading: %f   \r\n",diffheading);
			
			
			}
			else{
				diffheading=0; 
			
			}
				
				

			}
			
		//printf("\r\n ..................................................... \r\n");	
						
		} 
		else{
		
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET); //LED0亮
		}
		
		if(Status==10|Status==12){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET); //LED0亮
						
//			//printf("\r\n ..................................................... \r\n");	
//			if(arr_num<buff_size){			
//				north_v_arr[arr_num]=north_v;
//				east_v_arr[arr_num]=east_v;
//				azimuth_arr[arr_num]=azimuth;
//				
////			printf("\r\n latitude_arrA: %f %f %f %f %f  \r\n",latitude_arr[0],latitude_arr[1], 
////				latitude_arr[2],latitude_arr[3],latitude_arr[4]);
//				
//			arr_num++;
//			}
//			else{
//				uint16_t i=0;
//				while(i!=buff_size-1){
//					north_v_arr[i]=north_v_arr[i+1];
//					east_v_arr[i]=east_v_arr[i+1];
//					azimuth_arr[i]=azimuth_arr[i+1];
//					i++;					
//				}
//				north_v_arr[buff_size-1]=north_v;
//				east_v_arr[buff_size-1]=east_v;
//				azimuth_arr[buff_size-1]=azimuth;
//				
////			printf("\r\n latitude_arrA: %f %f %f %f %f  \r\n",latitude_arr[0],latitude_arr[1], 
////				latitude_arr[2],latitude_arr[3],latitude_arr[4]);
//				
//			if((fabs(north_v)+fabs(north_v))>1){
//				
//				double North_Velocitys,East_Velocitys,Azimuths,North_Velocity,East_Velocity,Azimuth;
//				double heading;
//				for(int i=0; i<100; i++)
//					{
//						North_Velocitys=North_Velocitys+north_v_arr[i];
//						East_Velocitys=East_Velocitys+east_v_arr[i];
//						Azimuths=azimuth_arr[i];
//						 }
//					North_Velocity  = North_Velocitys/100;
//					East_Velocity   = East_Velocitys/100;
//						 Azimuth    = Azimuths/100;
//						 
//			    if (East_Velocity<=0 && North_Velocity>=0){
//					heading=270+rad2deg(atan(fabs(East_Velocity/North_Velocity)));
//				}
//				
//				if (East_Velocity<=0 && North_Velocity<=0){
//					heading=180+rad2deg(atan(fabs(East_Velocity/North_Velocity)));
//				}
//				
//				if (East_Velocity>=0 && North_Velocity<=0){
//					heading=90+rad2deg(atan(fabs(East_Velocity/North_Velocity)));
//				}
//				
//				if (East_Velocity>=0 && North_Velocity>=0){
//					heading=rad2deg(atan(fabs(East_Velocity/North_Velocity)));
//				}
//				
//				
//				diffheading=Azimuth-heading;  
//				printf("\r\n diffheading: %f   \r\n",diffheading);
//			
//			
//			}
//				
//				

//			}
			
		//printf("\r\n ..................................................... \r\n");	
		
		
		} 
		else{
		
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET); //LED0亮
		}
		
		
//		printf("\r\n latitude: %0.12f \r\n",latitude);
//		printf("\r\n longtitude: %0.12f \r\n",longtitude);
//		printf("\r\n height: %0.12f \r\n",height);
//		
//		printf("\r\n north_v: %0.12f \r\n",north_v);
//		printf("\r\n east_v: %0.12f \r\n",east_v);
//		printf("\r\n up_v: %0.12f \r\n",up_v);
//		
//		printf("\r\n roll: %0.12f \r\n",roll);
//		printf("\r\n pitch: %0.12f \r\n",pitch);
//		printf("\r\n athium: %0.12f \r\n",athium);
//		printf("\r\n Status: %d \r\n",Status);
////	
//		
		
		ucRxCnt=0;//清空缓存区

		
		
	}
}



void uart_enc(){

	uint32_t enc4;
	enc4=(uint32_t)(__HAL_TIM_GET_COUNTER(&htim2));//获取定时器的值
	printf("\r\n 当前脉冲数为:%d \r\n",enc4);
	
}

