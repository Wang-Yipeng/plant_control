#include "uart.h"
#include "math.h"
#include "oem7crc32.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern struct SAngle stcAngle;

extern void plate_control(unsigned char plate_controldata);
extern void plate_model(unsigned char plate_modeldata);
extern float PID_P,PID_I,PID_D;

void USART_BRR_Configuration(UART_HandleTypeDef *huart, uint32_t BaudRate);

extern unsigned char sensordata[8];
extern double SetHeading;

extern double diffheading;
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART2->SR&0X40)==0);//ѭ������,ֱ���������   
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


uint8_t write_buff(queue ringbuff,double data)//���
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

uint8_t read_buff(queue ringbuff, uint8_t *rdata)//����
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

void init_buff(queue ringbuff)//��ʼ������
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



//CopeSerial2DataΪ����2�жϵ��ú���������ÿ�յ�һ�����ݣ�����һ�����������
	
//	AA 44 13				//ͬ������ͷ
//	58							//���ݳ���   Uchar       Message length, not including header or CRC  0x58---88        
//	FC 01						//��ϢID     Ushort
//	7C 08						//GNSSWeek   Ushort
//	68 6E	EC 0F			//Milliseconds from the beginning of the GNSS week    GPSec

//	7C 08	00 00     //GNSS Week      Ulong
//  00 00	00 00 44 4E 10 41		//Seconds from week start    Double
//	00 00 00 00 00 00 00 00   //γ�� 	  Double
//	00 00 00 00 00 00 00 00   //���� 	  Double
//	00 00 00 00 00 00 00 00   //�߶� 	  Double
//	00 00 00 00 00 00 00 00   //����  	Double
//	00 00 00 00 00 00 00 00   //���� 	  Double
//	00 00 00 00 00 00 00 00   //���� 	  Double
//	00 00 00 00 00 00 00 00   //Roll  	Double
//	00 00 00 00 00 00 00 00   //Pitch   Double
//	00 00 00 00 00 00 00 00   //Azimuth    Double
//	00 00 00 00               //Status   	 Enum 

//	58 1C 01 5C                 //32-bitCRC
	
/////////////////////INSPVASBһ������/////////////////һ֡��Ϣ�ܳ���12+88+4=104///////////////////////////////////////
//AA 44 13
//58
//FC 01
//7C 08
//38 BC 0A 10
//7C 08 00 00
//00 00 00 00 4C 6D 10 41
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

////////////////////INSSPDSBһ������////////////////һ֡��Ϣ�ܳ���12+40+4=56//////////////
//	AA 44 13				//ͬ������ͷ
//	28							//���ݳ���   Uchar       Message length, not including header or CRC  0x58---88        
//	43 01						//��ϢID     Ushort   0x0143---323
//	7C 08						//GNSSWeek   Ushort
//	68 6E	EC 0F			//Milliseconds from the beginning of the GNSS week    GPSec

//	7C 08	00 00     //GNSS Week      Ulong
//  00 00	00 00 44 4E 10 41		//Seconds from week start    Double
//	00 00 00 00 00 00 00 00   //Trk_gnd	  Double
//	00 00 00 00 00 00 00 00   //Horizontal_Speed	  Double
//	00 00 00 00 00 00 00 00   //Vertical_Speed 	  Double
//	00 00 00 00               //Status   	 Enum 

//	58 1C 01 5C                 //32-bitCRC


//�������貨����
//(104+56)*8*200=256000

void CopeSerial2Data(unsigned char ucData2)
{
	static unsigned char ucRxBuffer[104];
	static unsigned char ucRxCnt = 0;
	
//	HAL_UART_Transmit(&huart2, &ucData2, 1, 1);
  //	LED_REVERSE();					//���յ����ݣ�LED����˸һ��
	ucRxBuffer[ucRxCnt++]=ucData2;	//���յ������ݴ��뻺������

	
	if (ucRxBuffer[0]!=0xAA) //����ͷ���ԣ������¿�ʼѰ��0xAA����ͷ
	{
		ucRxCnt=0;
		
		return;
	}
	if (ucRxCnt==2 && ucRxBuffer[0]==0xAA && ucRxBuffer[1]!=0x44) //����ͷ���ԣ������¿�ʼѰ��0xAA����ͷ
	{
		ucRxCnt=0;
		return;
	}	
		if (ucRxCnt==3 && ucRxBuffer[0]==0xAA && ucRxBuffer[1]==0x44 && ucRxBuffer[2]!=0x13) //����ͷ���ԣ������¿�ʼѰ��0xAA 0x44 0x13����ͷ
	{
		ucRxCnt=0;
		return;
	}
	
//		uint8_t K1_RxBuffer2[20]="�յ�ͬ������ͷ \r\n";
//		HAL_UART_Transmit(&huart2, K1_RxBuffer2,20,1000);
	
		double INSPVASB_INF_GPSsec,INSPVASB_INF_AZIMUTH;
		unsigned long INSPVASB_INF_GPSweek;
		
		double INSSPDSB_INF_GPSsec,INSSPDSB_INF_Trk_gnd,INSSPDSB_INF_Horizontal_Speed;
		unsigned long INSSPDSB_INF_GPSweek;
	if(ucRxBuffer[4]==0xFC && ucRxBuffer[5]==01 && ucRxCnt==104){ //���յ�INSPVASB����,���ݳ���Ӧ��Ϊ104���ֽ�
		

				memcpy(&INSPVASB_INF_GPSweek,&ucRxBuffer[12],4); //�õ�GPS��
		
				memcpy(&INSPVASB_INF_GPSsec,&ucRxBuffer[16],8);  //�õ�GPS������
		
				memcpy(&INSPVASB_INF_AZIMUTH,&ucRxBuffer[88],8);  //�õ���̬��


				ucRxCnt=0;
		
			//	HAL_UART_Transmit(&huart2, ucRxBuffer,104,1000);
	
		}
		
		if(ucRxBuffer[4]==0x43 && ucRxBuffer[5]==01 && ucRxCnt==56){ //���յ�INSPVASB����


		
				memcpy(&INSSPDSB_INF_GPSweek,&ucRxBuffer[12],4);  //�õ�GPS��
		
				memcpy(&INSSPDSB_INF_GPSsec,&ucRxBuffer[16],8);  //�õ�GPS������
			
				memcpy(&INSSPDSB_INF_Trk_gnd,&ucRxBuffer[24],8); //�õ�����
		
				memcpy(&INSSPDSB_INF_Horizontal_Speed,&ucRxBuffer[32],8);  //�õ�ˮƽ����
				
				if(fabs(INSSPDSB_INF_GPSsec-INSPVASB_INF_GPSsec)<0.01 && INSSPDSB_INF_Horizontal_Speed> 0.5){
					
					  if(INSPVASB_INF_AZIMUTH-INSSPDSB_INF_Trk_gnd>=180){
                diffheading=INSPVASB_INF_AZIMUTH-INSSPDSB_INF_Trk_gnd-360;

               }else if(INSPVASB_INF_AZIMUTH-INSSPDSB_INF_Trk_gnd<=-180){
                    diffheading=INSPVASB_INF_AZIMUTH-INSSPDSB_INF_Trk_gnd+360;
               }else{

                    diffheading=INSPVASB_INF_AZIMUTH-INSSPDSB_INF_Trk_gnd;
                }
				}else{
				
				diffheading=0;
				
				}
				
			//	HAL_UART_Transmit(&huart2, ucRxBuffer,56,1000);
				
				ucRxCnt=0;


		}
	
}



void uart_enc(){

	uint32_t enc4;
	enc4=(uint32_t)(__HAL_TIM_GET_COUNTER(&htim2));//��ȡ��ʱ����ֵ
	printf("\r\n ��ǰ������Ϊ:%d \r\n",enc4);
	
}

