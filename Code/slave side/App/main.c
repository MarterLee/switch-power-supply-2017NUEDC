#include "stm32f10x.h"
#include "eval.h"
#include <stdio.h>
#include "GUI.h"
#include "lcd.h"
#include "dac.h"
#include "pid.h"
#include "delay.h"
#include "cpwm.h"
#include "adc.h"
#include "key.h"
#include "caiyang.h"
#include "string.h"
#include "stm32f10x_exti.h"
#include "usart.h"
#include "usart2.h"
#include "common.h"

const u16 sindata[360]={900,914,929,944,959,974,988,1003,1018,1032,1047,1062,
	1076,1091,1105,1119,1134,1148,1162,1176,1190,1204,1218,1232,1245,1259,1272,
	1285,1299,1312,1325,1337,1350,1362,1375,1387,1399,1411,1423,1434,1446,1457,
	1468,1479,1490,1501,1511,1521,1531,1541,1551,1560,1569,1578,1587,1596,1604,
	1612,1620,1628,1636,1643,1650,1657,1663,1670,1676,1682,1688,1693,1698,1703,
	1708,1712,1717,1721,1724,1728,1731,1734,1737,1739,1741,1743,1745,1746,1747,
	1748,1749,1749,1750,1749,1749,1748,1747,1746,1745,1743,1741,1739,1737,1734,
	1731,1728,1724,1721,1717,1712,1708,1703,1698,1693,1688,1682,1676,1670,1663,
	1657,1650,1643,1636,1628,1620,1612,1604,1596,1587,1578,1569,1560,1551,1541,
	1531,1521,1511,1501,1490,1479,1468,1457,1446,1434,1423,1411,1399,1387,1375,
	1362,1350,1337,1325,1312,1299,1285,1272,1259,1245,1232,1218,1204,1190,1176,
	1162,1148,1134,1119,1105,1091,1076,1062,1047,1032,1018,1003,988,974,959,944,
	929,914,900,885,870,855,840,825,811,796,781,767,752,737,723,708,694,680,665,
	651,637,623,609,595,581,567,554,540,527,514,500,487,474,462,449,437,424,412,
	400,388,376,365,353,342,331,320,309,298,288,278,268,258,248,239,230,221,212,
	203,195,187,179,171,163,156,149,142,136,129,123,117,111,106,101,96,91,87,82,
	78,75,71,68,65,62,60,58,56,54,53,52,51,50,50,50,50,50,51,52,53,54,56,58,60,
	62,65,68,71,75,78,82,87,91,96,101,106,111,117,123,129,136,142,149,156,163,
	171,179,187,195,203,212,221,230,239,248,258,268,278,288,298,309,320,331,342,
	353,365,376,388,400,412,424,437,449,462,474,487,500,514,527,540,554,567,581,
	595,609,623,637,651,665,680,694,708,723,737,752,767,781,796,811,825,840,855,
870,885};  	 
/* ȫ�ֽṹ������ ---------------------------------------------------------*/
PID_InitTypeDef  PID1;//�������ٻ���
/*��������------------------------------------------------------*/
void TIM4_Cap_Init(u16 arr,u16 psc); //���벶����»�
int asctoint(u8 ch);             //asciiתint
int stringtoint(u8 buff[],int star,int len); //�ַ���תint
int leofnint(int num);        //��������
//ȫ�ֱ���   ���ж���ʾ����Ҫȫ�֣�
int add1=0; //ռ�ձȲ��ָ��1
int add2=119; //ռ�ձȲ��ָ��2
int add3=239; //ռ�ձȲ��ָ��3
int persent=1800;  //������Ȱٷֱ�
int setI2=800; //����ָ�����ADCת��ֵ
u16 adc1;        //��������ADCת��ֵ
int eer;         //PID�����������
int idex=0;      //��������������
int temp3;
int i=0;
int n=0;
u8 cmd[11];
int adc3; //����ֱ������
//���������
int main(void){
	
int temp0[8];    //ƽ�����������adc��Ŷ�Ӧ
int temp2[8];
u8 Key_Vlaue;   //��ֵ
int temp1;
int temp3;	
int e;         //PID�����������
//��ʾ�ַ���
char *p[]={"Uo:","I1:","I2:","Io:",".","V","A","setpf:","PWM1:","PWM2:","%","Io out of 2.5A","pf:"};
////             0	   1     2     3    4   5   6    7        8      9      10         11        12
	//PID1 ����    ����Դ�ȶ��� ���ڻ���
	PID1.uKP_Coe=0.001; // P
	PID1.uKI_Coe=0.1;  //I
	PID1.uKD_Coe=0.1;   //D
	PID1.maxmin=2;   //�������
	PID1.iPriVal=0;    //���ֵ
	PID1.detmax=800;    //���˼����ƫ�Χ
	PID1.okedge=30;       //����ƫ�Χ
	//��ʼ��
   GPIO_ResetBits(GPIOB,GPIO_Pin_4);  //����PB4�͵�ƽ

  SystemInit();//  ��ʼ��RCC ����ϵͳ��ƵΪ72MHZ
	delay_init(72);	     //��ʱ��ʼ��
	LCD_Init();	   //Һ������ʼ��
  KEY_Init();    //������ʼ��
	Adc_Init();     //ģ��ת����ʼ��
	Dac1_Init();     //��ģת��ͨ��һ��ʼ�� ���۲���λ��
	Dac2_Init();     //��ģת��ͨ��2��ʼ��  ������ָ��ģ�������
	//uart_init(115200);
	USART2_Init(115200);  //����2��ʼ��������ͨ�ţ�
	LCD_Clear(BLUE);          //����	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //�����жϷ���    
	Timer3_Init(8000,7199);      // ��ʾ�ж�0.8S����һ����Ļ
	Timer2_Init(3999 ,0);     // ����ж� 72M/240/50-1=4799 
  CPWM_Init(1799,0);       //40k
	TIM4_Cap_Init(65535,71);  //���벶��������λ
//LCD_ShowString(0,0,16,p[0],1);
//LCD_ShowString(80,0,16,p[5],1);
//LCD_ShowString(0,16,16,p[1],1);
//LCD_ShowString(80,16,16,p[6],1);
//LCD_ShowString(0,32,16,p[2],1);
//LCD_ShowString(80,32,16,p[6],1);
//LCD_ShowString(0,48,16,p[3],1);
//LCD_ShowString(80,48,16,p[6],1);
LCD_ShowString(0,64,16,p[12],1);
LCD_ShowString(0,80,16,p[7],1);
//LCD_ShowString(0,96,16,p[9],1);
LCD_ShowString(0,96,16,p[8],1);
//LCD_ShowString(0,112,16,p[9],1);
//LCD_ShowString(43,0,16,p[4],1);
//LCD_ShowString(43,16,16,p[4],1);
//LCD_ShowString(43,32,16,p[4],1);
//LCD_ShowString(43,48,16,p[4],1);
LCD_ShowString(43,64,16,p[4],1);
LCD_ShowString(56,80,16,p[4],1);
//GPIO_SetBits(GPIOD, GPIO_Pin_2);
for(i=0;i<10;i++)
{ 
u2_printf("A%d%d#",2,20); //���͸����˿�ʼ�ź�
	delay_ms(5);//��ʱ5ms
}
i=0;
	while(1)
	{
				switch(cmd[0])
				{
					case 0x41:  //A
						persent=stringtoint(cmd,2,asctoint(cmd[1])); //���յ������
					  GPIO_SetBits(GPIOB,GPIO_Pin_4);  //����PB4�ߵ�ƽ���򿪵�ſ��أ� 
					break;
					case 0x42:  //B
						//usenum2=stringtoint(cmd,12,asctoint(cmd[9])-2);
					break;
				  default:
						;					
			}
		//����������λ
		Key_Vlaue=KEY_Scan();
		if(Key_Vlaue==1)
		{
			n+=1;
		}
		if(Key_Vlaue==2)
		{  
			n-=1;
		}
	  adc1=Get_Adc(ADC_Channel_6,1);   //PA6  �����������
		temp3=caiyang(8,adc1,&temp2[8]);  //����ƽ��
		//���������˵�������ֵ
		if(i>5) //���ⷢ�͹���
		{
    u2_printf("B%d%d#",leofnint(temp3),temp3); //���͸�����ִ��
		i=0;
		}
		i++;
		//delay_ms(10);//���ⷢ��̫��
//		PID1.iSetVal=setI2;
//		PID1.iCurVal=temp3;       //���������˲ʱֵ
//		e=PID_Operation(&PID1);   //�������������������ֵ
	//	persent+=e;	
//		}	
//    else
//		{
//	  PID1.iSetVal=setI2;
//		PID1.iCurVal=temp3;       //���������˲ʱֵ
//		e=PID_Operation(&PID1);   //�������������������ֵ
//		persent+=e;
//		}			
//		}	
//					// �������	
//		if(i==1)
//		{
		// persent=1800;
//		}
//		
//	  if(i==2)
//	 {
//	  PID1.iSetVal=900;
//		PID1.iCurVal=adc3;       //���������˲ʱֵ
//		e=PID_Operation(&PID1);   //�������������������ֵ
//		persent+=e;	 	 
//	 }
		if(persent>1950)
			persent=1950 ; 		
		if(persent<300)
 			persent=300;	
}

	

}
//�ж���ʾ����
void TIM3_IRQHandler(void)   //TIM3�жϷ�����
{
	int showtemp;     //��ʾ�ݴ���
	//LCD_Fill(0,0,64,16,BLUE);	
	LCD_Clear(BLUE); //����
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
		{
	LCD_ShowNum(0,0,persent,8,16);
	LCD_ShowNum(0,16,adc1,8,16);
  LCD_ShowNum(0,32,n,8,16);	
//GPIO_SetBits(GPIOD, GPIO_Pin_2);			
	}
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //���TIMx���жϴ�����λ:TIM �ж�Դ 
}

//TIM2�жϴ��������������±�ָ��
void TIM2_IRQHandler(void) { 
    int temp;
if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
		{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ
    if(add1==359)			
		add1=0;			
		else
		add1+=1;
	 // Dac1_Set_Vol(sindata[add1]);//pA4
		if(add1<=239)
		add2=add1+120;
		else
		add2=add1-239;
		if(add2<=239)
		add3=add2+120;
		else
		add3=add2-239;  
		temp=(int)(sindata[add1]*persent/2000-(persent*9/20))+900;
		TIM_SetCompare3(TIM8,temp);//PC7��PB0���
		Dac1_Set_Vol(sindata[add1]);
		temp=(int)(sindata[add2]*persent/2000-(persent*9/20))+900;
		TIM_SetCompare2(TIM8,temp);//PC8��PB1���
		temp=(int)(sindata[add3]*persent/2000-(persent*9/20))+900;
		//Dac2_Set_Vol(sindata[add3]);
		TIM_SetCompare2(TIM1,temp);//PA9��PB14���0 
		//Dac2_Set_Vol(temp);
		}		
}
//ͬ����λ
void TIM4_Cap_Init(u16 arr,u16 psc) {   //TIM4CH1���벶��������ڳ�ʼ������
	GPIO_InitTypeDef GPIO_InitStructure;  
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
	NVIC_InitTypeDef NVIC_InitStructure; 
   TIM_ICInitTypeDef  TIM4_ICInitStructure;  //TIm4���벶�����ýṹ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //ʹ�� TIM4 ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ�� GPIOB ʱ��   
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_7;  //PB7 ���֮ǰ����    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //PB7 �������� 
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	GPIO_ResetBits(GPIOB,GPIO_Pin_7);   //PB6 ����   
	//��ʼ����ʱ�� 2 TIM2    
	TIM_TimeBaseStructure.TIM_Period = arr; //�趨�������Զ���װֵ   
	TIM_TimeBaseStructure.TIM_Prescaler =psc;  //Ԥ��Ƶ��    
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�   
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���  
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //��ʼ�� TIMx ��ʱ�������λ     
	//��ʼ�� TIM2 ���벶����� 
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2; //ѡ������� IC1 ӳ�䵽 TI1 ��    
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //�����ز���    
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽 TI1 ��    
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;  //���������Ƶ,����Ƶ     
	TIM4_ICInitStructure.TIM_ICFilter = 0xff;//IC1F=0000 ���������˲��� ���˲�    
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);	
	
	//��ʼ�� TIM2 ���벶����� 
//	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4; //ѡ������� IC1 ӳ�䵽 TI1 ��    
//	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //�����ز���    
//	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽 TI1 ��    
//	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;  //���������Ƶ,����Ƶ     
//	TIM4_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�    
//	TIM_ICInit(TIM4, &TIM4_ICInitStructure);	
	//�жϷ����ʼ��  
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4 �ж�  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ� 2 ��  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ� 0 ��  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ ͨ����ʹ��  
	NVIC_Init(&NVIC_InitStructure);  //��ʼ������ NVIC �Ĵ���     
	TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_CC2,ENABLE); //��������ж� CC1IE �����ж�
	//TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_CC4,ENABLE); //��������ж� CC1IE �����ж�
	TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2); 
	TIM_PWMIConfig(TIM4, &TIM4_ICInitStructure);   
  TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);  
  TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
	TIM_ClearITPendingBit(TIM4,TIM_IT_CC2|TIM_IT_Update);
	
  TIM_Cmd(TIM4,ENABLE );  //ʹ�ܶ�ʱ�� 4 	
}
//��ʱ��4�жϷ������	 
void TIM4_IRQHandler(void){ 
	
		if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET&&TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET )		 
		{	 
    add1=55+n; 
		GPIO_SetBits(GPIOB,GPIO_Pin_4);
		}
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC2|TIM_IT_Update); //����жϱ�־λ
}
// wifiģ���ʼ��
void initwifi()
{ 
	
	  atk_8266_send_cmd("AT","OK",200);  //���Խ���ATģʽ
		atk_8266_send_cmd("AT+CWMODE=2","OK",200);  ///����APģʽ
		atk_8266_send_cmd("AT+RST","OK",2000);   //������Ч
		atk_8266_send_cmd("AT+CIPAP=\"192.168.12.1\",\"192.168.12.1\",\"255.255.255.0\"","OK",300);  //����IP
		atk_8266_send_cmd("AT+CWSAP=\"ESP8266\",\"12345678\",1,4","OK",300); //����AP���ƣ�����
		atk_8266_send_cmd("AT+CIPMUX=1","OK",200); //����������
		atk_8266_send_cmd("AT+CIPSERVER=1,8086","OK",200);  //����������ģʽ���˿ں�8086
	
}
//stringתint����
// buff[]Ϊ�����ַ�����starΪ�ַ���ʼλ����һλҪת��������ascii����0��ʼ����lenΪ����λ��
//������ʵint������
//ע��������Ҫ�����ַ�������
//@LM 2017.07.24
int stringtoint(u8 buff[],int star,int len)
{
	int number=0;
	int idex=0;
	int time=1; //���Ʊ���
	for(idex=star+len-1;idex>=star;idex--)
	{
	number+=asctoint(buff[idex])*time;
	time*=10;  //10����		
	}
	return number;
	
}
//����asciiתint����
//@LM 2017.07.24
int asctoint(u8 ch)
{
switch(ch)
{
	case 0x30:
		return 0;
	case 0x31:
		return 1;
	case 0x32:
		return 2;
	case 0x33:
		return 3;
	case 0x34:
		return 4;
	case 0x35:
		return 5;
	case 0x36:
		return 6;
	case 0x37:
		return 7;
	case 0x38:
		return 8;
	case 0x39:
		return 9;
	default :
		return 0;
}
}
//�ж����������ȵĺ���
//����ֵ��0 �쳣��1��2��3��4��5����
//int�����Ϊ5λ
//@LM 2017.07.24
int leofnint(int num)
{
	if(num<0)
		return 0;
	else if(num>=0&&num<10)
		return 1;
	else if(num>=10&&num<100)
		return 2;
	else if(num>=100&&num<1000)
		return 3;
	else if(num>=1000&&num<10000)
		return 4;
	else
		return 5;
}
	void USART2_IRQHandler(void)
{
	u8 res;	
  int ok=0;	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//���յ�����
	{	 
 
	res =USART_ReceiveData(USART2);		
		if(USART2_RX_STA<USART2_MAX_RECV_LEN)		//�����Խ�������
		{
			TIM_SetCounter(TIM4,0);//���������        				 
			if(USART2_RX_STA==0)TIM5_Set(1);	 	//ʹ�ܶ�ʱ��4���ж� 
			USART2_RX_BUF[USART2_RX_STA++]=res;		//��¼���յ���ֵ	 
		}else 
		{
	 		USART2_RX_STA|=1<<15;					//ǿ�Ʊ�ǽ������
		} 
		USART_SendData(USART1,res);
		cmd[idex]=res;
		idex++;
		if(idex>10||res=='#')     // ���ݽ�����
		{
			idex=0;
			ok=1;
		}
		//printf("cmd:%s",cmd);
//		if(res=='A')
//			usenum1++;
//		if(res=='B')
//			usenum1--;
	} 
}   
