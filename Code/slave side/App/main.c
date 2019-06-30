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
/* 全局结构体声明 ---------------------------------------------------------*/
PID_InitTypeDef  PID1;//电流跟踪环，
/*函数声明------------------------------------------------------*/
void TIM4_Cap_Init(u16 arr,u16 psc); //输入捕获出事化
int asctoint(u8 ch);             //ascii转int
int stringtoint(u8 buff[],int star,int len); //字符串转int
int leofnint(int num);        //整数长度
//全局变量   （中断显示所以要全局）
int add1=0; //占空比查表指针1
int add2=119; //占空比查表指针2
int add3=239; //占空比查表指针3
int persent=1800;  //调制深度百分比
int setI2=800; //电流指令采样ADC转化值
u16 adc1;        //电流采样ADC转化值
int eer;         //PID调节增量输出
int idex=0;      //缓冲区数据索引
int temp3;
int i=0;
int n=0;
u8 cmd[11];
int adc3; //输入直流电流
//主函数入口
int main(void){
	
int temp0[8];    //平滑数组序号与adc序号对应
int temp2[8];
u8 Key_Vlaue;   //键值
int temp1;
int temp3;	
int e;         //PID反馈误差增量
//显示字符库
char *p[]={"Uo:","I1:","I2:","Io:",".","V","A","setpf:","PWM1:","PWM2:","%","Io out of 2.5A","pf:"};
////             0	   1     2     3    4   5   6    7        8      9      10         11        12
	//PID1 配置    电流源稳定环 （内环）
	PID1.uKP_Coe=0.001; // P
	PID1.uKI_Coe=0.1;  //I
	PID1.uKD_Coe=0.1;   //D
	PID1.maxmin=2;   //输出极限
	PID1.iPriVal=0;    //输出值
	PID1.detmax=800;    //送人计算的偏差范围
	PID1.okedge=30;       //允许偏差范围
	//初始化
   GPIO_ResetBits(GPIOB,GPIO_Pin_4);  //设置PB4低电平

  SystemInit();//  初始化RCC 设置系统主频为72MHZ
	delay_init(72);	     //延时初始化
	LCD_Init();	   //液晶屏初始化
  KEY_Init();    //按键初始化
	Adc_Init();     //模数转换初始化
	Dac1_Init();     //数模转换通道一初始化 （观察相位）
	Dac2_Init();     //数模转换通道2初始化  （电流指令模拟输出）
	//uart_init(115200);
	USART2_Init(115200);  //串口2初始化（串口通信）
	LCD_Clear(BLUE);          //清屏	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //设置中断分组    
	Timer3_Init(8000,7199);      // 显示中断0.8S更新一次屏幕
	Timer2_Init(3999 ,0);     // 查表中断 72M/240/50-1=4799 
  CPWM_Init(1799,0);       //40k
	TIM4_Cap_Init(65535,71);  //输入捕获锁定相位
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
u2_printf("A%d%d#",2,20); //发送给主端开始信号
	delay_ms(5);//延时5ms
}
i=0;
	while(1)
	{
				switch(cmd[0])
				{
					case 0x41:  //A
						persent=stringtoint(cmd,2,asctoint(cmd[1])); //接收调制深度
					  GPIO_SetBits(GPIOB,GPIO_Pin_4);  //设置PB4高电平（打开电磁开关） 
					break;
					case 0x42:  //B
						//usenum2=stringtoint(cmd,12,asctoint(cmd[9])-2);
					break;
				  default:
						;					
			}
		//按键调整相位
		Key_Vlaue=KEY_Scan();
		if(Key_Vlaue==1)
		{
			n+=1;
		}
		if(Key_Vlaue==2)
		{  
			n-=1;
		}
	  adc1=Get_Adc(ADC_Channel_6,1);   //PA6  输出电流采样
		temp3=caiyang(8,adc1,&temp2[8]);  //滑动平均
		//发送至主端电流采样值
		if(i>5) //避免发送过快
		{
    u2_printf("B%d%d#",leofnint(temp3),temp3); //发送给主端执行
		i=0;
		}
		i++;
		//delay_ms(10);//避免发送太快
//		PID1.iSetVal=setI2;
//		PID1.iCurVal=temp3;       //输出电流的瞬时值
//		e=PID_Operation(&PID1);   //输出电流反馈电流增量值
	//	persent+=e;	
//		}	
//    else
//		{
//	  PID1.iSetVal=setI2;
//		PID1.iCurVal=temp3;       //输出电流的瞬时值
//		e=PID_Operation(&PID1);   //输出电流反馈电流增量值
//		persent+=e;
//		}			
//		}	
//					// 调制深度	
//		if(i==1)
//		{
		// persent=1800;
//		}
//		
//	  if(i==2)
//	 {
//	  PID1.iSetVal=900;
//		PID1.iCurVal=adc3;       //输出电流的瞬时值
//		e=PID_Operation(&PID1);   //输出电流反馈电流增量值
//		persent+=e;	 	 
//	 }
		if(persent>1950)
			persent=1950 ; 		
		if(persent<300)
 			persent=300;	
}

	

}
//中断显示函数
void TIM3_IRQHandler(void)   //TIM3中断服务函数
{
	int showtemp;     //显示暂存器
	//LCD_Fill(0,0,64,16,BLUE);	
	LCD_Clear(BLUE); //清屏
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{
	LCD_ShowNum(0,0,persent,8,16);
	LCD_ShowNum(0,16,adc1,8,16);
  LCD_ShowNum(0,32,n,8,16);	
//GPIO_SetBits(GPIOD, GPIO_Pin_2);			
	}
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源 
}

//TIM2中断处理函数，用来更新表指针
void TIM2_IRQHandler(void) { 
    int temp;
if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源
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
		TIM_SetCompare3(TIM8,temp);//PC7，PB0输出
		Dac1_Set_Vol(sindata[add1]);
		temp=(int)(sindata[add2]*persent/2000-(persent*9/20))+900;
		TIM_SetCompare2(TIM8,temp);//PC8，PB1输出
		temp=(int)(sindata[add3]*persent/2000-(persent*9/20))+900;
		//Dac2_Set_Vol(sindata[add3]);
		TIM_SetCompare2(TIM1,temp);//PA9，PB14输出0 
		//Dac2_Set_Vol(temp);
		}		
}
//同步相位
void TIM4_Cap_Init(u16 arr,u16 psc) {   //TIM4CH1输入捕获计算周期初始化函数
	GPIO_InitTypeDef GPIO_InitStructure;  
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
	NVIC_InitTypeDef NVIC_InitStructure; 
   TIM_ICInitTypeDef  TIM4_ICInitStructure;  //TIm4输入捕获设置结构体
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //使能 TIM4 时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能 GPIOB 时钟   
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_7;  //PB7 清除之前设置    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //PB7 下拉输入 
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	GPIO_ResetBits(GPIOB,GPIO_Pin_7);   //PB6 下拉   
	//初始化定时器 2 TIM2    
	TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值   
	TIM_TimeBaseStructure.TIM_Prescaler =psc;  //预分频器    
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割   
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数  
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //初始化 TIMx 的时间基数单位     
	//初始化 TIM2 输入捕获参数 
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2; //选择输入端 IC1 映射到 TI1 上    
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //上升沿捕获    
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到 TI1 上    
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;  //配置输入分频,不分频     
	TIM4_ICInitStructure.TIM_ICFilter = 0xff;//IC1F=0000 配置输入滤波器 不滤波    
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);	
	
	//初始化 TIM2 输入捕获参数 
//	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4; //选择输入端 IC1 映射到 TI1 上    
//	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //上升沿捕获    
//	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到 TI1 上    
//	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;  //配置输入分频,不分频     
//	TIM4_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波    
//	TIM_ICInit(TIM4, &TIM4_ICInitStructure);	
	//中断分组初始化  
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4 中断  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级 2 级  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级 0 级  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ 通道被使能  
	NVIC_Init(&NVIC_InitStructure);  //初始化外设 NVIC 寄存器     
	TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_CC2,ENABLE); //允许更新中断 CC1IE 捕获中断
	//TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_CC4,ENABLE); //允许更新中断 CC1IE 捕获中断
	TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2); 
	TIM_PWMIConfig(TIM4, &TIM4_ICInitStructure);   
  TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);  
  TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
	TIM_ClearITPendingBit(TIM4,TIM_IT_CC2|TIM_IT_Update);
	
  TIM_Cmd(TIM4,ENABLE );  //使能定时器 4 	
}
//定时器4中断服务程序	 
void TIM4_IRQHandler(void){ 
	
		if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET&&TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET )		 
		{	 
    add1=55+n; 
		GPIO_SetBits(GPIOB,GPIO_Pin_4);
		}
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC2|TIM_IT_Update); //清除中断标志位
}
// wifi模块初始化
void initwifi()
{ 
	
	  atk_8266_send_cmd("AT","OK",200);  //测试进入AT模式
		atk_8266_send_cmd("AT+CWMODE=2","OK",200);  ///进入AP模式
		atk_8266_send_cmd("AT+RST","OK",2000);   //重启生效
		atk_8266_send_cmd("AT+CIPAP=\"192.168.12.1\",\"192.168.12.1\",\"255.255.255.0\"","OK",300);  //设置IP
		atk_8266_send_cmd("AT+CWSAP=\"ESP8266\",\"12345678\",1,4","OK",300); //设置AP名称，密码
		atk_8266_send_cmd("AT+CIPMUX=1","OK",200); //开启多链接
		atk_8266_send_cmd("AT+CIPSERVER=1,8086","OK",200);  //开启服务器模式，端口号8086
	
}
//string转int函数
// buff[]为输入字符串，star为字符开始位（第一位要转化的数字ascii，从0开始），len为数字位数
//返回真实int型数字
//注意索引不要超出字符串长度
//@LM 2017.07.24
int stringtoint(u8 buff[],int star,int len)
{
	int number=0;
	int idex=0;
	int time=1; //进制倍率
	for(idex=star+len-1;idex>=star;idex--)
	{
	number+=asctoint(buff[idex])*time;
	time*=10;  //10进制		
	}
	return number;
	
}
//单个ascii转int函数
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
//判断正整数长度的函数
//返回值：0 异常；1，2，3，4，5正常
//int型最大为5位
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
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//接收到数据
	{	 
 
	res =USART_ReceiveData(USART2);		
		if(USART2_RX_STA<USART2_MAX_RECV_LEN)		//还可以接收数据
		{
			TIM_SetCounter(TIM4,0);//计数器清空        				 
			if(USART2_RX_STA==0)TIM5_Set(1);	 	//使能定时器4的中断 
			USART2_RX_BUF[USART2_RX_STA++]=res;		//记录接收到的值	 
		}else 
		{
	 		USART2_RX_STA|=1<<15;					//强制标记接收完成
		} 
		USART_SendData(USART1,res);
		cmd[idex]=res;
		idex++;
		if(idex>10||res=='#')     // 数据结束符
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
