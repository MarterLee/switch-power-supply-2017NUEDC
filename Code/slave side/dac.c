#include "stm32f10x_dac.h"
#include "dac.h"


void Dac_Out(int data)
{
   DAC_SetChannel1Data(DAC_Align_12b_R, data);
}
void Dac_Init(void)
{
	DAC_InitTypeDef DAC_InitType;	
//	//DAC配置
	DAC_InitType.DAC_Trigger=DAC_Trigger_None;//不使用触发功能
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//不使用波形发生器
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;// 屏蔽幅值选择器
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable; //DAC输出缓存关
  DAC_Init(DAC_Channel_1,&DAC_InitType);//初始化DAC通道1 

	DAC_Cmd(DAC_Channel_1, ENABLE); //使能指定的DAC通道

	//DAC_SetChannel1Data(DAC_Align_12b_R, 0);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE );   //使能DAC通道时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; //DAc输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//模拟输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA 
	GPIO_SetBits(GPIOA,GPIO_Pin_4);	//PA4输出高
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟使能
}
