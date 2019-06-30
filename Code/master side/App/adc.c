#include "eval.h"
#include "stm32f10x_adc.h"
//#include "adc.h"

//获得ADC值
//ch:通道值 0~3
u16 Get_Adc(u8 ch,int n)   
	{
	//设置转换序列	  		 
	//ADC1->SQR3&=0XFFFFFFE0;//规则序列1 通道ch
	//ADC1->SQR3|=ch;	
	//设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
	ADC_RegularChannelConfig(ADC1, ch, n, ADC_SampleTime_239Cycles5 );		//ADC1,ADC通道3,规则采样顺序值为1,采样时间为239.5周期	  			    
	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束
	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
	}
	void Adc_Init(void)
	{
	// 引脚配置
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure; 
	//PA1/2/3/ PB0 PC2/3/4/5 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_6;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;		//模拟输入引脚
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	ADC_DeInit(ADC1);  //将外设 ADC1 的全部寄存器重设为缺省值
	/* ADC1配置------------------------------------------------------*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //72M/6=12,ADC最大时间不能超过14M
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC	, ENABLE );	  //使能ADC1通道时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟使能
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式
	//ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//模数转换工作在多通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 3;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器
	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1	, ENABLE );	  //使能ADC1通道时钟
  
	while(ADC_GetResetCalibrationStatus(ADC1));	//获取ADC1重置校准寄存器的状态,设置状态则等待
	ADC_StartCalibration(ADC1);		//开始指定ADC1的校准状态
	while(ADC_GetCalibrationStatus(ADC1));		//获取指定ADC1的校准程序,设置状态则等待
	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能
	
	}
	//*************************************
// 函数名称：DMA_Config_ADC1
// 函数功能：DMA 初始化配置
// 入口参数：无
// 出口参数：无
// 返 回 值：无
//***************************************/
//void DMA_Config_ADC1(void)
//{
//    DMA_InitTypeDef DMA_InitStructure;
//     
//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);       //使能DMA时钟
//    DMA_DeInit(DMA1_Channel1);        //开启DMA1的第一通道
//    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address; //DMA对应的外设基地址
//    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&AD_Value[0];//内存存储基地址 自己开僻的数组
//    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//DMA的转换模式为SRC模式，由外设搬移到内存
//    DMA_InitStructure.DMA_BufferSize = Num_Adc_Chanel; //DMA缓存大小，N个
//    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;    //接收一次数据后，设备地址禁止后移
//    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //关闭接收一次数据后，目标内存地址后移
//    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //定义外设数据宽度为16位
//    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;  //DMA搬移数据尺寸，HalfWord就是为16位
//    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;   //转换模式，循环缓存模式。
//    DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMA优先级高
//    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;          //M2M模式禁用
//    DMA_Init(DMA1_Channel1, &DMA_InitStructure);          
//     
//    /* Enable DMA1 channel1 */
//    DMA_Cmd(DMA1_Channel1, ENABLE);
//}