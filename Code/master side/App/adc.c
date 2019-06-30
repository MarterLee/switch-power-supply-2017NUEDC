#include "eval.h"
#include "stm32f10x_adc.h"
//#include "adc.h"

//���ADCֵ
//ch:ͨ��ֵ 0~3
u16 Get_Adc(u8 ch,int n)   
	{
	//����ת������	  		 
	//ADC1->SQR3&=0XFFFFFFE0;//��������1 ͨ��ch
	//ADC1->SQR3|=ch;	
	//����ָ��ADC�Ĺ�����ͨ�����������ǵ�ת��˳��Ͳ���ʱ��
	ADC_RegularChannelConfig(ADC1, ch, n, ADC_SampleTime_239Cycles5 );		//ADC1,ADCͨ��3,�������˳��ֵΪ1,����ʱ��Ϊ239.5����	  			    
	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������
	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
	}
	void Adc_Init(void)
	{
	// ��������
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure; 
	//PA1/2/3/ PB0 PC2/3/4/5 ��Ϊģ��ͨ����������                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_6;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;		//ģ����������
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	ADC_DeInit(ADC1);  //������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ
	/* ADC1����------------------------------------------------------*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //72M/6=12,ADC���ʱ�䲻�ܳ���14M
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC	, ENABLE );	  //ʹ��ADC1ͨ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��ʹ��
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//ģ��ת�������ڵ�ͨ��ģʽ
	//ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//ģ��ת�������ڶ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 3;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���
	ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1	, ENABLE );	  //ʹ��ADC1ͨ��ʱ��
  
	while(ADC_GetResetCalibrationStatus(ADC1));	//��ȡADC1����У׼�Ĵ�����״̬,����״̬��ȴ�
	ADC_StartCalibration(ADC1);		//��ʼָ��ADC1��У׼״̬
	while(ADC_GetCalibrationStatus(ADC1));		//��ȡָ��ADC1��У׼����,����״̬��ȴ�
	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������
	
	}
	//*************************************
// �������ƣ�DMA_Config_ADC1
// �������ܣ�DMA ��ʼ������
// ��ڲ�������
// ���ڲ�������
// �� �� ֵ����
//***************************************/
//void DMA_Config_ADC1(void)
//{
//    DMA_InitTypeDef DMA_InitStructure;
//     
//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);       //ʹ��DMAʱ��
//    DMA_DeInit(DMA1_Channel1);        //����DMA1�ĵ�һͨ��
//    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address; //DMA��Ӧ���������ַ
//    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&AD_Value[0];//�ڴ�洢����ַ �Լ���Ƨ������
//    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//DMA��ת��ģʽΪSRCģʽ����������Ƶ��ڴ�
//    DMA_InitStructure.DMA_BufferSize = Num_Adc_Chanel; //DMA�����С��N��
//    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;    //����һ�����ݺ��豸��ַ��ֹ����
//    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //�رս���һ�����ݺ�Ŀ���ڴ��ַ����
//    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //�����������ݿ��Ϊ16λ
//    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;  //DMA�������ݳߴ磬HalfWord����Ϊ16λ
//    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;   //ת��ģʽ��ѭ������ģʽ��
//    DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMA���ȼ���
//    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;          //M2Mģʽ����
//    DMA_Init(DMA1_Channel1, &DMA_InitStructure);          
//     
//    /* Enable DMA1 channel1 */
//    DMA_Cmd(DMA1_Channel1, ENABLE);
//}