#include "stm32f10x_dac.h"
#include "dac.h"


void Dac_Out(int data)
{
   DAC_SetChannel1Data(DAC_Align_12b_R, data);
}
void Dac_Init(void)
{
	DAC_InitTypeDef DAC_InitType;	
//	//DAC����
	DAC_InitType.DAC_Trigger=DAC_Trigger_None;//��ʹ�ô�������
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//��ʹ�ò��η�����
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;// ���η�ֵѡ����
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable; //DAC��������
  DAC_Init(DAC_Channel_1,&DAC_InitType);//��ʼ��DACͨ��1 

	DAC_Cmd(DAC_Channel_1, ENABLE); //ʹ��ָ����DACͨ��

	//DAC_SetChannel1Data(DAC_Align_12b_R, 0);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE );   //ʹ��DACͨ��ʱ��
	
	GPIO_InitTypeDef GPIO_InitStructure;	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; //DAc���
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//ģ�����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA 
	GPIO_SetBits(GPIOA,GPIO_Pin_4);	//PA4�����
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��ʹ��
}
