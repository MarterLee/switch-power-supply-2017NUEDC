#include "stm32f10x.h"
#include "eval.h"
#include "delay.h"
#include "key.h"
void KEY_Init(void)
{
STM_EVAL_PBInit(Button_KEY0, Mode_GPIO);			//���ð���GPIO����
STM_EVAL_PBInit(Button_KEY1, Mode_GPIO);			//���ð���GPIO����
STM_EVAL_PBInit(Button_WAKEUP, Mode_GPIO);			//���ð���GPIO����
}
//����������
//���ذ���ֵ
//0��û���κΰ�������
//1��KEY0����
//2��KEY1����
//3��KEY2���� WK_UP
//ע��˺�������Ӧ���ȼ�,KEY0>KEY1>KEY2!!
	u8 KEY_Scan(void)
	{	 
	static u8 key_up=1;//�������ɿ���־	
	if(key_up&&(((STM_EVAL_PBGetState(Button_KEY0) == 0x00)||(STM_EVAL_PBGetState(Button_KEY1) == 0x00)||(STM_EVAL_PBGetState(Button_WAKEUP) == 0x01))))
		{
		delay_ms(10);//ȥ���� 
		key_up=0;
		if (STM_EVAL_PBGetState(Button_KEY0) == 0x00)		//��������:�͵�ƽ��Ч
			{
			return 1;
			}
		if (STM_EVAL_PBGetState(Button_KEY1) == 0x00)		//��������:�͵�ƽ��Ч
			{
			return 2;
			}
		if (STM_EVAL_PBGetState(Button_WAKEUP) == 0x01)		//��������:�ߵ�ƽ��Ч
			{
			return 3;
			}
		if (STM_EVAL_PBGetState(Button_KEY0) == 0x00&&STM_EVAL_PBGetState(Button_KEY1) == 0x00)	
		{
		return  4;
		}
		}
		else if((STM_EVAL_PBGetState(Button_KEY0) == 0x01)&&(STM_EVAL_PBGetState(Button_KEY1) == 0x01)&&(STM_EVAL_PBGetState(Button_WAKEUP) == 0x00)) key_up=1; 	    
	return 0;// �ް�������
	}