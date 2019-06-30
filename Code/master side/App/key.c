#include "stm32f10x.h"
#include "eval.h"
#include "delay.h"
#include "key.h"
void KEY_Init(void)
{
STM_EVAL_PBInit(Button_KEY0, Mode_GPIO);			//设置按键GPIO属性
STM_EVAL_PBInit(Button_KEY1, Mode_GPIO);			//设置按键GPIO属性
STM_EVAL_PBInit(Button_WAKEUP, Mode_GPIO);			//设置按键GPIO属性
}
//按键处理函数
//返回按键值
//0，没有任何按键按下
//1，KEY0按下
//2，KEY1按下
//3，KEY2按下 WK_UP
//注意此函数有响应优先级,KEY0>KEY1>KEY2!!
	u8 KEY_Scan(void)
	{	 
	static u8 key_up=1;//按键按松开标志	
	if(key_up&&(((STM_EVAL_PBGetState(Button_KEY0) == 0x00)||(STM_EVAL_PBGetState(Button_KEY1) == 0x00)||(STM_EVAL_PBGetState(Button_WAKEUP) == 0x01))))
		{
		delay_ms(10);//去抖动 
		key_up=0;
		if (STM_EVAL_PBGetState(Button_KEY0) == 0x00)		//按键按下:低电平有效
			{
			return 1;
			}
		if (STM_EVAL_PBGetState(Button_KEY1) == 0x00)		//按键按下:低电平有效
			{
			return 2;
			}
		if (STM_EVAL_PBGetState(Button_WAKEUP) == 0x01)		//按键按下:高电平有效
			{
			return 3;
			}
		if (STM_EVAL_PBGetState(Button_KEY0) == 0x00&&STM_EVAL_PBGetState(Button_KEY1) == 0x00)	
		{
		return  4;
		}
		}
		else if((STM_EVAL_PBGetState(Button_KEY0) == 0x01)&&(STM_EVAL_PBGetState(Button_KEY1) == 0x01)&&(STM_EVAL_PBGetState(Button_WAKEUP) == 0x00)) key_up=1; 	    
	return 0;// 无按键按下
	}