#ifndef __CPWM_H
#define __CPWM_H
#endif
//º¯ÊýÉêÃ÷
#include "eval.h"
void CPWM_Init(u16 arr,u16 psc);
void Timer3_Init(u16 arr,u16 psc);
void Timer2_Init(u16 arr,u16 psc);
void TIM3_IRQHandler(void);   //TIM3ÖÐ¶Ï
void NVIC_Configuration(void);
void TIM2_IRQHandler(void);
void Timer5_Init(u16 arr,u16 psc);
void TIM5_IRQHandler(void);