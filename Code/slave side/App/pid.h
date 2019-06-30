#ifndef __PID_H
#define __PID_H
#endif

//结构体定义
typedef struct  //PID计算参量
	{     
	long int liEkVal[3];          //差值保存，给定和反馈的差值            
	float uKP_Coe;             //比例系数     
	float uKI_Coe;             //积分常数     
	float uKD_Coe;             //微分常数     
	long int iPriVal;             //输出值（PID增量）   
	long int iSetVal;             //设定值     
	long int iCurVal;             //实际值 
	long int maxmin;              //上下限最大步进量 
	int detmax;               //计算临界值，偏差小于此送入计算，否则上限输出
	int okedge;                //允许误差，在此范围内不调整
	} PID_InitTypeDef; 
//函数定义
	int PID_Operation(PID_InitTypeDef* PID);

	