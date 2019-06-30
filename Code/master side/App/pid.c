#include "pid.h"

	PID_InitTypeDef PID;               //定义一个结构体，这个结构体用来存算法中要用到的各种数据  
	//* ******************************************************** 
	//* 函数名称：PID_Operation()                                   
	//* 函数功能：PID运算                     
	//* 入口参数：无（隐形输入，系数、设定值等）                      
	//* 出口参数：无（隐形输出，U(k)）
	//* 函数说明：U(k)=KP*[E(k)-E(k-1)]+KI*E(k)+KD*[E(k)-2E(k-1)+E(k-2)] 
	//******************************************************** 
	 int PID_Operation(PID_InitTypeDef* PID) 
	 {
	 long int Temp[3] = {0};   //中间临时变量
	 if(PID->iSetVal!=PID->iCurVal)                //设定值等于实际值否？
	 {
	 if(PID->iSetVal - PID->iCurVal >PID->detmax)      //偏差大于PID->detmax否？
		  PID->iPriVal = PID->maxmin;                  //偏差大于PID->detmax为上限幅值输出(全速加热)
	else if(PID->iSetVal - PID->iCurVal <-PID->detmax)      //偏差大于-PID->detmax否？
		  PID->iPriVal = -PID->maxmin; 
	else if((PID->iSetVal - PID->iCurVal<PID->okedge)&&(PID->iSetVal - PID->iCurVal>-PID->okedge))
		 PID->iPriVal =0;
	  else                                    //否则慢慢来 
		{
		Temp[0] = PID->iSetVal - PID->iCurVal;    //偏差<=PID->okedge,计算E(k) 
		 /* 数值进行移位，注意顺序，否则会覆盖掉前面的数值 */
		PID->liEkVal[2] = PID->liEkVal[1];
		PID->liEkVal[1] = PID->liEkVal[0];
		PID->liEkVal[0] = Temp[0];	
		//********************************************************************
		Temp[0]=	PID->liEkVal[0] - PID->liEkVal[1];//计算E（k)-E(k-1)
		//*********************************************************************
		Temp[2] = PID->liEkVal[1] * 2;                   //2E(k-1) 
		Temp[2] = (PID->liEkVal[0] + PID->liEkVal[2]) - Temp[2];//计算E(k-2)+E(k)-2E(k-1)
		//*************************************************************************
		Temp[0] = PID->uKP_Coe * Temp[0];        //KP*[E(k)-E(k-1)]             
		Temp[1] = PID->uKI_Coe * PID->liEkVal[0]; //KI*E(k)             
		Temp[2] = PID->uKD_Coe * Temp[2];        //KD*[E(k-2)+E(k)-2E(k-1)] 		
/* ========= 计算U(k) ========= */  
   PID->iPriVal=(int)(Temp[0]+Temp[1]+Temp[2]);
	 if (PID->iPriVal>PID->maxmin)
		 PID->iPriVal=PID->maxmin;
	 if (PID->iPriVal<-PID->maxmin)
		 PID->iPriVal=-PID->maxmin;
	 else 
		 ;
 }
}
	 return PID->iPriVal;
	 
	 }
	
	 