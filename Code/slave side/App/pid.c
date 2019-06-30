#include "pid.h"

	PID_InitTypeDef PID;               //����һ���ṹ�壬����ṹ���������㷨��Ҫ�õ��ĸ�������  
	//* ******************************************************** 
	//* �������ƣ�PID_Operation()                                   
	//* �������ܣ�PID����                     
	//* ��ڲ������ޣ��������룬ϵ�����趨ֵ�ȣ�                      
	//* ���ڲ������ޣ����������U(k)��
	//* ����˵����U(k)=KP*[E(k)-E(k-1)]+KI*E(k)+KD*[E(k)-2E(k-1)+E(k-2)] 
	//******************************************************** 
	 int PID_Operation(PID_InitTypeDef* PID) 
	 {
	 long int Temp[3] = {0};   //�м���ʱ����
	 if(PID->iSetVal!=PID->iCurVal)                //�趨ֵ����ʵ��ֵ��
	 {
	 if(PID->iSetVal - PID->iCurVal >PID->detmax)      //ƫ�����PID->detmax��
		  PID->iPriVal = PID->maxmin;                  //ƫ�����PID->detmaxΪ���޷�ֵ���(ȫ�ټ���)
	else if(PID->iSetVal - PID->iCurVal <-PID->detmax)      //ƫ�����-PID->detmax��
		  PID->iPriVal = -PID->maxmin; 
	else if((PID->iSetVal - PID->iCurVal<PID->okedge)&&(PID->iSetVal - PID->iCurVal>-PID->okedge))
		 PID->iPriVal =0;
	  else                                    //���������� 
		{
		Temp[0] = PID->iSetVal - PID->iCurVal;    //ƫ��<=PID->okedge,����E(k) 
		 /* ��ֵ������λ��ע��˳�򣬷���Ḳ�ǵ�ǰ�����ֵ */
		PID->liEkVal[2] = PID->liEkVal[1];
		PID->liEkVal[1] = PID->liEkVal[0];
		PID->liEkVal[0] = Temp[0];	
		//********************************************************************
		Temp[0]=	PID->liEkVal[0] - PID->liEkVal[1];//����E��k)-E(k-1)
		//*********************************************************************
		Temp[2] = PID->liEkVal[1] * 2;                   //2E(k-1) 
		Temp[2] = (PID->liEkVal[0] + PID->liEkVal[2]) - Temp[2];//����E(k-2)+E(k)-2E(k-1)
		//*************************************************************************
		Temp[0] = PID->uKP_Coe * Temp[0];        //KP*[E(k)-E(k-1)]             
		Temp[1] = PID->uKI_Coe * PID->liEkVal[0]; //KI*E(k)             
		Temp[2] = PID->uKD_Coe * Temp[2];        //KD*[E(k-2)+E(k)-2E(k-1)] 		
/* ========= ����U(k) ========= */  
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
	
	 