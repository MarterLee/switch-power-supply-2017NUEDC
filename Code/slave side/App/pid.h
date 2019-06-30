#ifndef __PID_H
#define __PID_H
#endif

//�ṹ�嶨��
typedef struct  //PID�������
	{     
	long int liEkVal[3];          //��ֵ���棬�����ͷ����Ĳ�ֵ            
	float uKP_Coe;             //����ϵ��     
	float uKI_Coe;             //���ֳ���     
	float uKD_Coe;             //΢�ֳ���     
	long int iPriVal;             //���ֵ��PID������   
	long int iSetVal;             //�趨ֵ     
	long int iCurVal;             //ʵ��ֵ 
	long int maxmin;              //��������󲽽��� 
	int detmax;               //�����ٽ�ֵ��ƫ��С�ڴ�������㣬�����������
	int okedge;                //�������ڴ˷�Χ�ڲ�����
	} PID_InitTypeDef; 
//��������
	int PID_Operation(PID_InitTypeDef* PID);

	