#include "caiyang.h"
//������ֵ�˲��㷨
//����˵����timesΪ��ֵƽ��������adcΪÿ�������˲ʱ����ֵ
//����ֵΪ�����ͣ���ֱ����ʾ��������ʾ
float caiyang(int adc)
{
	int i;
	long int sum=0;
	float out;
for(i=99;i>=1;i--)
		{
			temp[i]=temp[i-1];
			sum=sum+temp[i];
		}
		temp[0]=adc;
		sum=sum+temp[0];
		out=sum/(times+1);
    return out;
}
