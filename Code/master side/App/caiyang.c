#include "caiyang.h"
//������ֵ�˲��㷨
//����˵����timesΪ��ֵƽ��������adcΪÿ�������˲ʱ����ֵ,*tempΪ�洢���������ַ
//����ֵΪ�����ͣ���ֱ����ʾ��������ʾ
//ʹ��ǰҪ�������ݴ洢���飬��ַ���뺯��
float caiyang(int times,int adc,int *temp)
{
	int i;
	long int sum=0;
	float out;
for(i=times-1;i>=1;i--)
		{
			temp[i]=temp[i-1];
			sum=sum+temp[i];
		}
		temp[0]=adc;
		sum=sum+temp[0];
		out=sum/times;
    return out;
}