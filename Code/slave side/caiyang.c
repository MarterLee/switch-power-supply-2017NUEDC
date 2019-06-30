#include "caiyang.h"
//采样均值滤波算法
//参数说明：times为均值平滑次数，adc为每次输入的瞬时采样值
//返回值为浮点型，可直接显示或送入显示
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
