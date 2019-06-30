#include "caiyang.h"
//采样均值滤波算法
//参数说明：times为均值平滑次数，adc为每次输入的瞬时采样值,*temp为存储数据数组地址
//返回值为浮点型，可直接显示或送入显示
//使用前要定义数据存储数组，地址送入函数
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