#include "Read_AD.h"
Read_AD_Dri::Read_AD_Dri()
{
	Init_Sucess=0;//0 is fail while 1 is success
	error=0;//0 stands for all right, other contains mistake
	taskHandle=0;
	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/
	error= DAQmxCreateTask("",&taskHandle);
	if(error!=0)
	{
		Init_Sucess=0;
		return;
	}
	//ai1:7 7个通道
	error= DAQmxCreateAIVoltageChan(taskHandle,"Dev1/ai1:7","",DAQmx_Val_RSE ,-5.0,5.0,DAQmx_Val_Volts,NULL);
	if(error!=0)
	{
		Init_Sucess=0;
		return;
	}
	//第三个参数：采样率，尽可能大，小可能导致混叠 第五个参数：一直采样，最后一个参数定义了一直采样的缓冲区大小
	error= DAQmxCfgSampClkTiming(taskHandle,"",10000.0,DAQmx_Val_Rising,DAQmx_Val_ContSamps,1000);
	if(error!=0)
	{
		Init_Sucess=0;
		return;
	}

	/*********************************************/
	// DAQmx Start Code
	/*********************************************/
	error= DAQmxStartTask(taskHandle);
	if(error!=0)
	{
		Init_Sucess=0;
		return;
	}
	Init_Sucess=1;
}

bool Read_AD_Dri::Read(float64 *AD_DATA)
{
	error =DAQmxReadAnalogF64(taskHandle,1,10.0,DAQmx_Val_GroupByChannel,data,1000,&read,NULL);

	AD_DATA[0]=data[6];//左边 左右
	AD_DATA[1]=data[5];//左边 上下
	AD_DATA[2]=data[0];//左边 旋转
	AD_DATA[3]=data[3];//右边 左右
	AD_DATA[4]=data[4];//右边 上下
	AD_DATA[5]=data[2];//右边 旋转

	return 1;
}

Read_AD_Dri::~Read_AD_Dri()
{
	DAQmxStopTask(taskHandle);
	DAQmxClearTask(taskHandle);
}