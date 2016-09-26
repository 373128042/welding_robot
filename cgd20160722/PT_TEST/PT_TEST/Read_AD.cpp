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
	//ai1:7 7��ͨ��
	error= DAQmxCreateAIVoltageChan(taskHandle,"Dev1/ai1:7","",DAQmx_Val_RSE ,-5.0,5.0,DAQmx_Val_Volts,NULL);
	if(error!=0)
	{
		Init_Sucess=0;
		return;
	}
	//�����������������ʣ������ܴ�С���ܵ��»�� �����������һֱ���������һ������������һֱ�����Ļ�������С
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

	AD_DATA[0]=data[6];//��� ����
	AD_DATA[1]=data[5];//��� ����
	AD_DATA[2]=data[0];//��� ��ת
	AD_DATA[3]=data[3];//�ұ� ����
	AD_DATA[4]=data[4];//�ұ� ����
	AD_DATA[5]=data[2];//�ұ� ��ת

	return 1;
}

Read_AD_Dri::~Read_AD_Dri()
{
	DAQmxStopTask(taskHandle);
	DAQmxClearTask(taskHandle);
}