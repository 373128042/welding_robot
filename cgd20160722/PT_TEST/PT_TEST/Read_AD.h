#ifndef Read_ADH
#define Read_ADH

#include <stdio.h>
#include <Windows.h>
#include <NIDAQmx.h>

class Read_AD_Dri
{
public:
	bool Init_Sucess;
	bool Read(float64 *AD_DATA);
	Read_AD_Dri();
	~Read_AD_Dri();

private:
	int32 error;
	int32 read;
	float64 data[1000];
	TaskHandle taskHandle;
};

#endif