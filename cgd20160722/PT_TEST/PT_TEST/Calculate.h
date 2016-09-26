#ifndef CALCULATH
#define CALCULATH

#include <stdio.h>
#include <Windows.h>
#include "Read_AD.h"
#include "Inverse_kine.h"
#include "gts.h"
#include "conio.h"

class Calculate
{
public:
	Read_AD_Dri *Read_Operat;
	Inverse_kinematics *Inverse;
	int File_Out();
	Calculate();

private:

};

#endif