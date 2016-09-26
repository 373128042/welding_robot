#ifndef Inverse_kineH
#define Inverse_kineH

#include <stdio.h>
#include <Windows.h>

class Inverse_kinematics
{
public:
	double Px_in,Py_in,Pz_in;
	double ang_j_S,ang_j_L,ang_j_U;
	void Inverse_calculate(double x_in,double y_in,double z_in);
	void Forward_calculate(double ang_S, double ang_L, double ang_U);
	Inverse_kinematics();//must add
	~Inverse_kinematics();
private:

};

#endif