#include "stdafx.h"
#include "windows.h"
#include "conio.h"
#include "math.h"
#include "gts.h"
#include "stdio.h"
#include "Calculate.h"

#define AXIS		1
#define PI	3.1415926
#define DELTA     4

Calculate * Out_File_Csv;

void commandhandler(char *command, short error)
{
	// 如果指令执行返回值为非0，说明指令执行错误，向屏幕输出错误结果
	if(error)
	{
		printf("%s = %d\n", command, error);
	}
}
void ReadData(float a[50],char file_name[200],int num_samp)
{
	int num = 0;
	FILE* fp1 = fopen(file_name,"r");
	fscanf(fp1,"%f",&num);
	for(int i=0;i<num_samp;i++)
		fscanf(fp1,"%f",&a[i]);
	fclose(fp1);
}

void ReadTime(int a[50],char file_name[200],int num_samp)
{
	int num = 0;
	FILE* fp1 = fopen(file_name,"r");
	fscanf(fp1,"%d",&num);
	for(int i=0;i<num_samp;i++)
		fscanf(fp1,"%d",&a[i]);
	fclose(fp1);
}

int main(int argc, char* argv[])
{
	short sRtn, space;//sRtn--the return state, space--the applied space 
	double pos;//relative position
	long sts1, sts2, sts3, sts4, sts5;//5 axises status
	int	time;//delay time
	int num_samp=0;//the return number of sampling point
	double prfPos1,prfPos2,prfPos3,prfPos4,prfPos5,prfPos6;//6 positions
	double Enc_axis[5];//save encoder number
	TTrapPrm trap;//To get the machine arm setting value

	float s1[50] = { 0 };//axis 1 waiting points
	float m1[50][400] = { 0 };//dive into 400 pieces

	float s2[50] = { 0 };
	float m2[50][400] = { 0 };

	float s3[50] = { 0 };
	float m3[50][400] = { 0 };

	float s4[50] = { 0 };
	float m4[50][400] = { 0 };

	float s5[50] = { 0 };
	float m5[50][400] = { 0 };

	int tf[50] = { 0 };
	int tm[50] = { 0 };

	//打开运动控制器
	sRtn = GT_Open();
	// 指令返回值检测，请查阅例2-1
	commandhandler("GT_Open", sRtn);
	// 配置运动控制器
	// 注意：配置文件取消了各轴的报警和限位
	sRtn = GT_LoadConfig("test.cfg");
	commandhandler("GT_LoadConfig", sRtn);
	// 清除各轴的报警和限位
	sRtn = GT_ClrSts(1, 8);
	commandhandler("GT_ClrSts", sRtn);
	for (int i=0;i<5;i++)
	{
		// 伺服使能
		sRtn = GT_AxisOn(AXIS+i);
		commandhandler("GT_AxisOn", sRtn);
		// 位置清零
		sRtn = GT_ZeroPos(AXIS+i);		
		commandhandler("GT_ZeroPos", sRtn);

		// 将AXIS轴设为点位模式,先让各个关节运动到第一个点
		sRtn = GT_PrfTrap(AXIS+i);
		commandhandler("GT_PrfTrap", sRtn);
		// 读取点位运动参数
		sRtn = GT_GetTrapPrm(AXIS+i, &trap);
		commandhandler("GT_GetTrapPrm", sRtn);
		trap.acc = 0.25;
		trap.dec = 0.125;
		trap.smoothTime = 25;
		// 设置点位运动参数
		sRtn = GT_SetTrapPrm(AXIS+i, &trap);
		commandhandler("GT_SetTrapPrm", sRtn);
		// 设置AXIS轴的目标速度
		sRtn = GT_SetVel(AXIS+i, 100);
		commandhandler("GT_SetVel", sRtn);
	}

	//Generate the output value files 
	Out_File_Csv=new Calculate();
	num_samp=Out_File_Csv->File_Out();//num_samp 要减1 因为函数结束多加了1
	printf("num_samp is %d \n",--num_samp);
	ReadData(s1,"F:\\cgd20160722\\五角星堆焊各关节角\\第1关节.txt",num_samp);
	ReadData(s2,"F:\\cgd20160722\\五角星堆焊各关节角\\第2关节.txt",num_samp);
	ReadData(s3,"F:\\cgd20160722\\五角星堆焊各关节角\\第3关节.txt",num_samp);
	ReadData(s4,"F:\\cgd20160722\\五角星堆焊各关节角\\第4关节.txt",num_samp);
	ReadData(s5,"F:\\cgd20160722\\五角星堆焊各关节角\\第5关节.txt",num_samp);

	ReadTime(tf,"F:\\cgd20160722\\五角星堆焊各关节角\\时间tf(i).txt",num_samp);
	num_samp-=1;//下面用的个数少了1，因为第一个数是0
	for (int i = 0; i<num_samp; i++)
	{
		for (int j = 0; j<400; j++)
		{
			m1[i][j] = s1[i] + (s1[i + 1] - s1[i]) / 400 * j;
			m2[i][j] = s2[i] + (s2[i + 1] - s2[i]) / 400 * j;
			m3[i][j] = s3[i] + (s3[i + 1] - s3[i]) / 400 * j;
			m4[i][j] = s4[i] + (s4[i + 1] - s4[i]) / 400 * j;
			m5[i][j] = s5[i] + (s5[i + 1] - s5[i]) / 400 * j;
			tm[i] = tf[i] / 400 / 2;
		}
	}
	for (int i=0;i<5;i++)
	{
		//必须要清楚FIFO
		sRtn = GT_PtClear(AXIS+i);
		commandhandler("GT_PtClear", sRtn);
		sRtn = GT_PtSpace(AXIS+i, &space);
		commandhandler("GT_PtClear", sRtn);
	}

	// 设置AXIS轴的目标位置
	time=10000;//回去11个点以前的位置 30s 要慢一点

	pos = (s1[0]-s1[num_samp]) ;
	sRtn = GT_PtData(AXIS + 0, pos, time, PT_SEGMENT_EVEN);
	commandhandler("GT_PtData", sRtn);
	pos = (s2[0]-s2[num_samp]) ;
	sRtn = GT_PtData(AXIS + 1, pos, time, PT_SEGMENT_EVEN);
	pos = (s3[0]-s3[num_samp]) ;
	sRtn = GT_PtData(AXIS + 2, pos, time, PT_SEGMENT_EVEN);
	pos = (s4[0]-s4[num_samp]) ;
	sRtn = GT_PtData(AXIS + 3, pos, time, PT_SEGMENT_EVEN);
	pos = (s5[0]-s5[num_samp]) ;
	sRtn = GT_PtData(AXIS + 4, pos, time, PT_SEGMENT_EVEN);


	sRtn = GT_PtStart(1 << (AXIS + 0 - 1));
	commandhandler("GT_PtStart", sRtn);
	sRtn = GT_PtStart(1 << (AXIS + 1 - 1));
	commandhandler("GT_PtStart", sRtn);
	sRtn = GT_PtStart(1 << (AXIS + 2 - 1));
	commandhandler("GT_PtStart", sRtn);
	sRtn = GT_PtStart(1 << (AXIS + 3 - 1));
	commandhandler("GT_PtStart", sRtn);
	sRtn = GT_PtStart(1 << (AXIS + 4 - 1));
	commandhandler("GT_PtStart", sRtn);

	//back to the position s1-5[0]
	printf("back to the position s1-5[0]\n");

	while(1)
	{
		sRtn = GT_GetPrfPos(AXIS+0, &prfPos1);
		prfPos1 = prfPos1 / 25600;
		sRtn = GT_GetPrfPos(AXIS+1, &prfPos2);
		prfPos2 = prfPos2 / 22026;
		sRtn = GT_GetPrfPos(AXIS+2, &prfPos3);
		prfPos3 = prfPos3 / (-22026);
		sRtn = GT_GetPrfPos(AXIS+3, &prfPos4);
		prfPos4 = prfPos4 / 14545;
		sRtn = GT_GetPrfPos(AXIS+4, &prfPos5);
		prfPos5 = prfPos5 / (-13220);
		sRtn = GT_GetSts(AXIS + 0, &sts1);
		sRtn = GT_GetSts(AXIS + 1, &sts2);
		sRtn = GT_GetSts(AXIS + 2, &sts3);
		sRtn = GT_GetSts(AXIS + 3, &sts4);
		sRtn = GT_GetSts(AXIS + 4, &sts5);
		printf("%-10.1lf,%-10.1lf,%-10.1lf,%-10.1lf,%-10.1lf\r", prfPos1,prfPos2,prfPos3,prfPos4,prfPos5);
		
		if (((sts1 & 0x400) == 0) && ((sts2 & 0x400) == 0) && ((sts3 & 0x400) == 0) && ((sts4 & 0x400) == 0) && ((sts5 & 0x400) == 0))
			break;
	}
	
	
	printf("Already back to the position s1-5[0]\n");
	for (int i=0;i<5;i++)
	{
		//读取编码器数值
		sRtn = GT_GetEncPos(AXIS+i,&Enc_axis[i]);//Enc_axis
	}
	printf("%-10.6lf,%-10.6lf,%-10.6lf,%-10.6lf,%-10.6lf\n", Enc_axis[0],Enc_axis[1],Enc_axis[2],Enc_axis[3],Enc_axis[4]);
	printf("teaching is over!!!\n press any key to exit.\n");
	
	// 伺服关闭
	for (int i=0;i<5;i++)
	{
		sRtn = GT_AxisOff(AXIS+i);
		commandhandler("GT_AxisOff", sRtn);
	}
	getch();
	return 0;
}

