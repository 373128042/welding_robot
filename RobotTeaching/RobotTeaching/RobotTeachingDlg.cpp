
// RobotTeachingDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "RobotTeaching.h"
#include "RobotTeachingDlg.h"
#include "afxdialogex.h"
#include "gts.h"
#include "Inverse_kine.h"
#include <math.h>
#include "NIDAQmx.h"

#define AXIS 1
#define p1 25600
#define p2 22026
#define p3 -22026
#define p4 14545
#define p5 -13220

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

int         error=0;
TaskHandle	writeTaskHandle=0;
TaskHandle	readTaskHandle=0;
uInt32      data=0x4;
int32		written;
int32		read;

void CheckPosition();
short sRtn, space;//sRtn--the return state, space--the applied space 
int times;//PT's time 
int k;//the welding speed controller
int num;//points number
bool axis_on=false;//a flag
long sts1, sts2, sts3, sts4, sts5;//5 axises status
double pos;//relative position
double Enc_axis[5];
double prfPos1,prfPos2,prfPos3,prfPos4,prfPos5,prfPos6;//6 positions

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

void ReadData(float a[50],char file_name[200])
{
	num = 0;
	FILE* fp1 = fopen(file_name,"r");
	fscanf(fp1,"%d",&num);
	for(int i=0;i<num+1;i++)
		fscanf(fp1,"%f",&a[i]);
	fclose(fp1);
}

void ReadTime(int a[50],char file_name[200])
{
	int num_test = 0;
	FILE* fp1 = fopen(file_name,"r");
	fscanf(fp1,"%d",&num_test);
	for(int i=0;i<num+1;i++)
		fscanf(fp1,"%d",&a[i]);
	fclose(fp1);
}

void commandhandler(char *command, short error)
{
	// 如果指令执行返回值为非0，说明指令执行错误，向屏幕输出错误结果
	if(error)
	{
		printf("%s = %d\n", command, error);
	}
}

// CRobotTeachingDlg 对话框

CRobotTeachingDlg::CRobotTeachingDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CRobotTeachingDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CRobotTeachingDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CRobotTeachingDlg, CDialogEx)
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON1, &CRobotTeachingDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON6, &CRobotTeachingDlg::OnBnClickedButton6)
	ON_BN_CLICKED(IDC_BUTTON2, &CRobotTeachingDlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BUTTON3, &CRobotTeachingDlg::OnBnClickedButton3)
	ON_BN_CLICKED(IDC_BUTTON4, &CRobotTeachingDlg::OnBnClickedButton4)
	ON_BN_CLICKED(IDC_BUTTON5, &CRobotTeachingDlg::OnBnClickedButton5)
	ON_BN_CLICKED(IDC_BUTTON7, &CRobotTeachingDlg::OnBnClickedButton7)
	ON_BN_CLICKED(IDC_BUTTON8, &CRobotTeachingDlg::OnBnClickedButton8)
	ON_BN_CLICKED(IDC_BUTTON9, &CRobotTeachingDlg::OnBnClickedButton9)
	ON_BN_CLICKED(IDC_BUTTON10, &CRobotTeachingDlg::OnBnClickedButton10)
	ON_BN_CLICKED(IDC_BUTTON11, &CRobotTeachingDlg::OnBnClickedButton11)
END_MESSAGE_MAP()


// CRobotTeachingDlg 消息处理程序

BOOL CRobotTeachingDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 设置此对话框的图标。当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码
	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/
	error=DAQmxCreateTask("",&writeTaskHandle);
	error=DAQmxCreateDOChan(writeTaskHandle,"Dev1/port0","",DAQmx_Val_ChanForAllLines);

	/*********************************************/
	// DAQmx Start Code
	/*********************************************/
	error=DAQmxStartTask(writeTaskHandle);

	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/
	error=DAQmxCreateTask("",&readTaskHandle);
	error=DAQmxCreateDIChan(readTaskHandle,"Dev1/port1","",DAQmx_Val_ChanForAllLines );

	/*********************************************/
	// DAQmx Start Code
	/*********************************************/
	error=DAQmxStartTask(readTaskHandle);

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CRobotTeachingDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CRobotTeachingDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


//启动使能
void CRobotTeachingDlg::OnBnClickedButton1()
{
	// TODO: 在此添加控件通知处理程序代码
	INT_PTR nRes;             // 用于保存DoModal函数的返回值   
	//nRes = MessageBox(_T("您确定要进行加法计算吗？"), _T("加法计算器"), MB_OKCANCEL | MB_ICONQUESTION);   
	// 判断消息对话框返回值。如果为IDCANCEL就return，否则继续向下执行 
	CString message=_T("使能成功");
	CString title=_T("使能结果");
	TTrapPrm trap;//To get the machine arm setting value
	short sRtn, space;//sRtn--the return state, space--the applied space 
	//打开运动控制器
	sRtn = GT_Open();
	// 指令返回值检测，请查阅例2-1
	commandhandler("GT_Open", sRtn);
	// 清除各轴的报警和限位
	sRtn = GT_ClrSts(1, 8);
	commandhandler("GT_ClrSts", sRtn);
	for (int i=0;i<5;i++)
	{
		// 伺服使能
		sRtn = GT_AxisOn(AXIS+i);
		commandhandler("GT_AxisOn", sRtn);
		// 位置清零
		//sRtn = GT_ZeroPos(AXIS+i);		
		//commandhandler("GT_ZeroPos", sRtn);
	}
	if(sRtn==0){
		axis_on=true;
		MessageBox(_T("使能开启成功"), _T("标题"),MB_OK);
	}else{
		axis_on=false;
		MessageBox(_T("使能开启失败"), _T("标题"),MB_OK);
	}

}

//关闭私服
void CRobotTeachingDlg::OnBnClickedButton6()
{
	// TODO: 在此添加控件通知处理程序代码
	short sRtn, space;//sRtn--the return state, space--the applied space 

	// 伺服关闭
	for (int i=0;i<6;i++)
	{
		sRtn = GT_AxisOff(AXIS+i);
		commandhandler("GT_AxisOff", sRtn);
	}
	if(sRtn==0){
		axis_on=false;
		MessageBox(_T("使能关闭成功"), _T("标题"),MB_OK);
	}else{
		MessageBox(_T("使能关闭失败"), _T("标题"),MB_OK);
	}
}

//启动示教程序
void CRobotTeachingDlg::OnBnClickedButton2()
{
	// TODO: 在此添加控件通知处理程序代码
	ShellExecuteA(NULL, "open", "F:\\cgd20160722\\PT_TEST\\Debug\\PT_TEST.exe",NULL, NULL, SW_SHOWNORMAL);
}

//加载各轴数据
void CRobotTeachingDlg::OnBnClickedButton3()
{
	// TODO: 在此添加控件通知处理程序代码
	ReadData(s1,"F:\\cgd20160722\\五角星堆焊各关节角\\第1关节.txt");
	ReadData(s2,"F:\\cgd20160722\\五角星堆焊各关节角\\第2关节.txt");
	ReadData(s3,"F:\\cgd20160722\\五角星堆焊各关节角\\第3关节.txt");
	ReadData(s4,"F:\\cgd20160722\\五角星堆焊各关节角\\第4关节.txt");
	ReadData(s5,"F:\\cgd20160722\\五角星堆焊各关节角\\第5关节.txt");
	ReadTime(tf,"F:\\cgd20160722\\五角星堆焊各关节角\\时间tf(i).txt");
	if(num==0){
		MessageBox(_T("加载轴数据失败，点个数为0！"), _T("标题"),MB_OK);
	}else{
		MessageBox(_T("加载成功"), _T("标题"),MB_OK);
	}
}

//repeat teaching line
void CRobotTeachingDlg::OnBnClickedButton4()
{
	if(!axis_on){
		MessageBox(_T("请先使能！"), _T("标题"),MB_OK);
		return;
	}
	if(num==0){
		MessageBox(_T("点个数为0！"), _T("标题"),MB_OK);
		return;
	}
	
	CRobotTeachingDlg::OnBnClickedButton8();
	
	while(1){
		// DAQmx Read Code
		DAQmxReadDigitalU32(readTaskHandle,1,10.0,DAQmx_Val_GroupByChannel,&data,1,&read,NULL);
		if(data==2){
			break;
		}
		Sleep(100);
	}

	CRobotTeachingDlg::OnBnClickedButton11();
}

//back to 1st point
void CRobotTeachingDlg::OnBnClickedButton5()
{
	if(!axis_on){
		MessageBox(_T("请先使能！"), _T("标题"),MB_OK);
		return;
	}
	// TODO: 在此添加控件通知处理程序代码
	if(num==0){
		MessageBox(_T("加载轴数据失败，点个数为0！"), _T("标题"),MB_OK);
		return;
	}

	for (int i=0;i<5;i++)
	{
		// 将AXIS1轴设为PT模式
		sRtn = GT_PrfPt(AXIS + i, PT_MODE_DYNAMIC);

		// 清空AXIS1轴的FIFO
		sRtn = GT_PtClear(AXIS+i);
		sRtn = GT_SetPtMemory(AXIS + i,0);
	}

	for (int i=0;i<5;i++)
	{
		//读取编码器数值
		sRtn = GT_GetEncPos(AXIS+i,&Enc_axis[i]);//Enc_axis
	}

	// 设置AXIS轴的目标位置
	times=10000;//回去11个点以前的位置 30s 要慢一点
	pos = (s1[0]-Enc_axis[0]) ;
	sRtn = GT_PtData(AXIS + 0, pos, times, PT_SEGMENT_STOP);
	commandhandler("GT_PtData", sRtn);
	pos = (s2[0]-Enc_axis[1]) ;
	sRtn = GT_PtData(AXIS + 1, pos, times, PT_SEGMENT_STOP);
	pos = (s3[0]-Enc_axis[2]) ;
	sRtn = GT_PtData(AXIS + 2, pos, times, PT_SEGMENT_STOP);
	pos = (s4[0]-Enc_axis[3]) ;
	sRtn = GT_PtData(AXIS + 3, pos, times, PT_SEGMENT_STOP);
	pos = (s5[0]-Enc_axis[4]) ;
	sRtn = GT_PtData(AXIS + 4, pos, times, PT_SEGMENT_STOP);

	//start
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

	CheckPosition();
	if(sRtn==0){
		MessageBox(_T("回第一个点成功！"), _T("标题"),MB_OK);
	}else{
		MessageBox(_T("回第一个点失败"), _T("标题"),MB_OK);
	}
}

//back to 0 point
void CRobotTeachingDlg::OnBnClickedButton7()
{
	if(!axis_on){
		MessageBox(_T("请先使能！"), _T("标题"),MB_OK);
		return;
	}
	TTrapPrm trap;//To get the machine arm setting value
	// TODO: 在此添加控件通知处理程序代码
	for (int i=0;i<6;i++)
	{
		// 将AXIS轴设为点位模式
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
		// 设置AXIS轴的目标位置
		sRtn = GT_SetPos(AXIS+i, 0);
		commandhandler("GT_SetPos", sRtn);
		// 设置AXIS轴的目标速度
		sRtn = GT_SetVel(AXIS+i, 100);
		commandhandler("GT_SetVel", sRtn);
		// 启动AXIS轴的运动
		sRtn = GT_Update(1<<(AXIS+i-1));
		commandhandler("GT_Update", sRtn);
	}
	CheckPosition();
	if(sRtn==0){
		MessageBox(_T("回第零点成功！"), _T("标题"),MB_OK);
	}else{
		MessageBox(_T("回第零点失败"), _T("标题"),MB_OK);
	}
}

//开信号
void CRobotTeachingDlg::OnBnClickedButton8()
{
	// TODO: 在此添加控件通知处理程序代码
	data=0x4;
	/*********************************************/
	// DAQmx Write Code
	/*********************************************/
	error=DAQmxWriteDigitalU32(writeTaskHandle,1,1,10.0,DAQmx_Val_GroupByChannel,&data,&written,NULL);
}

//关信号
void CRobotTeachingDlg::OnBnClickedButton9()
{
	// TODO: 在此添加控件通知处理程序代码
	data=0x0;
	/*********************************************/
	// DAQmx Write Code
	/*********************************************/
	error=DAQmxWriteDigitalU32(writeTaskHandle,1,1,10.0,DAQmx_Val_GroupByChannel,&data,&written,NULL);
}

//记录运动速度
void CheckPosition(){
	FILE *fp[5];
	fp[0]=fopen("F:\\cgd20160722\\五角星堆焊各关节角\\速度v(1).csv","w");
	fp[1]=fopen("F:\\cgd20160722\\五角星堆焊各关节角\\速度v(2).csv","w");
	fp[2]=fopen("F:\\cgd20160722\\五角星堆焊各关节角\\速度v(3).csv","w");
	fp[3]=fopen("F:\\cgd20160722\\五角星堆焊各关节角\\速度v(4).csv","w");
	fp[4]=fopen("F:\\cgd20160722\\五角星堆焊各关节角\\速度v(5).csv","w");
	int i=0;
	while(1)
	{
		sRtn = GT_GetPrfVel(AXIS+0, &prfPos1);
		prfPos1 = prfPos1 / 25600;
		sRtn = GT_GetPrfVel(AXIS+1, &prfPos2);
		prfPos2 = prfPos2 / 22026;
		sRtn = GT_GetPrfVel(AXIS+2, &prfPos3);
		prfPos3 = prfPos3 / (-22026);
		sRtn = GT_GetPrfVel(AXIS+3, &prfPos4);
		prfPos4 = prfPos4 / 14545;
		sRtn = GT_GetPrfVel(AXIS+4, &prfPos5);
		prfPos5 = prfPos5 / (-13220);
		sRtn = GT_GetSts(AXIS + 0, &sts1);
		sRtn = GT_GetSts(AXIS + 1, &sts2);
		sRtn = GT_GetSts(AXIS + 2, &sts3);
		sRtn = GT_GetSts(AXIS + 3, &sts4);
		sRtn = GT_GetSts(AXIS + 4, &sts5);
		printf("%-10.1lf,%-10.1lf,%-10.1lf,%-10.1lf,%-10.1lf\r", prfPos1,prfPos2,prfPos3,prfPos4,prfPos5);
		i+=1;
		fprintf(fp[0],"%f,",prfPos1);
		fprintf(fp[1],"%f,",prfPos2);
		fprintf(fp[2],"%f,",prfPos3);
		fprintf(fp[3],"%f,",prfPos4);
		fprintf(fp[4],"%f,",prfPos5);
		if (((sts1 & 0x400) == 0) && ((sts2 & 0x400) == 0) && ((sts3 & 0x400) == 0) && ((sts4 & 0x400) == 0) && ((sts5 & 0x400) == 0))
			break;
	}
	for(int j=0;j<5;j++){
		fclose(fp[j]);
	}

	printf("%d",i);
}

//设置速度倍率
void CRobotTeachingDlg::OnBnClickedButton10()
{
	// TODO: 在此添加控件通知处理程序代码
	//CDialogEx::OnOK();
	//获得EDIT
	CEdit* pBoxOne;
	pBoxOne = (CEdit*) GetDlgItem(IDC_EDIT1);
	//赋值
	//pBoxOne-> SetWindowText( _T("FOO ") );
	//取值
	CString str;
	pBoxOne-> GetWindowText(str);
	//int nGetInt = _tstoi( LPCTSTR(a) );
	int nGetInt = _ttoi(str);
	if(nGetInt!=0){
		k=nGetInt;
		MessageBox(_T("设置成功！"),_T("标题"),MB_OK);
	}else{
		MessageBox(_T("请输入大于0的整数！"),_T("标题！"),MB_OK);
	}
	//MessageBox(nGetInt,_T("程序运行结果"),MB_OK);
	str.ReleaseBuffer();
}

//空走一次
void CRobotTeachingDlg::OnBnClickedButton11()
{
	// TODO: 在此添加控件通知处理程序代码
	if(!axis_on){
		MessageBox(_T("请先使能！"), _T("标题"),MB_OK);
		return;
	}
	if(num==0){
		MessageBox(_T("点个数为0！"), _T("标题"),MB_OK);
		return;
	}

	// TODO: 在此添加控件通知处理程序代码
	for (int i=0;i<5;i++)
	{
		// 将AXIS轴设为PT模式
		sRtn = GT_PrfPt(AXIS + i, PT_MODE_DYNAMIC);
		commandhandler("GT_PrfPt", sRtn);

		// 清空AXIS轴的FIFO
		sRtn = GT_PtClear(AXIS+i);
		commandhandler("GT_PtClear", sRtn);

		sRtn = GT_SetPtMemory(AXIS + i,0);
	}
	pos = 0;
	times = 0;
	int stop = 0;
	int start = 0;
	int dot = 0;
	int part = 0;

	Inverse_kine *Inverse = new Inverse_kine();
	for (int i = 0; i<num-1; i++)
	{
		//calculate the length between two points
		float XYZ[2][3]={0};
		Inverse->Forward_calculate(s1[i]/p1,s2[i]/p2,s3[i]/p3);
		XYZ[0][0]=Inverse->Px_in;
		XYZ[0][1]=Inverse->Py_in;
		XYZ[0][2]=Inverse->Pz_in;
		Inverse->Forward_calculate(s1[i+1]/p1,s2[i+1]/p2,s3[i+1]/p3);
		XYZ[1][0]=Inverse->Px_in;
		XYZ[1][1]=Inverse->Py_in;
		XYZ[1][2]=Inverse->Pz_in;
		//len stand for the how many times the time should be
		int len=int(sqrt(pow(XYZ[1][0]-XYZ[0][0],2)+pow(XYZ[1][1]-XYZ[0][1],2)+pow(XYZ[1][2]-XYZ[0][2],2))+0.5);
		if(len==0){len=1;}
		for (int j = 0; j<400; j++)
		{
			m1[i][j] = s1[i] + (s1[i + 1] - s1[i]) / 400 * j;
			m2[i][j] = s2[i] + (s2[i + 1] - s2[i]) / 400 * j;
			m3[i][j] = s3[i] + (s3[i + 1] - s3[i]) / 400 * j;
			m4[i][j] = s4[i] + (s4[i + 1] - s4[i]) / 400 * j;
			m5[i][j] = s5[i] + (s5[i + 1] - s5[i]) / 400 * j;
			if(k!=0){
				tm[i] = (tf[i]  * len / 400 / k );
			}else{
				tm[i] = (tf[i]  * len / 400 );
			}
			
		}
	}

	//repeat teaching line procedure
	while (1)
	{
		sRtn = GT_PtSpace(AXIS, &space);
		if (space > 0)
		{
			if (dot < num-1)
			{
				if (part < 400)
				{
					times += tm[dot];
					pos = (m1[dot][part] - s1[0]) ;
					sRtn = GT_PtData(AXIS + 0, pos, times,PT_SEGMENT_EVEN);//PT_SEGMENT_EVEN 匀速转
					commandhandler("GT_PtData", sRtn);
					pos = (m2[dot][part] - s2[0]) ;
					sRtn = GT_PtData(AXIS + 1, pos, times,PT_SEGMENT_EVEN);
					pos = (m3[dot][part] - s3[0]) ;
					sRtn = GT_PtData(AXIS + 2, pos, times,PT_SEGMENT_EVEN);
					pos = (m4[dot][part] - s4[0]) ;
					sRtn = GT_PtData(AXIS + 3, pos, times,PT_SEGMENT_EVEN);
					pos = (m5[dot][part] - s5[0]) ;
					sRtn = GT_PtData(AXIS + 4, pos, times,PT_SEGMENT_EVEN);
					part = part + 1;
				}
				else
				{
					part = 0;
					dot = dot + 1;
				}
			}
			else
			{
				dot = 0;
				break;
			}
		}
		else if (start == 0)
		{
			sRtn = GT_PtStart(1 << (AXIS + 0 - 1));
			commandhandler("GT_PtStart", sRtn);
			sRtn = GT_PtStart(1 << (AXIS + 1 - 1));
			sRtn = GT_PtStart(1 << (AXIS + 2 - 1));
			sRtn = GT_PtStart(1 << (AXIS + 3 - 1));
			sRtn = GT_PtStart(1 << (AXIS + 4 - 1));
			start = 1;
		}	

	}
	CheckPosition();
	//示教结束 收弧
	CRobotTeachingDlg::OnBnClickedButton9();
	MessageBox(_T("演示结束！"), _T("标题"),MB_OK);
}
