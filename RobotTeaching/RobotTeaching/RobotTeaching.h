
// RobotTeaching.h : PROJECT_NAME Ӧ�ó������ͷ�ļ�
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"		// ������


// CRobotTeachingApp:
// �йش����ʵ�֣������ RobotTeaching.cpp
//

class CRobotTeachingApp : public CWinApp
{
public:
	CRobotTeachingApp();

// ��д
public:
	virtual BOOL InitInstance();

// ʵ��

	DECLARE_MESSAGE_MAP()
};

extern CRobotTeachingApp theApp;