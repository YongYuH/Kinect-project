
// Kinect-capture.h : PROJECT_NAME ���ε{�����D�n���Y��
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�� PCH �]�t���ɮ׫e���]�t 'stdafx.h'"
#endif

#include "resource.h"		// �D�n�Ÿ�


// CKinectcaptureApp: 
// �аѾ\��@�����O�� Kinect-capture.cpp
//

class CKinectcaptureApp : public CWinApp
{
public:
	CKinectcaptureApp();

// �мg
public:
	virtual BOOL InitInstance();

// �{���X��@

	DECLARE_MESSAGE_MAP()
};

extern CKinectcaptureApp theApp;