
// Kinect-captureDlg.h : ���Y��
//

#pragma once


// CKinectcaptureDlg ��ܤ��
class CKinectcaptureDlg : public CDialogEx
{
// �غc
public:
	CKinectcaptureDlg(CWnd* pParent = NULL);	// �зǫغc�禡

// ��ܤ�����
	enum { IDD = IDD_KINECTCAPTURE_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV �䴩


// �{���X��@
protected:
	HICON m_hIcon;

	// ���ͪ��T�������禡
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButton_Background();
	afx_msg void OnBnClickedButton_Capture();
	afx_msg void OnBnClickedButton_Output();
	afx_msg void OnBnClickedButton_Release();
	afx_msg void OnBnClickedButton_Coordinate();
};
