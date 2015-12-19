
// Kinect-captureDlg.h : 標頭檔
//

#pragma once


// CKinectcaptureDlg 對話方塊
class CKinectcaptureDlg : public CDialogEx
{
// 建構
public:
	CKinectcaptureDlg(CWnd* pParent = NULL);	// 標準建構函式

// 對話方塊資料
	enum { IDD = IDD_KINECTCAPTURE_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支援


// 程式碼實作
protected:
	HICON m_hIcon;

	// 產生的訊息對應函式
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
