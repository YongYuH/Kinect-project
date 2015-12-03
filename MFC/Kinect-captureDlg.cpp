
// Kinect-captureDlg.cpp : 實作檔

#include "stdafx.h"
#include "Kinect-capture.h"
#include "Kinect-captureDlg.h"
#include "afxdialogex.h"

#include <iostream>
#include <fstream>
#include <math.h>
#include <sstream>

// Play the sound effect
#include <Mmsystem.h>
#pragma comment(lib,"Winmm.lib")

// Kinect for Windows SDK Header
#include "Kinect.h"
// Link Kinect Library
#pragma comment( lib, "kinect20.lib" )

// OpenCV Header
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
// Link OpenCV Library
#define OPENCV300

#ifdef OPENCV300
#ifdef _DEBUG
#pragma comment( lib, "opencv_ts300d.lib" )
#pragma comment( lib, "opencv_world300d.lib" )
#else
#pragma comment( lib, "opencv_ts300.lib" )
#pragma comment( lib, "opencv_world300.lib" )
#endif
#endif

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

using namespace std;

// Threshold for mask
#define ANKLE_HEIGHT (750)
#define X_BACK (3100)
#define X_FRONT (1900)
#define Y_LEFT (690)
#define Y_RIGHT (-690)
#define Z_HEIGHT (-950)

// global variables
IKinectSensor*		g_pSensor = nullptr;
IColorFrameReader*	g_pColorFrameReader = nullptr;
IDepthFrameReader*	g_pDepthFrameReader = nullptr;
IBodyFrameReader*	g_pBodyFrameReader = nullptr;
ICoordinateMapper*	g_pCoordinateMapper = nullptr;

IBody** g_aBodyData = nullptr;
INT32 g_iBodyCount = 0;

int		g_iColorWidth = 0, g_iColorHeight = 0;
int		g_iDepthWidth = 0, g_iDepthHeight = 0;

UINT	g_uDepthPointNum = 0;
UINT	g_uColorPointNum = 0;
UINT	g_uColorBufferSize = 0;

UINT16*	g_pDepthBuffer = nullptr;
BYTE*	g_pColorBuffer = nullptr;
CameraSpacePoint* g_pCSPoints = nullptr;
CameraSpacePoint* g_totalPoints = nullptr;

// global variables
cv::Mat	g_mColorImg;
cv::Mat	g_mImg = cv::Mat(g_iColorHeight, g_iColorWidth, CV_8UC4);
bool g_isCapture = false;
int g_CaptureNum = 0;					// 抓取到的人體數目

int g_frame_count = 0;
int g_frame_count_for_standby = 0;

float g_positionX0[25] = { 0.0 };
float g_positionX1[25] = { 0.0 };
float g_positionY0[25] = { 0.0 };
float g_positionY1[25] = { 0.0 };
float g_positionZ0[25] = { 0.0 };
float g_positionZ1[25] = { 0.0 };

float g_velocityX[25] = { 0.0 };
float g_velocityY[25] = { 0.0 };
float g_velocityZ[25] = { 0.0 };
float g_current_velocity[25] = { 0.0 };
float g_previous_velocity[8] = { 0.0 };
float g_current_total_velocity = 0.0;
float g_current_average_velocity = 0.0;
float g_total_velocity = 0.0;
float g_average_velocity = 0.0;

// Debug:output the velocity of joints
ofstream current_average_velocityTXT("current_average_velocity.txt");
ofstream average_velocityTXT("average_velocity.txt");

// 對 App About 使用 CAboutDlg 對話方塊

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 對話方塊資料
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支援

// 程式碼實作
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CKinectcaptureDlg 對話方塊
CKinectcaptureDlg::CKinectcaptureDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CKinectcaptureDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CKinectcaptureDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CKinectcaptureDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON1, &CKinectcaptureDlg::OnBnClickedButton_Background)
	ON_BN_CLICKED(IDC_BUTTON2, &CKinectcaptureDlg::OnBnClickedButton_Capture)
	ON_BN_CLICKED(IDC_BUTTON3, &CKinectcaptureDlg::OnBnClickedButton_Output)
END_MESSAGE_MAP()


// CKinectcaptureDlg 訊息處理常式

BOOL CKinectcaptureDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 將 [關於...] 功能表加入系統功能表。

	// IDM_ABOUTBOX 必須在系統命令範圍之中。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 設定此對話方塊的圖示。當應用程式的主視窗不是對話方塊時，
	// 框架會自動從事此作業
	SetIcon(m_hIcon, TRUE);			// 設定大圖示
	SetIcon(m_hIcon, FALSE);		// 設定小圖示

	// TODO:  在此加入額外的初始設定
	AllocConsole();
	freopen("CONOUT$", "w", stdout);

	return TRUE;  // 傳回 TRUE，除非您對控制項設定焦點
}

void CKinectcaptureDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果將最小化按鈕加入您的對話方塊，您需要下列的程式碼，
// 以便繪製圖示。對於使用文件/檢視模式的 MFC 應用程式，
// 框架會自動完成此作業。

void CKinectcaptureDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 繪製的裝置內容

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 將圖示置中於用戶端矩形
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 描繪圖示
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// 當使用者拖曳最小化視窗時，
// 系統呼叫這個功能取得游標顯示。
HCURSOR CKinectcaptureDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

void default_sensor() {
	cout << "Try to get default sensor" << endl;
	if (GetDefaultKinectSensor(&g_pSensor) != S_OK) {
		cerr << "Get Sensor failed" << endl;
		return;
	}

	cout << "Try to open sensor" << endl;
	if (g_pSensor->Open() != S_OK)	 {
		cerr << "Can't open sensor" << endl;
		return;
	}
}

void color_source() {
	cout << "Try to get color source" << endl;
	// Get frame source
	IColorFrameSource* pFrameSource = nullptr;
	if (g_pSensor->get_ColorFrameSource(&pFrameSource) != S_OK) {
		cerr << "Can't get color frame source" << endl;
		return;
	}

	// Get frame description
	cout << "get color frame description" << endl;
	IFrameDescription* pFrameDescription = nullptr;
	if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)	{
		pFrameDescription->get_Width(&g_iColorWidth);
		pFrameDescription->get_Height(&g_iColorHeight);

		g_uColorPointNum = g_iColorWidth * g_iColorHeight;
		g_uColorBufferSize = g_uColorPointNum * 4 * sizeof(BYTE);

		g_pCSPoints = new CameraSpacePoint[g_uColorPointNum];
		g_totalPoints = new CameraSpacePoint[8 * g_uColorPointNum];
		g_pColorBuffer = new BYTE[4 * g_uColorPointNum];
	}
	pFrameDescription->Release();
	pFrameDescription = nullptr;

	// get frame reader
	cout << "Try to get color frame reader" << endl;
	if (pFrameSource->OpenReader(&g_pColorFrameReader) != S_OK) {
		cerr << "Can't get color frame reader" << endl;
		return;
	}

	// release Frame source
	cout << "Release frame source" << endl;
	pFrameSource->Release();
	pFrameSource = nullptr;
}

void depth_source() {
	cout << "Try to get depth source" << endl;
	// Get frame source
	IDepthFrameSource* pFrameSource = nullptr;
	if (g_pSensor->get_DepthFrameSource(&pFrameSource) != S_OK) {
		cerr << "Can't get depth frame source" << endl;
		return;
	}

	// Get frame description
	cout << "get depth frame description" << endl;
	IFrameDescription* pFrameDescription = nullptr;
	if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK) {
		pFrameDescription->get_Width(&g_iDepthWidth);
		pFrameDescription->get_Height(&g_iDepthHeight);
		g_uDepthPointNum = g_iDepthWidth * g_iDepthHeight;
		g_pDepthBuffer = new UINT16[g_uDepthPointNum];
	}
	pFrameDescription->Release();
	pFrameDescription = nullptr;

	// get frame reader
	cout << "Try to get depth frame reader" << endl;
	if (pFrameSource->OpenReader(&g_pDepthFrameReader) != S_OK) {
		cerr << "Can't get depth frame reader" << endl;
		return;
	}

	// release Frame source
	cout << "Release frame source" << endl;
	pFrameSource->Release();
	pFrameSource = nullptr;
}

void body_source() {
	cout << "Try to get body source" << endl;
	// Get frame source
	IBodyFrameSource* pFrameSource = nullptr;
	if (g_pSensor->get_BodyFrameSource(&pFrameSource) != S_OK) {
		cerr << "Can't get body frame source" << endl;
		return;
	}

	// Get the number of body
	if (pFrameSource->get_BodyCount(&g_iBodyCount) != S_OK) {
		cerr << "Can't get body count" << std::endl;
		return;
	}
	std::cout << "Can trace " << g_iBodyCount << " bodies" << std::endl;
	g_aBodyData = new IBody*[g_iBodyCount];
	for (int i = 0; i < g_iBodyCount; ++i)
		g_aBodyData[i] = nullptr;

	// get frame reader
	cout << "Try to get body frame reader" << endl;
	if (pFrameSource->OpenReader(&g_pBodyFrameReader) != S_OK) {
		cerr << "Can't get body frame reader" << endl;
		return;
	}

	// release Frame source
	cout << "Release frame source" << endl;
	pFrameSource->Release();
	pFrameSource = nullptr;
}

void map_coordinate() {
	// Read depth data
	if (g_pSensor->get_CoordinateMapper(&g_pCoordinateMapper) != S_OK) {
		std::cout << "Can't get coordinate mapper" << std::endl;
		return;
	}

	IDepthFrame* pDFrame = nullptr;
	if (g_pDepthFrameReader->AcquireLatestFrame(&pDFrame) == S_OK)
	{
		pDFrame->CopyFrameDataToArray(g_uDepthPointNum, g_pDepthBuffer);

		pDFrame->Release();
		pDFrame = nullptr;

		// map to camera space
		g_pCoordinateMapper->MapColorFrameToCameraSpace(g_uDepthPointNum, g_pDepthBuffer, g_uColorPointNum, g_pCSPoints);
	}
}

void DrawLine(cv::Mat& rImg, const Joint& rJ1, const Joint& rJ2, ICoordinateMapper* pCMapper) {
	if (rJ1.TrackingState == TrackingState_NotTracked || rJ2.TrackingState == TrackingState_NotTracked)
		return;

	ColorSpacePoint ptJ1, ptJ2;
	pCMapper->MapCameraPointToColorSpace(rJ1.Position, &ptJ1);
	pCMapper->MapCameraPointToColorSpace(rJ2.Position, &ptJ2);

	cv::line(rImg, cv::Point((int)ptJ1.X, (int)ptJ1.Y), cv::Point((int)ptJ2.X, (int)ptJ2.Y), cv::Vec3b(0, 0, 255), 5);
}

void get_latest_color_frame() {
	IColorFrame* pColorFrame = nullptr;
	while (g_pColorFrameReader->AcquireLatestFrame(&pColorFrame) != S_OK) {
		if (g_pColorFrameReader->AcquireLatestFrame(&pColorFrame) == S_OK)
			break;		
	}	
	g_mColorImg = cv::Mat(g_iColorHeight, g_iColorWidth, CV_8UC4);
	// Copy to OpenCV image
	if (pColorFrame->CopyConvertedFrameDataToArray(g_uColorBufferSize, g_mColorImg.data, ColorImageFormat_Bgra) != S_OK)	 {
		cerr << "Data copy error" << endl;
	}
	// release frame
	pColorFrame->Release();
}

void draw_joint_position(IBody* pBody, Joint* aJoints) {
	// get joint position
	if (pBody->GetJoints(JointType::JointType_Count, aJoints) == S_OK) {
		DrawLine(g_mImg, aJoints[JointType_SpineBase], aJoints[JointType_SpineMid], g_pCoordinateMapper);
		DrawLine(g_mImg, aJoints[JointType_SpineMid], aJoints[JointType_SpineShoulder], g_pCoordinateMapper);
		DrawLine(g_mImg, aJoints[JointType_SpineShoulder], aJoints[JointType_Neck], g_pCoordinateMapper);
		DrawLine(g_mImg, aJoints[JointType_Neck], aJoints[JointType_Head], g_pCoordinateMapper);

		DrawLine(g_mImg, aJoints[JointType_SpineShoulder], aJoints[JointType_ShoulderLeft], g_pCoordinateMapper);
		DrawLine(g_mImg, aJoints[JointType_ShoulderLeft], aJoints[JointType_ElbowLeft], g_pCoordinateMapper);
		DrawLine(g_mImg, aJoints[JointType_ElbowLeft], aJoints[JointType_WristLeft], g_pCoordinateMapper);
		DrawLine(g_mImg, aJoints[JointType_WristLeft], aJoints[JointType_HandLeft], g_pCoordinateMapper);
		DrawLine(g_mImg, aJoints[JointType_HandLeft], aJoints[JointType_HandTipLeft], g_pCoordinateMapper);
		DrawLine(g_mImg, aJoints[JointType_HandLeft], aJoints[JointType_ThumbLeft], g_pCoordinateMapper);

		DrawLine(g_mImg, aJoints[JointType_SpineShoulder], aJoints[JointType_ShoulderRight], g_pCoordinateMapper);
		DrawLine(g_mImg, aJoints[JointType_ShoulderRight], aJoints[JointType_ElbowRight], g_pCoordinateMapper);
		DrawLine(g_mImg, aJoints[JointType_ElbowRight], aJoints[JointType_WristRight], g_pCoordinateMapper);
		DrawLine(g_mImg, aJoints[JointType_WristRight], aJoints[JointType_HandRight], g_pCoordinateMapper);
		DrawLine(g_mImg, aJoints[JointType_HandRight], aJoints[JointType_HandTipRight], g_pCoordinateMapper);
		DrawLine(g_mImg, aJoints[JointType_HandRight], aJoints[JointType_ThumbRight], g_pCoordinateMapper);

		DrawLine(g_mImg, aJoints[JointType_SpineBase], aJoints[JointType_HipLeft], g_pCoordinateMapper);
		DrawLine(g_mImg, aJoints[JointType_HipLeft], aJoints[JointType_KneeLeft], g_pCoordinateMapper);
		DrawLine(g_mImg, aJoints[JointType_KneeLeft], aJoints[JointType_AnkleLeft], g_pCoordinateMapper);
		DrawLine(g_mImg, aJoints[JointType_AnkleLeft], aJoints[JointType_FootLeft], g_pCoordinateMapper);

		DrawLine(g_mImg, aJoints[JointType_SpineBase], aJoints[JointType_HipRight], g_pCoordinateMapper);
		DrawLine(g_mImg, aJoints[JointType_HipRight], aJoints[JointType_KneeRight], g_pCoordinateMapper);
		DrawLine(g_mImg, aJoints[JointType_KneeRight], aJoints[JointType_AnkleRight], g_pCoordinateMapper);
		DrawLine(g_mImg, aJoints[JointType_AnkleRight], aJoints[JointType_FootRight], g_pCoordinateMapper);
	}
}

void get_latest_body_frame(IBodyFrame* pBodyFrame) {
	while (true) {
		if (pBodyFrame->GetAndRefreshBodyData(g_iBodyCount, g_aBodyData) == S_OK) {
			break;
		}
	}
}

// void absdiff(input background image, input human image, output absdiff color image, output absdiff gray image, output absdiff binary image)
void absdiff(char* background_color_image, char* human_color_image, char* absdiff_color_image, char* absdiff_gray_image, char* absdiff_binary_image)
{
	cv::Mat background_color = cv::imread(background_color_image);	// get the background image frame
	cv::Mat human_color = cv::imread(human_color_image);		// the image that is constantly being updated
	cv::Mat absdiff_color = cv::Mat(cv::Size(human_color.rows, human_color.cols), CV_8UC1);
	cv::absdiff(background_color, human_color, absdiff_color);
	cv::Mat absdiff_gray = cv::Mat(cv::Size(absdiff_color.rows, absdiff_color.cols), CV_8UC1);
	cv::Mat absdiff_binary = cv::Mat(cv::Size(absdiff_color.rows, absdiff_color.cols), CV_8UC1);
	cv::cvtColor(absdiff_color, absdiff_gray, CV_RGB2GRAY);
	cv::threshold(absdiff_gray, absdiff_binary, 40, 255, cv::THRESH_BINARY);
	// Save the difference image
	cv::imwrite(absdiff_color_image, absdiff_color);
	cv::imwrite(absdiff_gray_image, absdiff_gray);
	cv::imwrite(absdiff_binary_image, absdiff_binary);

	background_color.release();
	human_color.release();
	absdiff_color.release();
	absdiff_gray.release();
	absdiff_binary.release();
}

// void watershed(input absdiff image, output mask image)
void watershed(char* absdiff_color_image, char* watershed_segment_image)
{
	cv::Mat absdiff_color = cv::imread(absdiff_color_image);
	cv::Mat absdiff_gray = cv::Mat(cv::Size(absdiff_color.rows, absdiff_color.cols), CV_8UC1);
	cv::cvtColor(absdiff_color, absdiff_gray, CV_BGR2GRAY);
	cv::Mat absdiff_binary = cv::Mat(cv::Size(absdiff_color.rows, absdiff_color.cols), CV_8UC1);
	cv::threshold(absdiff_gray, absdiff_binary, 40, 255, cv::THRESH_BINARY);

	// Eliminate noise and smaller objects
	cv::Mat fg;
	cv::erode(absdiff_binary, fg, cv::Mat(), cv::Point(-1, -1), 2);

	// Identify image pixels without objects
	cv::Mat bg;
	cv::dilate(absdiff_binary, bg, cv::Mat(), cv::Point(-1, -1), 3);
	cv::threshold(bg, bg, 1, 128, cv::THRESH_BINARY_INV);

	// Create markers image
	cv::Mat markers(absdiff_binary.size(), CV_8U, cv::Scalar(0));
	markers = fg + bg;

	markers.convertTo(markers, CV_32S);
	cv::watershed(absdiff_color, markers);

	markers.convertTo(markers, CV_8U);
	cv::threshold(markers, markers, 128, 255, cv::THRESH_BINARY);
	cv::imwrite(watershed_segment_image, markers);

	// Release memory of opencv matrix
	//absdiff_color.release();
	//absdiff_gray.release();
	absdiff_binary.release();
	fg.release();
	bg.release();
	markers.release();
}

// void human_mask(input mask image, output mask asc file)
void human_mask(char* watershed_segment_image, char* human_3Dpoints_asc)
{
	// Size of input picture, the watershed segmented picture
	const int width = g_iColorWidth;
	const int height = g_iColorHeight;
	const long int total_pixels = width * height;

	cv::Mat img = cv::imread(watershed_segment_image);
	// An one dimensional array to index whether the grayscale of each pixel is white
	short* idx;
	idx = new short[total_pixels];
	memset(idx, 0, total_pixels*sizeof(short));
	// load grayscale of each pixel in the mask
	for (int j = 0; j < img.rows; j++) {
		for (int i = 0; i < img.cols; i++) {
			cv::Scalar intensity = img.at<cv::Vec3b>(j, i);
			// when the mask is white
			if (intensity.val[0] == 255) {
				idx[i + j * img.cols] = 1;
			}
		}
	}

	// initialize the output asc file
	std::ofstream human_3Dpoints(human_3Dpoints_asc);
	//std::ofstream colored_human_asc("colored_human_point.asc");

	//int pixel_height_of_ankle = 750;
	// output human point cloud above the ankle segmented by bounding box

	for (long int i = 0; i < width * ANKLE_HEIGHT; i++) {
		if (g_totalPoints[(g_CaptureNum)* total_pixels + i].Z > X_FRONT && g_totalPoints[(g_CaptureNum)* total_pixels + i].Z != 0 && g_totalPoints[(g_CaptureNum)* total_pixels + i].Z < X_BACK && g_totalPoints[(g_CaptureNum)* total_pixels + i].X < Y_LEFT && g_totalPoints[(g_CaptureNum)* total_pixels + i].X > Y_RIGHT && g_totalPoints[(g_CaptureNum)* total_pixels + i].Y > Z_HEIGHT) {
			human_3Dpoints << -g_totalPoints[(g_CaptureNum)* total_pixels + i].Z << " " << g_totalPoints[(g_CaptureNum)* total_pixels + i].X << " " << g_totalPoints[(g_CaptureNum)* total_pixels + i].Y << " " << endl;
		}
		//if (Z[i] < X_FRONT && Z[i] != 0 && Z[i] > X_BACK && X[i] < Y_LEFT && X[i] > Y_RIGHT && Y[i] > Z_HEIGHT) {
		//	colored_human_asc << X[i] << " " << Y[i] << " " << Z[i] << " " << endl;
		//}
	}

	// output human point cloud below the ankle segmented by Absdiff  
	for (long int i = (width * ANKLE_HEIGHT); i < total_pixels; i++) {
		if (idx[i] == 1) {
			if (g_totalPoints[(g_CaptureNum)* total_pixels + i].Z > X_FRONT && g_totalPoints[(g_CaptureNum)* total_pixels + i].Z != 0 && g_totalPoints[i].Z < X_BACK && g_totalPoints[(g_CaptureNum)* total_pixels + i].X < Y_LEFT && g_totalPoints[(g_CaptureNum)* total_pixels + i].X > Y_RIGHT && g_totalPoints[(g_CaptureNum)* total_pixels + i].Y > Z_HEIGHT) {
				human_3Dpoints << -g_totalPoints[(g_CaptureNum)* total_pixels + i].Z << " " << g_totalPoints[(g_CaptureNum)* total_pixels + i].X << " " << g_totalPoints[(g_CaptureNum)* total_pixels + i].Y << " " << endl;
			}
		}
		//if (idx[i]) {
		//	if (Z[i] < X_FRONT && Z[i] != 0 && Z[i] > X_BACK && X[i] < Y_LEFT && X[i] > Y_RIGHT && Y[i] > Z_HEIGHT) {
		//		colored_human_asc << X[i] << " " << Y[i] << " " << Z[i] << " " << endl;
		//	}
		//}
	}

	// release the memory of array
	delete[] idx;
	// close the asc file
	human_3Dpoints.close();
	//colored_human_asc.close();
	// release the memory of opencv matrix
	//markers.release();
	//cv::waitKey(0);
}

void capture_human_point() {
	// Read color data
	IColorFrame* pCFrame = nullptr;
	if (g_pColorFrameReader->AcquireLatestFrame(&pCFrame) == S_OK) {
		// 使用openCV擷取Kinect畫面
		cv::Mat	human_color(g_iColorHeight, g_iColorWidth, CV_8UC4);
		cv::Mat human_gray(g_iColorHeight, g_iColorWidth, CV_8UC4);
		//char* win_name = "Color Image";
		//cv::namedWindow(win_name, 0);									// 創造彩色圖片顯示視窗

		char* background_image = "background_color.bmp";

		char human_color_image[30] = "human_color1.bmp";	
		char absdiff_color_image[30] = "absdiff_color1.bmp";
		char absdiff_gray_image[30] = "absdiff_gray1.bmp";
		char absdiff_binary_image[30] = "absdiff_binary1.bmp";
		char watershed_segment_image[30] = "watershed_segment1.bmp";
		sprintf(human_color_image, "human_color%d.bmp", g_CaptureNum);
		sprintf(absdiff_color_image, "absdiff_color%d.bmp", g_CaptureNum);
		sprintf(absdiff_gray_image, "absdiff_gray%d.bmp", g_CaptureNum);
		sprintf(absdiff_binary_image, "absdiff_binary%d.bmp", g_CaptureNum);
		sprintf(watershed_segment_image, "watershed_segment%d.bmp", g_CaptureNum);

		if (pCFrame->CopyConvertedFrameDataToArray(g_uColorBufferSize, human_color.data, ColorImageFormat_Bgra) == S_OK)	{
			if ((int)g_pColorBuffer[0] != 0) {
				cv::cvtColor(human_color, human_gray, CV_RGB2GRAY);		// 彩色轉灰階
				cv::imwrite(human_color_image, human_color);			// 儲存彩色畫面
				absdiff(human_color_image, background_image, absdiff_color_image, absdiff_gray_image, absdiff_binary_image);
				watershed(absdiff_color_image, watershed_segment_image);
				g_CaptureNum++;
			}
		}
		pCFrame->Release();
		pCFrame = nullptr;
	}

	for (int y = 0; y < g_iColorHeight; ++y) {
		for (int x = 0; x < g_iColorWidth; ++x) {
			int idx = x + y * g_iColorWidth;
			g_totalPoints[(g_CaptureNum)* idx] = g_pCSPoints[idx];
		}
	}
}

void joint_driven() {
	// Get latest color frame
	get_latest_color_frame();
	g_mImg = g_mColorImg.clone();

	IBodyFrame* pBodyFrame = nullptr;
	if (g_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame) == S_OK) {
		// get Body data
		get_latest_body_frame(pBodyFrame);
		if (pBodyFrame->GetAndRefreshBodyData(g_iBodyCount, g_aBodyData) == S_OK) {
			// for each body
			IBody* pBody;
			Joint aJoints[JointType::JointType_Count];
			for (int i = 0; i < g_iBodyCount; ++i) {
				pBody = g_aBodyData[i];
				// check if is tracked
				BOOLEAN bTracked = false;
				if ((pBody->get_IsTracked(&bTracked) == S_OK) && bTracked) {
					draw_joint_position(pBody, aJoints);
					//cv::imshow("Body Image", g_mImg);					// show image

					// Debug:print out the number of frame					
					std::cout << "frame " << ++g_frame_count << std::endl;
					// Update the previous 8 frame velocity
					g_total_velocity = g_previous_velocity[0];
					for (int j = 0; j < 7; j++) {
						g_previous_velocity[j + 1] = g_previous_velocity[j];
						g_total_velocity += g_previous_velocity[j + 1];
					}
					g_average_velocity = (float)(g_total_velocity / 8.0);

					if (g_average_velocity <= 0.0015 && g_average_velocity != 0) {
						// determine if the person is still 
						if (g_frame_count_for_standby == 0) {
							PlaySound(TEXT("Alarm02.wav"), NULL, SND_FILENAME);
							std::cout << "Start capturing points!" << std::endl;
							capture_human_point();
						}
						// count the number of frame whose velocity is below the threshold
						g_frame_count_for_standby++;
						if (g_frame_count_for_standby >= 5) {
							g_frame_count_for_standby = 0;
							//isCapture = false;
						}
					}
					// Debug:output the average velocity 
					average_velocityTXT << g_frame_count << " " << g_average_velocity << std::endl;
					g_total_velocity = 0;
					// Update the average velocity
					int available_joints = 0;
					for (int i = 0; i < 25; i++) {
						// X 
						g_positionX1[i] = g_positionX0[i];
						g_positionX0[i] = aJoints[i].Position.X;
						g_velocityX[i] = (g_positionX1[i] - g_positionX0[i]) * (g_positionX1[i] - g_positionX0[i]);
						// Y
						g_positionY1[i] = g_positionY0[i];
						g_positionY0[i] = aJoints[i].Position.Y;
						g_velocityY[i] = (g_positionY1[i] - g_positionY0[i]) * (g_positionY1[i] - g_positionY0[i]);
						// Z
						g_positionZ1[i] = g_positionZ0[i];
						g_positionZ0[i] = aJoints[i].Position.Z;
						g_velocityZ[i] = (g_positionZ1[i] - g_positionZ0[i]) * (g_positionZ1[i] - g_positionZ0[i]);
						g_current_velocity[i] = sqrtf(g_velocityX[i] + g_velocityY[i] + g_velocityZ[i]);
						// exclude the discrete velocity
						if (g_current_velocity[i] < 0.01) {
							g_current_total_velocity += g_current_velocity[i];
							available_joints++;
						}
					}
					// If no joint is available, save the velocity of last frame
					if (available_joints != 0) {
						g_current_average_velocity = g_current_total_velocity / available_joints;
					}
					g_previous_velocity[0] = g_current_average_velocity;
					// Debug:output the current average velocity 
					current_average_velocityTXT << g_frame_count << " " << g_current_average_velocity << std::endl;

					g_current_total_velocity = 0;
				}
			}
		}
		// release frame
		pBodyFrame->Release();
	}

}

void end_joint_driven() {
	// Delete body data array
	delete[] g_aBodyData;
	// Delete buffer
	delete[] g_pDepthBuffer;
	delete[] g_pColorBuffer;
	delete[] g_pCSPoints;

	// Release frame reader
	std::cout << "Release body frame reader" << std::endl;
	g_pBodyFrameReader->Release();
	g_pBodyFrameReader = nullptr;
	// Release color frame reader
	std::cout << "Release color frame reader" << std::endl;
	g_pColorFrameReader->Release();
	g_pColorFrameReader = nullptr;
	// Release depth frame reader
	g_pDepthFrameReader->Release();
	g_pDepthFrameReader = nullptr;
	// release coordinate mapper
	g_pCoordinateMapper->Release();
	g_pCoordinateMapper = nullptr;
	// Close Sensor
	std::cout << "close sensor" << std::endl;
	g_pSensor->Close();
	// Release Sensor
	std::cout << "Release sensor" << std::endl;
	g_pSensor->Release();
	g_pSensor = nullptr;
}

// To test...
void output_asc() {
	for (g_CaptureNum = 0; g_CaptureNum < 8; g_CaptureNum++) {
		char watershed_segment_image[30] = "watershed_segment1.bmp";
		sprintf(watershed_segment_image, "watershed_segment%d.bmp", g_CaptureNum);

		char human_3Dpoints_asc[30] = "human_3Dpoints1.asc";
		sprintf(human_3Dpoints_asc, "human_3Dpoints%d.asc", g_CaptureNum);

		human_mask(watershed_segment_image, human_3Dpoints_asc);
	}
	delete[] g_totalPoints;
	cout << "Process finished!" << endl;
}

void CKinectcaptureDlg::OnBnClickedButton_Background()
{
	// Sensor related code
	default_sensor();
	// Color related code
	color_source();

	IColorFrame* pCFrame = nullptr;
	while (g_pColorFrameReader->AcquireLatestFrame(&pCFrame) != S_OK) {
		if (g_pColorFrameReader->AcquireLatestFrame(&pCFrame) == S_OK)
			break;
	}

	// 使用openCV擷取Kinect畫面
	cv::Mat	background_color(g_iColorHeight, g_iColorWidth, CV_8UC4);
	cv::Mat background_gray(g_iColorHeight, g_iColorWidth, CV_8UC4);
	char* window_name = "Background Image";
	cv::namedWindow(window_name, 0);									// 創造彩色背景顯示視窗
	if (pCFrame->CopyConvertedFrameDataToArray(g_uColorBufferSize, background_color.data, ColorImageFormat_Bgra) == S_OK)	{	
		cv::cvtColor(background_color, background_gray, CV_RGB2GRAY);	// 彩色轉灰階
		cv::imshow(window_name, background_color);						// 顯示彩色的背景
		cv::imwrite("background_color.bmp", background_color);			// 輸出彩色的背景
		//cv::imwrite("background_gray.bmp", background_gray);			// 輸出灰階的背景
		cv::waitKey();
	}
	cv::destroyWindow(window_name);
	pCFrame->Release();
	pCFrame = nullptr;

}

void CKinectcaptureDlg::OnBnClickedButton_Capture()
{
	// Sensor related code
	default_sensor();
	// Color related code
	color_source();
	// Depth related code
	depth_source();
	// Body releated code
	body_source();
	// coordinate mapper code
	map_coordinate();

	while (g_CaptureNum <= 7) {
		joint_driven();
	}
	end_joint_driven();
}

// To test...
void CKinectcaptureDlg::OnBnClickedButton_Output()
{
	output_asc();
}
