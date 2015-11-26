// Created by Heresy @ 2015/02/03
// Blog Page: https://kheresy.wordpress.com/2015/02/11/k4w-v2-part-6-draw-with-opengl/
// This sample is used to show Kinect data in cloud points form.
// Revised by YongYuH @2015/11/26


// Standard Library
#include <array>
#include <iostream>

// freeglut header
#include "GL/freeglut.h"	

// Kinect for Windows SDK Header
#include <Kinect.h>

// for OpenGL camera navigation
#include "common/OpenGLCamera.h"

// output asc file
#include <fstream>

// OpenCV Header
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// Link OpenCV Library
#ifdef _DEBUG
#pragma comment( lib, "opencv_core249d.lib" )
#pragma comment( lib, "opencv_highgui249d.lib" )
#pragma comment( lib, "opencv_imgproc249d.lib")
#else
#pragma comment( lib, "opencv_core249.lib" )
#pragma comment( lib, "opencv_highgui249.lib" )
#pragma comment( lib, "opencv_imgproc249.lib")
#endif

// 開啟或關閉抓三維點功能
#define ASC
// 開啟或關閉抓彩色圖功能
#define HUMANCOLORIMAGE
// 開啟或關閉抓主軸功能
//#define AXIS

using namespace std;

// global objects
IKinectSensor*		pSensor				= nullptr;
IColorFrameReader*	pColorFrameReader	= nullptr;
IDepthFrameReader*	pDepthFrameReader	= nullptr;
IBodyIndexFrameReader* pBIFrameReader	= nullptr;   // 遮罩用
ICoordinateMapper*	pCoordinateMapper	= nullptr;

int		iColorWidth	    = 0,
		iColorHeight    = 0;

int		iDepthWidth	    = 0,			// 遮罩用
		iDepthHeight    = 0;			// 遮罩用

UINT	uDepthPointNum		= 0;
UINT	uColorPointNum		= 0;
UINT	uColorBufferSize	= 0;

UINT16*	pDepthBuffer = nullptr;
BYTE*	pColorBuffer = nullptr;
CameraSpacePoint* pCSPoints = nullptr;

SimpleCamera g_Camera;

cv::Mat	imgColor(iColorHeight, iColorWidth, CV_8UC4);
cv::Mat imgTarget(iColorHeight, iColorWidth, CV_8UC3);

// glut display function(draw)
void display()
{
	// clear previous screen
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// draw points
	//glPointSize(1.0f);
	//glBegin(GL_POINTS);

	

	#ifdef ASC
	std::ofstream raw_point("raw_point.asc");							// 輸出3維點的asc檔
	std::ofstream for_icp("for_icp.asc");
	//std::ofstream point_cloud_with_rgb("point_cloud_with_rgb.asc");		// 輸出3維點+RGB資料的asc檔
	//std::ofstream axis_point("axis_point.asc");
	int count_number = 0;		// counting number
	if ((int)pColorBuffer[0] != 0) {	
		float tempX = 0.0;
		float tempZ = 0.0;
		int i = 0;
		cout << "Starting capture points!" << endl;

		#ifdef HUMANCOLORIMAGE
		// Read color data
		IColorFrame* pCFrame = nullptr;
		if (pColorFrameReader->AcquireLatestFrame(&pCFrame) == S_OK) {
			// 使用openCV擷取Kinect畫面
			cv::Mat	mImg(iColorHeight, iColorWidth, CV_8UC4);
			cv::Mat gray_image(iColorHeight, iColorWidth, CV_8UC4);
			//cv::namedWindow("Color Map");							// 創造彩色圖片顯示視窗
			cv::namedWindow("Color Image", 0);						// 創造灰階圖片顯示視窗
			if (pCFrame->CopyConvertedFrameDataToArray(uColorBufferSize, mImg.data, ColorImageFormat_Bgra) == S_OK)	{
				//cv::imshow("Color Map", mImg);					// 顯示彩色畫面		
				cv::cvtColor(mImg, gray_image, CV_RGB2GRAY);		// 彩色轉灰階
				cv::imshow("Color Image", mImg);				    // 顯示灰階畫面	
				//cv::imwrite("axis_color.bmp", mImg);			    // 輸出有雙軸的彩色畫面
				//cv::imwrite("background_color.bmp", mImg);			// 輸出背景的彩色畫面
				cv::imwrite("human_color.bmp", mImg);				// 輸出有人體的彩色畫面
				cout << "Output the gray image!" << endl;
				cout << "Press any key on the gray image window to continue!" << endl;
				cv::waitKey();
			}
			cv::destroyWindow("Color Image");
			pCFrame->Release();
			pCFrame = nullptr;
		}
		#endif	

		for (int y = 0; y < iColorHeight; ++y) {
			for (int x = 0; x < iColorWidth; ++x) {
				int idx = x + y * iColorWidth;
				CameraSpacePoint& rPt = pCSPoints[idx];	

				glColor4ubv((const GLubyte*)(&pColorBuffer[4 * idx]));
				glVertex3f(rPt.X, rPt.Y, rPt.Z);

				if (rPt.Z <= 0) {
					rPt.X = rPt.Y = rPt.Z = 0;
				}			
				// output all 3D points(x, y, z) as asc file
				//raw_point << rPt.X * 1000 << " " << rPt.Y * 1000 << " " << rPt.Z * 1000 << " " << endl;
				// 轉成實驗室所需的座標系
				raw_point << rPt.Z * -1000 << " " << rPt.X * 1000 << " " << rPt.Y * 1000 << " " << endl;

				// output asc file for icp process
				if (rPt.Z < 3.1 && rPt.Z != 0 && rPt.Z > 1.9 && rPt.X < 0.69 && rPt.X > -0.69 && rPt.Y > -0.88) {
					if (rPt.Y > -0.6 && rPt.Y < 2 && idx % 75 == 0) {
						//for_icp << rPt.X * 1000 << " " << rPt.Y * 1000 << " " << rPt.Z * 1000 << " " << endl;
						// 轉成實驗室所需的座標系
						for_icp << rPt.Z * -1000 << " " << rPt.X * 1000 << " " << rPt.Y * 1000 << " " << endl;
					}
				}
				

				#ifdef AXIS
				if (rPt.Z < 2.35 && rPt.Z != 0 && rPt.Z > 1.15 && rPt.X < 0.69 && rPt.X > -0.69 && rPt.Y > -0.88) {
					if (rPt.Y > -0.70 && rPt.Y < -0.65)	{				
						tempX += rPt.X;
						tempZ += rPt.Z;
						i++;
					}
				}				
				#endif
				// 將某一瞬間x, y, z值+RGB值輸出成asc檔	
				//point_cloud_with_rgb << rPt.X << " " << rPt.Y << " " << rPt.Z << " "
				//							<< (unsigned short)(pColorBuffer[4*idx]) << " "
				//							<< (unsigned short)(pColorBuffer[4*idx + 1]) << " "
				//							<< (unsigned short)(pColorBuffer[4*idx + 2]) << " " << endl;
				count_number++;				
			}		
		}	
	}
	// 關閉輸出3d點函式
	//axis_point.close();
	for_icp.close();
	raw_point.close();
	system("pause");
	//point_cloud_with_rgb.close();
	#endif
	
	//glEnd();

	// Coordinate
	//glLineWidth(5.0f);
	//glBegin(GL_LINES);
	//	glColor3ub(255, 0, 0);
	//	glVertex3f(0, 0, 0);
	//	glVertex3f(1, 0, 0);

	//	glColor3ub(0, 255, 0);
	//	glVertex3f(0, 0, 0);
	//	glVertex3f(0, 1, 0);

	//	glColor3ub(0, 0, 255);
	//	glVertex3f(0, 0, 0);
	//	glVertex3f(0, 0, 1);
	//glEnd();

	// swap buffer
	//glutSwapBuffers();
}

// glut idle function
void idle()
{
	// Read color data
	IColorFrame* pCFrame = nullptr;
	if (pColorFrameReader->AcquireLatestFrame(&pCFrame) == S_OK)
	{
		pCFrame->CopyConvertedFrameDataToArray( uColorBufferSize, pColorBuffer, ColorImageFormat_Rgba);

		pCFrame->Release();
		pCFrame = nullptr;
	}

	// Read depth data
	IDepthFrame* pDFrame = nullptr;
	if (pDepthFrameReader->AcquireLatestFrame(&pDFrame) == S_OK)
	{
		pDFrame->CopyFrameDataToArray(uDepthPointNum, pDepthBuffer);

		pDFrame->Release();
		pDFrame = nullptr;

		// map to camera space
		pCoordinateMapper->MapColorFrameToCameraSpace( uDepthPointNum, pDepthBuffer, uColorPointNum, pCSPoints);

		// redraw
		glutPostRedisplay();
	}
}

// glut keyboard function
//void keyboard(unsigned char key, int x, int y)
//{
//	float fSpeed = 0.1f;
//	switch (key)
//	{
//	case VK_ESCAPE:
//		glutExit();
//
//	case 's':
//		g_Camera.MoveForward(-fSpeed);
//		break;
//
//	case 'w':
//		g_Camera.MoveForward(fSpeed);
//		break;
//
//	case 'a':
//		g_Camera.MoveSide(-fSpeed);
//		break;
//
//	case 'd':
//		g_Camera.MoveSide(fSpeed);
//		break;
//
//	case 'z':
//		g_Camera.MoveUp(-fSpeed);
//		break;
//
//	case 'x':
//		g_Camera.MoveUp(fSpeed);
//		break;
//	}
//}

// glut special keyboard function
//void specialKey(int key, int x, int y)
//{
//	float fRotateScale = 0.01f;
//	switch (key)
//	{
//	case GLUT_KEY_DOWN:
//		g_Camera.RotateUp(-fRotateScale);
//		break;
//
//	case GLUT_KEY_UP:
//		g_Camera.RotateUp(fRotateScale);
//		break;
//
//	case GLUT_KEY_RIGHT:
//		g_Camera.RotateSide(fRotateScale);
//		break;
//
//	case GLUT_KEY_LEFT:
//		g_Camera.RotateSide(-fRotateScale);
//		break;
//	}
//}

//void reshape(int w, int h)
//{
//	glMatrixMode(GL_PROJECTION);
//	glLoadIdentity();
//	gluPerspective(40.0, (float)w / h, 0.01, 50.0);
//
//	g_Camera.SetCamera();
//
//	glViewport(0, 0, w, h);
//}

void ExitFunction()
{
	// release buffer
	delete[] pDepthBuffer;
	delete[] pColorBuffer;
	delete[] pCSPoints;

	// release coordinate mapper
	pCoordinateMapper->Release();
	pCoordinateMapper = nullptr;

	// release frame reader
	pColorFrameReader->Release();
	pColorFrameReader = nullptr;
	pDepthFrameReader->Release();
	pDepthFrameReader = nullptr;

	// Close and Release Sensor
	pSensor->Close();
	pSensor->Release();
	pSensor = nullptr;
}

int main(int argc, char** argv)
{
	// 1. Sensor related code
	cout << "Try to get default sensor" << endl;
	{
		if (GetDefaultKinectSensor(&pSensor) != S_OK)
		{
			cerr << "Get Sensor failed" << endl;
			return -1;
		}

		cout << "Try to open sensor" << endl;
		if (pSensor->Open() != S_OK)
		{
			cerr << "Can't open sensor" << endl;
			return -1;
		}
	}

	// 2. Color related code
	cout << "Try to get color source" << endl;
	{
		// Get frame source
		IColorFrameSource* pFrameSource = nullptr;
		if (pSensor->get_ColorFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get color frame source" << endl;
			return -1;
		}

		// Get frame description
		cout << "get color frame description" << endl;
		IFrameDescription* pFrameDescription = nullptr;
		if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
		{
			pFrameDescription->get_Width(&iColorWidth);
			pFrameDescription->get_Height(&iColorHeight);

			uColorPointNum = iColorWidth * iColorHeight;
			uColorBufferSize = uColorPointNum * 4 * sizeof(BYTE);

			pCSPoints = new CameraSpacePoint[uColorPointNum];
			pColorBuffer = new BYTE[4 * uColorPointNum];
		}
		pFrameDescription->Release();
		pFrameDescription = nullptr;

		// get frame reader
		cout << "Try to get color frame reader" << endl;
		if (pFrameSource->OpenReader(&pColorFrameReader) != S_OK)
		{
			cerr << "Can't get color frame reader" << endl;
			return -1;
		}

		// release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;
	}

	// 3. Depth related code
	cout << "Try to get depth source" << endl;
	{
		// Get frame source
		IDepthFrameSource* pFrameSource = nullptr;
		if (pSensor->get_DepthFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get depth frame source" << endl;
			return -1;
		}

		// Get frame description
		cout << "get depth frame description" << endl;
		IFrameDescription* pFrameDescription = nullptr;
		if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
		{
			int	iDepthWidth = 0,
				iDepthHeight = 0;
			pFrameDescription->get_Width(&iDepthWidth);
			pFrameDescription->get_Height(&iDepthHeight);
			uDepthPointNum = iDepthWidth * iDepthHeight;
			pDepthBuffer = new UINT16[uDepthPointNum];
		}
		pFrameDescription->Release();
		pFrameDescription = nullptr;

		// get frame reader
		cout << "Try to get depth frame reader" << endl;
		if (pFrameSource->OpenReader(&pDepthFrameReader) != S_OK)
		{
			cerr << "Can't get depth frame reader" << endl;
			return -1;
		}

		// release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;
	}

	// 3.5 Body Index releated code
	cout << "Try to get body index source" << endl;
	{
		// Get frame source
		IBodyIndexFrameSource* pFrameSource = nullptr;
		if (pSensor->get_BodyIndexFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get body index frame source" << endl;
			return -1;
		}

		// get frame reader
		cout << "Try to get body index frame reader" << endl;
		if (pFrameSource->OpenReader(&pBIFrameReader) != S_OK)
		{
			cerr << "Can't get depth frame reader" << endl;
			return -1;
		}

		// release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;
	}

	// 4. Coordinate Mapper
	if (pSensor->get_CoordinateMapper(&pCoordinateMapper) != S_OK)
	{
		cerr << "get_CoordinateMapper failed" << endl;
		return -1;
	}
	
	// 5. initial glut
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB);

	glutInitWindowSize(640, 480);
	glutCreateWindow("Kinect OpenGL 3D Point");

	glEnable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);

	// default camera
	//g_Camera.vCenter = Vector3(0.0, 0.0, 1);
	//g_Camera.vPosition = Vector3(0.0, 0.0, -2.0);
	//g_Camera.vUpper = Vector3(0.0, 1.0, 0.0);

	// register glut callback functions
	glutDisplayFunc(display);
	glutIdleFunc(idle);
	//glutReshapeFunc(reshape);
	//glutKeyboardFunc(keyboard);
	//glutSpecialFunc(specialKey);

	// STD Exit function
	atexit(ExitFunction);

	// start
	glutMainLoop();

	return 0;
}
