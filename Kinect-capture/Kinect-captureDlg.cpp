
// Kinect-captureDlg.cpp : 實作檔

#include "stdafx.h"
#include "Kinect-capture.h"
#include "Kinect-captureDlg.h"
#include "afxdialogex.h"

#include <iostream>
#include <fstream>
#include <math.h>
#include <sstream>
#include <string>

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

// Threshold for mask (unit:m)
#define ANKLE_HEIGHT (750)
#define X_BACK (3.300)
#define X_FRONT (1.900)
#define Y_LEFT (0.690)
#define Y_RIGHT (-0.690)
#define Z_HEIGHT (-0.950)

float X_back_human;
float X_front_human;
float Y_left_human;
float Y_right_human;
float Z_human;
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
BYTE*   g_pTotalColor = nullptr;
CameraSpacePoint* g_pCSPoints = nullptr;
CameraSpacePoint* g_pboardPoints = nullptr;
CameraSpacePoint* g_ptotalPoints = nullptr;

cv::Mat	g_display(g_iColorHeight, g_iColorWidth, CV_8UC4);

// global variables
cv::Mat	g_mColorImg;
cv::Mat	g_mImg = cv::Mat(g_iColorHeight, g_iColorWidth, CV_8UC4);

IBody* pBody;
Joint aJoints[JointType::JointType_Count];

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

// global output file name
char g_watershed_segment_image[30] = "watershed_segment0.bmp";
char g_human_3Dpoints_asc[30] = "human_3Dpoints0.asc";
char g_color_human_3Dpoints_cpt[30] = "color_human_3Dpoints0.txt";

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
		g_pboardPoints = new CameraSpacePoint[g_uColorPointNum];
		g_ptotalPoints = new CameraSpacePoint[8 * g_uColorPointNum];
		g_pColorBuffer = new BYTE[4 * g_uColorPointNum];
		g_pTotalColor = new BYTE[8 * 4 * g_uColorPointNum];
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
		std::cerr << "Can't get coordinate mapper" << std::endl;
		return;
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

void capture_human_image() {
	// Read color data
	IColorFrame* pCFrame = nullptr;
	if (g_pColorFrameReader->AcquireLatestFrame(&pCFrame) == S_OK) {
		// Use OpenCV to get the image from Kinect
		cv::Mat	human_color(g_iColorHeight, g_iColorWidth, CV_8UC4);
		cv::Mat human_gray(g_iColorHeight, g_iColorWidth, CV_8UC4);

		char* background_image = "background_board_color.bmp";

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
				cv::cvtColor(human_color, human_gray, CV_RGB2GRAY);		// convert color image to gray image
				cv::imwrite(human_color_image, human_color);			// Store the color image
				absdiff(human_color_image, background_image, absdiff_color_image, absdiff_gray_image, absdiff_binary_image);
				watershed(absdiff_color_image, watershed_segment_image);
			}
			// To test: the index
			for (int y = 0; y < g_iColorHeight; ++y) {
				for (int x = 0; x < g_iColorWidth; ++x) {
					int idx = 4 * (x + y * g_iColorWidth);
					g_pTotalColor[4 * g_CaptureNum * g_uColorPointNum + idx] = human_color.data[idx];		// Red
					g_pTotalColor[4 * g_CaptureNum * g_uColorPointNum + idx + 1] = human_color.data[idx + 1];	// Green
					g_pTotalColor[4 * g_CaptureNum * g_uColorPointNum + idx + 2] = human_color.data[idx + 2];	// Blue
					g_pTotalColor[4 * g_CaptureNum * g_uColorPointNum + idx + 3] = human_color.data[idx + 3];	// Alpha
				}
			}
		}

		g_display = human_color.clone();

		pCFrame->Release();
		pCFrame = nullptr;
	}
}

void capture_human_3Dpoints() {
	// Read depth data
	IDepthFrame* pDFrame = nullptr;
	if (g_pDepthFrameReader->AcquireLatestFrame(&pDFrame) == S_OK) {
		pDFrame->CopyFrameDataToArray(g_uDepthPointNum, g_pDepthBuffer);
		pDFrame->Release();
		pDFrame = nullptr;
		// map to camera space
		g_pCoordinateMapper->MapColorFrameToCameraSpace(g_uDepthPointNum, g_pDepthBuffer, g_uColorPointNum, g_pCSPoints);
		for (int y = 0; y < g_iColorHeight; ++y) {
			for (int x = 0; x < g_iColorWidth; ++x) {
				int idx = x + y * g_iColorWidth;
				g_ptotalPoints[g_CaptureNum * g_uColorPointNum + idx].X = g_pCSPoints[idx].X;
				g_ptotalPoints[g_CaptureNum * g_uColorPointNum + idx].Y = g_pCSPoints[idx].Y;
				g_ptotalPoints[g_CaptureNum * g_uColorPointNum + idx].Z = g_pCSPoints[idx].Z;
			}
		}
	}
}

void get_joint_position() {
	IBodyFrame* pBodyFrame = nullptr;

	if (g_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame) == S_OK) {
		// get Body data
		if (pBodyFrame->GetAndRefreshBodyData(g_iBodyCount, g_aBodyData) == S_OK) {
			// for each body		
			for (int i = 0; i < g_iBodyCount; ++i) {
				pBody = g_aBodyData[i];
				// check if is tracked
				BOOLEAN bTracked = false;
				if ((pBody->get_IsTracked(&bTracked) == S_OK) && bTracked) {
					draw_joint_position(pBody, aJoints);
					//cv::imshow("Body Image", g_mImg);					// show image					
				}
			}
			// Debug:print out the number of frame					
			std::cout << "frame " << ++g_frame_count << std::endl;
		}
		// release frame
		pBodyFrame->Release();
		pBodyFrame = nullptr;
	}
}

void capture_human_data() {
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
	//current_average_velocityTXT << g_frame_count << " " << g_current_average_velocity << std::endl;

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
			std::cout << "Start capturing point" << g_CaptureNum << std::endl;
			// To test: the wait time to avoid ghost
			for (int i = 0; i < 15; i++) {
				std::cout << "capture frame: " << ++g_frame_count << std::endl;
			}
			capture_human_3Dpoints();
			capture_human_image();
			if (g_CaptureNum == 7) {
				PlaySound(TEXT("clap.wav"), NULL, SND_FILENAME);
			}
			else {
				PlaySound(TEXT("door_ring.wav"), NULL, SND_FILENAME);
			}
			g_CaptureNum++;
		}
		// count the number of frame whose velocity is below the threshold
		g_frame_count_for_standby++;
		if (g_frame_count_for_standby >= 5) {
			g_frame_count_for_standby = 0;
		}
	}
	// Debug:output the average velocity 
	//average_velocityTXT << g_frame_count << " " << g_average_velocity << std::endl;
	g_total_velocity = 0;

	g_current_total_velocity = 0;
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

void human_mask()
{
	// Size of input picture, the watershed segmented picture
	const int width = g_iColorWidth;
	const int height = g_iColorHeight;
	const long int total_pixels = width * height;

	cv::Mat img = cv::imread(g_watershed_segment_image);
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
	std::ofstream human_3Dpoints(g_human_3Dpoints_asc);
	std::ofstream color_human_cpt(g_color_human_3Dpoints_cpt);

	// output human point cloud above the ankle segmented by bounding box
	for (long int i = 0; i < width * ANKLE_HEIGHT; i++) {
		if (g_ptotalPoints[(g_CaptureNum)* total_pixels + i].Z > -X_back_human && g_ptotalPoints[(g_CaptureNum)* total_pixels + i].Z != 0 && g_ptotalPoints[(g_CaptureNum)* total_pixels + i].Z < -X_front_human && g_ptotalPoints[(g_CaptureNum)* total_pixels + i].X < Y_left_human && g_ptotalPoints[(g_CaptureNum)* total_pixels + i].X > Y_right_human && g_ptotalPoints[(g_CaptureNum)* total_pixels + i].Y > Z_human) {
			human_3Dpoints << -g_ptotalPoints[g_CaptureNum * total_pixels + i].Z << " "
				<< g_ptotalPoints[g_CaptureNum * total_pixels + i].X << " "
				<< g_ptotalPoints[g_CaptureNum * total_pixels + i].Y << " " << endl;
			color_human_cpt << -g_ptotalPoints[g_CaptureNum * total_pixels + i].Z << " "
				<< g_ptotalPoints[g_CaptureNum * total_pixels + i].X << " "
				<< g_ptotalPoints[g_CaptureNum * total_pixels + i].Y << " "
				<< (unsigned short)(g_pTotalColor[4 * g_CaptureNum * total_pixels + 4 * i]) << " "
				<< (unsigned short)(g_pTotalColor[4 * g_CaptureNum * total_pixels + 4 * i + 1]) << " "
				<< (unsigned short)(g_pTotalColor[4 * g_CaptureNum * total_pixels + 4 * i + 2]) << " " << endl;
		}
	}

	// output human point cloud below the ankle segmented by Absdiff  
	for (long int i = (width * ANKLE_HEIGHT); i < total_pixels; i++) {
		if (idx[i] == 1) {
			if (g_ptotalPoints[(g_CaptureNum)* total_pixels + i].Z > -X_back_human && g_ptotalPoints[(g_CaptureNum)* total_pixels + i].Z != 0 && g_ptotalPoints[(g_CaptureNum)* total_pixels + i].Z < -X_front_human && g_ptotalPoints[(g_CaptureNum)* total_pixels + i].X < Y_left_human && g_ptotalPoints[(g_CaptureNum)* total_pixels + i].X > Y_right_human && g_ptotalPoints[(g_CaptureNum)* total_pixels + i].Y > Z_human) {
				human_3Dpoints << -g_ptotalPoints[g_CaptureNum * total_pixels + i].Z << " "
					<< g_ptotalPoints[g_CaptureNum * total_pixels + i].X << " "
					<< g_ptotalPoints[g_CaptureNum * total_pixels + i].Y << " " << endl;
				color_human_cpt << -g_ptotalPoints[g_CaptureNum * total_pixels + i].Z << " "
					<< g_ptotalPoints[g_CaptureNum * total_pixels + i].X << " "
					<< g_ptotalPoints[g_CaptureNum * total_pixels + i].Y << " "
					<< (unsigned short)(g_pTotalColor[4 * g_CaptureNum * total_pixels + 4 * i]) << " "
					<< (unsigned short)(g_pTotalColor[4 * g_CaptureNum * total_pixels + 4 * i + 1]) << " "
					<< (unsigned short)(g_pTotalColor[4 * g_CaptureNum * total_pixels + 4 * i + 2]) << " " << endl;
			}
		}
	}

	// release the memory of array
	delete[] idx;
	// close the asc file
	human_3Dpoints.close();
	color_human_cpt.close();
	// release the memory of opencv matrix
	img.release();
}


// Ray:global variable
#include "Plane.h"

#define ROW_SMALL (600)
#define ROW_BIG (1000)
#define COL_SMALL (700)
#define COL_BIG (1200)

cv::Mat Imageresult;
cv::Mat binaryimage;
cv::Mat Image_de;

void onTrackbar(int position);
void filter()
{
	cv::Mat img = cv::imread("mask.bmp");
	// An one dimensional array to index whether the grayscale of each pixel is white
	short* idx;
	idx = new short[g_uColorPointNum];
	memset(idx, 0, g_uColorPointNum*sizeof(short));
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
	std::ofstream floor_asc("floor_point.asc");
	cout << "Starting filtering the floor points!" << endl;
	for (long int i = 0; i < g_uColorPointNum; i++) {
		if (idx[i] == 1) {		
			if (g_pboardPoints[i].Z > X_FRONT && g_pboardPoints[i].Z != 0 && g_pboardPoints[i].Z < X_BACK && g_pboardPoints[i].X < Y_LEFT && g_pboardPoints[i].X > Y_RIGHT && g_pboardPoints[i].Y > Z_HEIGHT) {
				floor_asc << -g_pboardPoints[i].Z << " " << g_pboardPoints[i].X << " "  << g_pboardPoints[i].Y << " " << endl;
			}
		}
	}
	cout << "Finishing filtering the floor points!" << endl;
	// release the memory of array
	delete[] idx;

	// close the asc file

	floor_asc.close();
	img.release();
}
void Mask_center();
void Center_filter();
//void cal_center(Point &center);
void PlaneFit(Plane &floor);
void bottle_filter();
void watershed(cv::Mat &input);
void cvFillHoles(cv::Mat &input);
void test(cv::Mat &input, int k);
void corner_filter(Point &center_point)
{
	// Size of input picture, the watershed segmented picture
	const int width = 1920;
	const int height = 1080;
	const long int total_pixels = width * height;

	cv::Mat img1 = cv::imread("four_point_mask34.bmp");
	cv::Mat img2 = cv::imread("four_point_mask12.bmp");
	cv::Mat img3 = cv::imread("four_point_mask23.bmp");
	cv::Mat img4 = cv::imread("four_point_mask14.bmp");
	// An one dimensional array to index whether the grayscale of each pixel is white
	short *idx1, *idx2, *idx3, *idx4;
	idx1 = new short[total_pixels];
	idx2 = new short[total_pixels];
	idx3 = new short[total_pixels];
	idx4 = new short[total_pixels];
	memset(idx1, 0, total_pixels*sizeof(short));
	memset(idx2, 0, total_pixels*sizeof(short));
	memset(idx3, 0, total_pixels*sizeof(short));
	memset(idx4, 0, total_pixels*sizeof(short));
	// load grayscale of each pixel in the mask
	for (int j = 0; j < img1.rows; j++) {
		for (int i = 0; i < img1.cols; i++) {
			cv::Scalar intensity = img1.at<cv::Vec3b>(j, i);
			// when the mask is white
			if (intensity.val[0] == 255) {
				idx1[i + j * img1.cols] = 1;
			}
		}
	}
	for (int j = 0; j < img2.rows; j++) {
		for (int i = 0; i < img2.cols; i++) {
			cv::Scalar intensity = img2.at<cv::Vec3b>(j, i);
			// when the mask is white
			if (intensity.val[0] == 255) {
				idx2[i + j * img2.cols] = 1;
			}
		}
	}
	for (int j = 0; j < img3.rows; j++) {
		for (int i = 0; i < img3.cols; i++) {
			cv::Scalar intensity = img3.at<cv::Vec3b>(j, i);
			// when the mask is white
			if (intensity.val[0] == 255) {
				idx3[i + j * img3.cols] = 1;
			}
		}
	}
	for (int j = 0; j < img4.rows; j++) {
		for (int i = 0; i < img4.cols; i++) {
			cv::Scalar intensity = img4.at<cv::Vec3b>(j, i);
			// when the mask is white
			if (intensity.val[0] == 255) {
				idx4[i + j * img4.cols] = 1;
			}
		}
	}
	// initialize the output asc file
	std::ofstream corner_asc("corner_point.asc");

	vector<double> x_corner;
	vector<double> y_corner;
	vector<double> z_corner;
	double x_sum1 = 0; double x_sum2 = 0; double x_sum3 = 0; double x_sum4 = 0;
	double y_sum1 = 0; double y_sum2 = 0; double y_sum3 = 0; double y_sum4 = 0;
	double z_sum1 = 0; double z_sum2 = 0; double z_sum3 = 0; double z_sum4 = 0;
	for (long int i = 0; i < total_pixels; i++) {
		if (idx1[i] == 1 && (g_pboardPoints[i].Z > X_FRONT && g_pboardPoints[i].Z != 0 && g_pboardPoints[i].Z < X_BACK && g_pboardPoints[i].X < Y_LEFT && g_pboardPoints[i].X > Y_RIGHT && g_pboardPoints[i].Y > Z_HEIGHT)) {
			x_corner.push_back(g_pboardPoints[i].X);
			y_corner.push_back(g_pboardPoints[i].Y);
			z_corner.push_back(g_pboardPoints[i].Z);
		}
	}
	for (int i = 0; i < x_corner.size(); i++){
		x_sum1 += x_corner[i];
		y_sum1 += y_corner[i];
		z_sum1 += z_corner[i];
	}
	x_sum1 /= x_corner.size();
	y_sum1 /= y_corner.size();
	z_sum1 /= z_corner.size();

	x_corner.clear();
	y_corner.clear();
	z_corner.clear();
	for (long int i = 0; i < total_pixels; i++) {
		if (idx2[i] == 1&& (g_pboardPoints[i].Z > X_FRONT && g_pboardPoints[i].Z != 0 && g_pboardPoints[i].Z < X_BACK && g_pboardPoints[i].X < Y_LEFT && g_pboardPoints[i].X > Y_RIGHT && g_pboardPoints[i].Y > Z_HEIGHT)) {
			x_corner.push_back(g_pboardPoints[i].X);
			y_corner.push_back(g_pboardPoints[i].Y);
			z_corner.push_back(g_pboardPoints[i].Z);
		}
	}
	for (int i = 0; i < x_corner.size(); i++){
		x_sum2 += x_corner[i];
		y_sum2 += y_corner[i];
		z_sum2 += z_corner[i];
	}
	x_sum2 /= x_corner.size();
	y_sum2 /= y_corner.size();
	z_sum2 /= z_corner.size();

	x_corner.clear();
	y_corner.clear();
	z_corner.clear();
	for (long int i = 0; i < total_pixels; i++) {
		if (idx3[i] == 1&& (g_pboardPoints[i].Z > X_FRONT && g_pboardPoints[i].Z != 0 && g_pboardPoints[i].Z < X_BACK && g_pboardPoints[i].X < Y_LEFT && g_pboardPoints[i].X > Y_RIGHT && g_pboardPoints[i].Y > Z_HEIGHT)) {
			x_corner.push_back(g_pboardPoints[i].X);
			y_corner.push_back(g_pboardPoints[i].Y);
			z_corner.push_back(g_pboardPoints[i].Z);
		}
	}
	for (int i = 0; i < x_corner.size(); i++) {
		x_sum3 += x_corner[i];
		y_sum3 += y_corner[i];
		z_sum3 += z_corner[i];
	}
	x_sum3 /= x_corner.size();
	y_sum3 /= y_corner.size();
	z_sum3 /= z_corner.size();

	x_corner.clear();
	y_corner.clear();
	z_corner.clear();
	for (long int i = 0; i < total_pixels; i++) {
		if (idx4[i] == 1&& (g_pboardPoints[i].Z > X_FRONT && g_pboardPoints[i].Z != 0 && g_pboardPoints[i].Z < X_BACK && g_pboardPoints[i].X < Y_LEFT && g_pboardPoints[i].X > Y_RIGHT && g_pboardPoints[i].Y > Z_HEIGHT)) {
			x_corner.push_back(g_pboardPoints[i].X);
			y_corner.push_back(g_pboardPoints[i].Y);
			z_corner.push_back(g_pboardPoints[i].Z);
		}
	}
	for (int i = 0; i < x_corner.size(); i++){
		x_sum4 += x_corner[i];
		y_sum4 += y_corner[i];
		z_sum4 += z_corner[i];
	}
	x_sum4 /= x_corner.size();
	y_sum4 /= y_corner.size();
	z_sum4 /= z_corner.size();

	corner_asc << -z_sum1 << " " << x_sum1 << " " << y_sum1 << endl;
	corner_asc << -z_sum2 << " " << x_sum2 << " " << y_sum2 << endl;
	corner_asc << -z_sum3 << " " << x_sum3 << " " << y_sum3 << endl;
	corner_asc << -z_sum4 << " " << x_sum4 << " " << y_sum4 << endl;
	
	center_point.x = -(z_sum1 + z_sum2 + z_sum3 + z_sum4) / 4;
	center_point.y = (x_sum1 + x_sum2 + x_sum3 + x_sum4) / 4;
	center_point.z = (y_sum1 + y_sum2 + y_sum3 + y_sum4) / 4;
	
	// release the memory of array
	delete[] idx1;
	delete[] idx2;
	delete[] idx3;
	delete[] idx4;

	// close the asc file
	corner_asc.close();
	img1.release();
	img2.release();
	img3.release();
	img4.release();
}
void Find_contour(cv::Mat &input, int row_small, int row_big, int col_small, int col_big, int &k);

// Debug:output the velocity of joints
//ofstream current_average_velocityTXT("current_average_velocity.txt");
//ofstream average_velocityTXT("average_velocity.txt");

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
	ON_BN_CLICKED(IDC_BUTTON4, &CKinectcaptureDlg::OnBnClickedButton_Release)
	ON_BN_CLICKED(IDC_BUTTON5, &CKinectcaptureDlg::OnBnClickedButton_Coordinate)
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

	// Use OpenCV to get the image from Kinect
	cv::Mat	background_color(g_iColorHeight, g_iColorWidth, CV_8UC4);
	cv::Mat background_gray(g_iColorHeight, g_iColorWidth, CV_8UC4);
	char* window_name = "Background Image";
	cv::namedWindow(window_name, 0);									
	if (pCFrame->CopyConvertedFrameDataToArray(g_uColorBufferSize, background_color.data, ColorImageFormat_Bgra) == S_OK)	{	
		cv::cvtColor(background_color, background_gray, CV_RGB2GRAY);	
		cv::imshow(window_name, background_color);						
		cv::imwrite("background_color.bmp", background_color);	
		cv::imwrite("background_gray.bmp", background_gray);
		cv::waitKey();
	}
	cv::destroyWindow(window_name);
	pCFrame->Release();
	pCFrame = nullptr;
	cout << "Finish outputting the image of the background." << endl;
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
	g_CaptureNum = 0;
	while (g_CaptureNum <= 7) {
		get_joint_position();
		capture_human_data();
		if (g_display.data) {
			cv::namedWindow("Current condition", 0);
			cv::imshow("Current condition", g_display);
			// 4c. check keyboard input
			if (cv::waitKey(30) == VK_ESCAPE) {
				break;
			}
		}
	}
	end_joint_driven();
}


void CKinectcaptureDlg::OnBnClickedButton_Output()
{
	for (g_CaptureNum = 0; g_CaptureNum < 8; g_CaptureNum++) {
		sprintf(g_watershed_segment_image, "watershed_segment%d.bmp", g_CaptureNum);
		sprintf(g_human_3Dpoints_asc, "human_3Dpoints%d.asc", g_CaptureNum);
		sprintf(g_color_human_3Dpoints_cpt, "color_human_3Dpoints%d.txt", g_CaptureNum);
		human_mask();
		cout << g_human_3Dpoints_asc << " has been finished!" << endl;
	}
	delete[] g_ptotalPoints;
	cout << "g_ptotalPoints has been deleted!" << endl;
	delete[] g_pTotalColor;
	cout << "g_pTotalColor has been deleted!" << endl;
}


void CKinectcaptureDlg::OnBnClickedButton_Release()
{
	// Sensor related code
	default_sensor();
	// Color related code
	color_source();
	// Depth related code
	depth_source();
	// coordinate mapper code
	map_coordinate();

	IColorFrame* pCFrame = nullptr;
	while (g_pColorFrameReader->AcquireLatestFrame(&pCFrame) != S_OK) {
		if (g_pColorFrameReader->AcquireLatestFrame(&pCFrame) == S_OK)
			break;
	}

	// Use OpenCV to get the image from Kinect
	cv::Mat	background_board_color(g_iColorHeight, g_iColorWidth, CV_8UC4);
	cv::Mat background_board_gray(g_iColorHeight, g_iColorWidth, CV_8UC4);
	if (pCFrame->CopyConvertedFrameDataToArray(g_uColorBufferSize, background_board_color.data, ColorImageFormat_Bgra) == S_OK)	{
		cv::imwrite("background_board_color.bmp", background_board_color);
		cv::cvtColor(background_board_color, background_board_gray, CV_RGB2GRAY);
		cv::imwrite("background_board_gray.bmp", background_board_gray);
	}
	pCFrame->Release();
	pCFrame = nullptr;
	cout << "Finish outputting the image of the board." << endl;

	// Read depth data
	IDepthFrame* pDFrame = nullptr;
	if (g_pDepthFrameReader->AcquireLatestFrame(&pDFrame) == S_OK) {
		pDFrame->CopyFrameDataToArray(g_uDepthPointNum, g_pDepthBuffer);
		pDFrame->Release();
		pDFrame = nullptr;
		// map to camera space
		g_pCoordinateMapper->MapColorFrameToCameraSpace(g_uDepthPointNum, g_pDepthBuffer, g_uColorPointNum, g_pCSPoints);
		for (int y = 0; y < g_iColorHeight; ++y) {
			for (int x = 0; x < g_iColorWidth; ++x) {
				int idx = x + y * g_iColorWidth;
				g_pboardPoints[idx].X = g_pCSPoints[idx].X;
				g_pboardPoints[idx].Y = g_pCSPoints[idx].Y;
				g_pboardPoints[idx].Z = g_pCSPoints[idx].Z;
			}
		}
	}

	cv::Mat Image = cv::imread("background_board_gray.bmp", 0);		// 背景+板子
	cv::Mat Imagebg = cv::imread("background_gray.bmp", 0);			// 背景
	cv::subtract(Imagebg, Image, Imageresult);				// 相減
	// Debug
	//cv::imwrite("subtract.bmp", Imageresult);
	threshold(Imageresult, binaryimage, 20, 255, CV_THRESH_BINARY);
	// Debug
	//cv::imwrite("result.bmp", binaryimage);
	watershed(binaryimage);
	cv::dilate(binaryimage, Image_de, cv::Mat(), cv::Point(-1, -1), 6, 1, 1);
	cv::erode(Image_de, Image_de, cv::Mat(), cv::Point(-1, -1), 5, 1, 1);
	// Debug
	//imwrite("d&e.bmp", Image_de);
	cvFillHoles(Image_de);
	// Debug
	//imwrite("flood.bmp", Image_de);
	cv::Mat blurImg;
	medianBlur(Image_de, blurImg, 3);

	cv::Mat afterdil;
	cv::dilate(blurImg, afterdil, cv::Mat(), cv::Point(-1, -1), 8, 1, 1);
	cv::Mat Image_bound;
	cv::subtract(afterdil, blurImg, Image_bound);
	cv::imwrite("mask.bmp", Image_bound);

	int k = 0;
	
	Find_contour(blurImg, ROW_SMALL, ROW_BIG, COL_SMALL, COL_BIG, k);
	// Debug
	//cv::imshow("test", blurImg);
	//cv::waitKey(0);
	test(blurImg, k);
	cv::destroyAllWindows();
	Image.release();
	Imagebg.release();
	Imageresult.release();
	binaryimage.release();	
	cout << "Finish outputting the mask of the board." << endl;
}


void CKinectcaptureDlg::OnBnClickedButton_Coordinate()
{
	filter();
	Plane floor;
	PlaneFit(floor);
	Point center_point;
	corner_filter(center_point);

	Point center_proj;
	center_proj = floor.Projection(center_point);
	cout << center_proj.x << " " << center_proj.y << " " << center_proj.z << endl;
	std::ofstream center_only_one("center_proj.asc");
	center_only_one << center_proj.x << " " << center_proj.y << " " << center_proj.z << " " << endl;
	center_only_one.close();

	////calculate bounding box for human
	X_back_human = center_proj.x + 0.7;
	X_front_human = center_proj.x-0.7;
	Y_left_human = center_proj.y+0.7;
	Y_right_human = center_proj.y - 0.7;
	Z_human = center_proj.z-0.1;


	/////////


	cout << "Finishing output principal axis and the ground plane!" << endl;
}
