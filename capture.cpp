// capture.cpp : 合併抓點與濾點程式

// Standard Library
#include <iostream>
#include <fstream>

// Kinect for Windows SDK Header
#include <Kinect.h>

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

#define HUMANCOLORIMAGE
#define BACKGROUND
//#define CAPTURE
 
using namespace std;

// global objects
IKinectSensor*		pSensor = nullptr;
IColorFrameReader*	pColorFrameReader = nullptr;
IDepthFrameReader*	pDepthFrameReader = nullptr;
ICoordinateMapper*	pCoordinateMapper = nullptr;

int		iColorWidth = 0, iColorHeight = 0;

UINT	uDepthPointNum = 0;
UINT	uColorPointNum = 0;
UINT	uColorBufferSize = 0;

UINT16*	pDepthBuffer = nullptr;
BYTE*	pColorBuffer = nullptr;
CameraSpacePoint* pCSPoints = nullptr;

char* window_name = "Image output";
char* original_image = "human_color.bmp";
char* absdiff_binary_image = "absdiff_binary.bmp";
//char* Background_Remove_Color_image_file = "Background_Remove_Color.bmp";
char* watershed_segment_image = "watershed_segment.bmp";

char* background_image = "background_color.bmp";

cv::Mat background_color = cv::imread(background_image);				// get the background image frame
cv::Mat background_color_resize;

cv::Mat raw_color;	// the image that is constantly being updated
cv::Mat absdiff_color;
cv::Mat absdiff_gray;
cv::Mat absdiff_binary;
cv::Mat markers;

void idle();
void capture_point();
void ExitFunction();
void release_mat_memory();
void absdiff();
void watershed();
void human_mask();

int main()
{
	// 1. Sensor related code
	cout << "Try to get default sensor" << endl;
	{
		if (GetDefaultKinectSensor(&pSensor) != S_OK) {
			cerr << "Get Sensor failed" << endl;
			return -1;
		}

		cout << "Try to open sensor" << endl;
		if (pSensor->Open() != S_OK)	 {
			cerr << "Can't open sensor" << endl;
			return -1;
		}
	}

	// 2. Color related code
	cout << "Try to get color source" << endl;
	{
		// Get frame source
		IColorFrameSource* pFrameSource = nullptr;
		if (pSensor->get_ColorFrameSource(&pFrameSource) != S_OK) {
			cerr << "Can't get color frame source" << endl;
			return -1;
		}

		// Get frame description
		cout << "get color frame description" << endl;
		IFrameDescription* pFrameDescription = nullptr;
		if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)	{
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
		if (pFrameSource->OpenReader(&pColorFrameReader) != S_OK) {
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
		if (pSensor->get_DepthFrameSource(&pFrameSource) != S_OK) {
			cerr << "Can't get depth frame source" << endl;
			return -1;
		}

		// Get frame description
		cout << "get depth frame description" << endl;
		IFrameDescription* pFrameDescription = nullptr;
		if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK) {
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
		if (pFrameSource->OpenReader(&pDepthFrameReader) != S_OK) {
			cerr << "Can't get depth frame reader" << endl;
			return -1;
		}

		// release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;
	}

	// 4. Coordinate Mapper
	if (pSensor->get_CoordinateMapper(&pCoordinateMapper) != S_OK) {
		cerr << "get_CoordinateMapper failed" << endl;
		return -1;
	}

	while (1) {
		idle();
		if ((int)pColorBuffer[0] != 0) {		
			capture_point();
		}
	}
	ExitFunction();

	return 0;
}

void capture_point()    // 抓人體
{
	int count_number = 0;
	
	#ifdef HUMANCOLORIMAGE
	// Read color data
	IColorFrame* pCFrame = nullptr;
	if (pColorFrameReader->AcquireLatestFrame(&pCFrame) == S_OK) {
		// 使用openCV擷取Kinect畫面
		cv::Mat	mImg(iColorHeight, iColorWidth, CV_8UC4);
		cv::Mat gray_image(iColorHeight, iColorWidth, CV_8UC4);
		cv::namedWindow("Color Image", 0);						// 創造彩色圖片顯示視窗
		if (pCFrame->CopyConvertedFrameDataToArray(uColorBufferSize, mImg.data, ColorImageFormat_Bgra) == S_OK)	{
			if ((int)pColorBuffer[0] != 0) {
				raw_color = mImg.clone();
				cv::cvtColor(mImg, gray_image, CV_RGB2GRAY);		// 彩色轉灰階
				cv::imshow("Color Image", raw_color);				// 顯示彩色畫面	

				# ifdef BACKGROUND
				cv::imwrite("background_color.bmp", mImg);		// 輸出背景的彩色畫面
				# endif

				cout << "Press any key on the image window to continue!" << endl;
				cv::waitKey();
			}
		}

		# ifdef CAPTURE
		absdiff();
		watershed();
		human_mask();
		# endif

		cv::destroyWindow("Color Image");
		pCFrame->Release();
		pCFrame = nullptr;
	}
	#endif

	for (int y = 0; y < iColorHeight; ++y) {
		for (int x = 0; x < iColorWidth; ++x) {
			int idx = x + y * iColorWidth;
			CameraSpacePoint& rPt = pCSPoints[idx];

			if (rPt.Z <= 0)
				rPt.X = rPt.Y = rPt.Z = 0;

			if (rPt.Z < 2.35 && rPt.Z != 0 && rPt.Z>1.1 && rPt.X<0.69 && rPt.X>-0.69 && rPt.Y>-0.88) {
				//raw_point << rPt.X * 1000 << " " << rPt.Z * 1000 << " " << rPt.Y * 1000 << endl;

				if (rPt.Y > -0.74 && idx % 75 == 0) 	{
					//for_icp << rPt.X * 1000 << " " << rPt.Z * 1000 << " " << rPt.Y * 1000 << endl;
				}
			}
			count_number++;
		}
	}
	
	//for_icp.close();
	//raw_point.close();

	//if (count_number>0)
		//system("pause");
}

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

void idle()
{
	// Read color data
	IColorFrame* pCFrame = nullptr;
	if (pColorFrameReader->AcquireLatestFrame(&pCFrame) == S_OK)
	{
		pCFrame->CopyConvertedFrameDataToArray(uColorBufferSize, pColorBuffer, ColorImageFormat_Rgba);

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
		pCoordinateMapper->MapColorFrameToCameraSpace(uDepthPointNum, pDepthBuffer, uColorPointNum, pCSPoints);
	}
}

void release_mat_memory()
{
	background_color.release();
	background_color_resize.release();
	raw_color.release();
	absdiff_color.release();
	absdiff_gray.release();
	absdiff_binary.release();
	markers.release();
}

void absdiff()
{
	cv::resize(background_color, background_color_resize, background_color_resize.size(), 0, 0, cv::INTER_LINEAR);
	// To test...
	cv::absdiff(raw_color, background_color, absdiff_color);
	cv::cvtColor(absdiff_color, absdiff_gray, CV_RGB2GRAY);
	cv::threshold(absdiff_gray, absdiff_binary, 40, 255, cv::THRESH_BINARY);
	// Save the difference image
	//cv::imwrite("absdiff_color.bmp", absdiff_color);
	//cv::imwrite("absdiff_gray.bmp", absdiff_gray);
	//cv::imwrite("absdiff_binary_image", absdiff_binary);
}

void watershed()
{
	cv::Mat image;
	absdiff_color.copyTo(image);
	cv::Mat gray;
	cv::cvtColor(image, gray, CV_BGR2GRAY);
	cv::Mat binary;
	cv::threshold(gray, binary, 40, 255, cv::THRESH_BINARY);

	// Eliminate noise and smaller objects
	cv::Mat fg;
	cv::erode(binary, fg, cv::Mat(), cv::Point(-1, -1), 2);

	// Identify image pixels without objects
	cv::Mat bg;
	cv::dilate(binary, bg, cv::Mat(), cv::Point(-1, -1), 3);
	cv::threshold(bg, bg, 1, 128, cv::THRESH_BINARY_INV);

	// Create markers image
	//cv::Mat markers(binary.size(), CV_8U, cv::Scalar(0));
	markers = fg + bg;

	markers.convertTo(markers, CV_32S);
	cv::watershed(image, markers);

	markers.convertTo(markers, CV_8U);
	cv::threshold(markers, markers, 128, 255, cv::THRESH_BINARY);
	//cv::imwrite(watershed_segment_image, markers);

	// Release memory of opencv matrix
	image.release();
	gray.release();
	binary.release();
	fg.release();
	bg.release();
	//markers.release();
}

void human_mask()
{
	// Size of input picture, the watershed segmented picture
	const int width = 1920;
	const int height = 1080;
	const long int total_pixels = width * height;

	cv::Mat img;
	markers.copyTo(img);
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

	// initialize the input asc file
	//std::fstream raw_point;
	// Load the raw point cloud file
	//raw_point.open("raw_point.asc", std::ios::in);
	// initialize the input colored asc file
	//std::fstream point_cloud_with_rgb;
	// 讀入原始3維點的彩色雲點asc檔
	//point_cloud_with_rgb.open("point_cloud_with_rgb.asc", std::ios::in);

	// An array to load each point (x, y, z) in the input asc file
	double* points_array;
	points_array = new double[3 * total_pixels];
	memset(points_array, 0, 3 * total_pixels*sizeof(double));

	double* X;
	X = new double[total_pixels];
	memset(X, 0, total_pixels*sizeof(double));
	int count_x = 0;

	double* Y;
	Y = new double[total_pixels];
	memset(Y, 0, total_pixels*sizeof(double));
	int count_y = 0;

	double* Z;
	Z = new double[total_pixels];
	memset(Z, 0, total_pixels*sizeof(double));
	int count_z = 0;

	int pixel_height_of_ankle = 750;
	// Save the (x, y, z) points into three arrays
	for (long int i = 0; i < 3 * total_pixels; i++) {
		//raw_point >> points_array[i];
		if (i % 3 == 0) {
			X[count_x] = pCSPoints[i].X;
			count_x++;
		}
		if (i % 3 == 1) {
			Y[count_y] = pCSPoints[i].Y;
			count_y++;
		}
		if (i % 3 == 2) {
			Z[count_z] = pCSPoints[i].Z;
			count_z++;
		}
	}

	// initialize the output asc file
	std::ofstream human_asc("human_point.asc");
	//std::ofstream colored_human_asc("colored_human_point.asc");

	// output human point cloud above the ankle segmented by bounding box
	for (long int i = 0; i < width * pixel_height_of_ankle; i++) {
		if (Z[i] < 3030 && Z[i] != 0 && Z[i] > 2100 && X[i] < 800 && X[i] > -690 && Y[i] > -950) {
			human_asc << X[i] << " " << Y[i] << " " << Z[i] << " " << endl;
		}
		//if (Z[i] < 2800 && Z[i] != 0 && Z[i] > 2350 && X[i] < 800 && X[i] > -690 && Y[i] > -950) {
		//	colored_human_asc << X[i] << " " << Y[i] << " " << Z[i] << " " << endl;
		//}
	}

	// output human point cloud below the ankle segmented by Absdiff  
	for (long int i = width * pixel_height_of_ankle; i < total_pixels; i++) {
		if (idx[i] == 1) {
			if (Z[i] < 2800 && Z[i] != 0 && Z[i] > 2350 && X[i] < 500 && X[i] > -500 && Y[i] > -950) {
				human_asc << X[i] << " " << Y[i] << " " << Z[i] << " " << endl;
			}
		}
		//if (idx[i]) {
		//	if (Z[i] < 2800 && Z[i] != 0 && Z[i] > 2350 && X[i] < 800 && X[i] > -690 && Y[i] > -950) {
		//		colored_human_asc << X[i] << " " << Y[i] << " " << Z[i] << " " << endl;
		//	}
		//}
	}

	// release the memory of array
	delete[] idx;
	delete[] points_array;
	delete[] X;
	delete[] Y;
	delete[] Z;
	// close the asc file
	//raw_point.close();
	human_asc.close();
	//colored_human_asc.close();
	// release the memory of opencv matrix
	//markers.release();
	//cv::waitKey(0);
}