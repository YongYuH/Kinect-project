// Created by Heresy @ 2015/02/25
// Blog Page: https://kheresy.wordpress.com/2015/04/10/k4w-v2-part-7a-draw-user-skeleton/
// This sample is used to read information of body joint nad draw with OpenCV.

// Standard Library
#include <iostream>
#include <fstream>
#include <math.h>
#include <stdio.h>

// OpenCV Header
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// Kinect for Windows SDK Header
#include <Kinect.h>

// Play the sound effect
#include <Mmsystem.h>
#pragma comment(lib,"Winmm.lib")

using namespace std;

void DrawLine(cv::Mat& rImg, const Joint& rJ1, const Joint& rJ2, ICoordinateMapper* pCMapper ) {
	if (rJ1.TrackingState == TrackingState_NotTracked || rJ2.TrackingState == TrackingState_NotTracked)
		return;

	ColorSpacePoint ptJ1, ptJ2;
	pCMapper->MapCameraPointToColorSpace(rJ1.Position, &ptJ1);
	pCMapper->MapCameraPointToColorSpace(rJ2.Position, &ptJ2);

	cv::line(rImg, cv::Point(ptJ1.X, ptJ1.Y), cv::Point(ptJ2.X, ptJ2.Y), cv::Vec3b(0, 0, 255), 5);
}

int main(int argc, char** argv) {
	// 1a. Get default Sensor
	std::cout << "Try to get default sensor" << std::endl;
	IKinectSensor* pSensor = nullptr;
	if (GetDefaultKinectSensor(&pSensor) != S_OK) {
		cerr << "Get Sensor failed" << std::endl;
		return -1;
	}

	// 1b. Open sensor
	std::cout << "Try to open sensor" << std::endl;
	if (pSensor->Open() != S_OK) {
		cerr << "Can't open sensor" << std::endl;
		return -1;
	}

	// 2. Color Related code
	IColorFrameReader* pColorFrameReader = nullptr;
	cv::Mat	mColorImg;
	UINT uBufferSize = 0;
	{
		// 2a. Get color frame source
		std::cout << "Try to get color source" << std::endl;
		IColorFrameSource* pFrameSource = nullptr;
		if (pSensor->get_ColorFrameSource(&pFrameSource) != S_OK) {
			cerr << "Can't get color frame source" << std::endl;
			return -1;
		}

		// 2b. Get frame description
		std::cout << "get color frame description" << std::endl;
		int		iWidth = 0;
		int		iHeight = 0;
		IFrameDescription* pFrameDescription = nullptr;
		if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)	{
			pFrameDescription->get_Width(&iWidth);
			pFrameDescription->get_Height(&iHeight);
		}
		pFrameDescription->Release();
		pFrameDescription = nullptr;

		// 2c. get frame reader
		std::cout << "Try to get color frame reader" << std::endl;
		if (pFrameSource->OpenReader(&pColorFrameReader) != S_OK) {
			cerr << "Can't get color frame reader" << std::endl;
			return -1;
		}

		// 2d. release Frame source
		std::cout << "Release frame source" << std::endl;
		pFrameSource->Release();
		pFrameSource = nullptr;

		// Prepare OpenCV data
		mColorImg = cv::Mat(iHeight, iWidth, CV_8UC4);
		uBufferSize = iHeight * iWidth * 4 * sizeof(BYTE);
	}

	// 3. Body related code
	IBodyFrameReader* pBodyFrameReader = nullptr;
	IBody** aBodyData = nullptr;
	INT32 iBodyCount = 0;
	{
		// 3a. Get frame source
		std::cout << "Try to get body source" << std::endl;
		IBodyFrameSource* pFrameSource = nullptr;
		if (pSensor->get_BodyFrameSource(&pFrameSource) != S_OK) {
			cerr << "Can't get body frame source" << std::endl;
			return -1;
		}

		// 3b. Get the number of body
		if (pFrameSource->get_BodyCount(&iBodyCount) != S_OK) {
			cerr << "Can't get body count" << std::endl;
			return -1;
		}
		std::cout << " > Can trace " << iBodyCount << " bodies" << std::endl;
		aBodyData = new IBody*[iBodyCount];
		for (int i = 0; i < iBodyCount; ++i)
			aBodyData[i] = nullptr;

		// 3c. get frame reader
		std::cout << "Try to get body frame reader" << std::endl;
		if (pFrameSource->OpenReader(&pBodyFrameReader) != S_OK) {
			cerr << "Can't get body frame reader" << std::endl;
			return -1;
		}

		// 3d. release Frame source
		std::cout << "Release frame source" << std::endl;
		pFrameSource->Release();
		pFrameSource = nullptr;
	}

	// 4. get CoordinateMapper
	ICoordinateMapper* pCoordinateMapper = nullptr;
	if (pSensor->get_CoordinateMapper(&pCoordinateMapper) != S_OK) {
		std::cout << "Can't get coordinate mapper" << std::endl;
		return -1;
	}

	// Enter main loop
	cv::namedWindow("Body Image");

	// Debug:output the velocity of joints
	ofstream current_average_velocityTXT("current_average_velocity.txt");
	ofstream average_velocityTXT("average_velocity.txt");
	int frame_count = 0;
	int frame_count_for_standby = 0;
	float positionX0[25] = {0};
	float positionX1[25] = {0};
	float positionY0[25] = { 0 };
	float positionY1[25] = { 0 };
	float positionZ0[25] = { 0 };
	float positionZ1[25] = { 0 };

	float velocityX[25] = { 0 };
	float velocityY[25] = { 0 };
	float velocityZ[25] = { 0 };
	float current_velocity[25] = { 0 };
	float velocityee[8] = { 0 };
	float current_total_velocity = 0;
	float current_average_velocity = 0;
	float total_velocity = 0;
	float average_velocity = 0;

	while (true)
	{
		// 4a. Get last frame
		IColorFrame* pColorFrame = nullptr;
		if (pColorFrameReader->AcquireLatestFrame(&pColorFrame) == S_OK)	 {
			// 4c. Copy to OpenCV image
			if (pColorFrame->CopyConvertedFrameDataToArray(uBufferSize, mColorImg.data, ColorImageFormat_Bgra) != S_OK)	{
				cerr << "Data copy error" << endl;
			}
			// 4e. release frame
			pColorFrame->Release();
		}
		cv::Mat mImg = mColorImg.clone();
		// 4b. Get body data
		IBodyFrame* pBodyFrame = nullptr;	
		if (pBodyFrameReader->AcquireLatestFrame(&pBodyFrame) == S_OK) {
			// 4b. get Body data
			if (pBodyFrame->GetAndRefreshBodyData(iBodyCount, aBodyData) == S_OK) {
				// 4c. for each body
				for (int i = 0; i < iBodyCount; ++i) {
					IBody* pBody = aBodyData[i];
					// check if is tracked
					BOOLEAN bTracked = false;
					if ((pBody->get_IsTracked(&bTracked) == S_OK) && bTracked) {
						// get joint position
						Joint aJoints[JointType::JointType_Count];
						if (pBody->GetJoints(JointType::JointType_Count, aJoints) == S_OK) {
							DrawLine(mImg, aJoints[JointType_SpineBase], aJoints[JointType_SpineMid], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_SpineMid], aJoints[JointType_SpineShoulder], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_SpineShoulder], aJoints[JointType_Neck], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_Neck], aJoints[JointType_Head], pCoordinateMapper);

							DrawLine(mImg, aJoints[JointType_SpineShoulder], aJoints[JointType_ShoulderLeft], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_ShoulderLeft], aJoints[JointType_ElbowLeft], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_ElbowLeft], aJoints[JointType_WristLeft], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_WristLeft], aJoints[JointType_HandLeft], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_HandLeft], aJoints[JointType_HandTipLeft], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_HandLeft], aJoints[JointType_ThumbLeft], pCoordinateMapper);

							DrawLine(mImg, aJoints[JointType_SpineShoulder], aJoints[JointType_ShoulderRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_ShoulderRight], aJoints[JointType_ElbowRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_ElbowRight], aJoints[JointType_WristRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_WristRight], aJoints[JointType_HandRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_HandRight], aJoints[JointType_HandTipRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_HandRight], aJoints[JointType_ThumbRight], pCoordinateMapper);

							DrawLine(mImg, aJoints[JointType_SpineBase], aJoints[JointType_HipLeft], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_HipLeft], aJoints[JointType_KneeLeft], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_KneeLeft], aJoints[JointType_AnkleLeft], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_AnkleLeft], aJoints[JointType_FootLeft], pCoordinateMapper);

							DrawLine(mImg, aJoints[JointType_SpineBase], aJoints[JointType_HipRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_HipRight], aJoints[JointType_KneeRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_KneeRight], aJoints[JointType_AnkleRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_AnkleRight], aJoints[JointType_FootRight], pCoordinateMapper);
						}
						// Debug:print out the number of frame					
						std::cout << "frame " << ++frame_count << std::endl;
					
						for (int j = 1; j < 8; j++) {
							velocityee[j] = velocityee[j-1];
							total_velocity += velocityee[j];
						}
						average_velocity = total_velocity / 8.0;
						
						if (average_velocity <= 0.0015) {	
							// determine if the person is still 
							if (frame_count_for_standby == 0) {
								PlaySound(TEXT("Alarm02.wav"), NULL, SND_FILENAME);
								std::cout << "Start capturing points!" << std::endl;
							}
							// count the number of frame whose velocity is below the threshold
							frame_count_for_standby++;
							if (frame_count_for_standby >= 5) {
								frame_count_for_standby = 0;
							}
						} 
						// Debug:output the average velocity 
						average_velocityTXT << frame_count << " " << average_velocity << std::endl;
						total_velocity = 0;
						// Update the average velocity
						int available_joints = 0;
						for (int i = 0; i < 25; i++) {
							// X 
							positionX1[i] = positionX0[i];
							positionX0[i] = aJoints[i].Position.X;
							velocityX[i] = (positionX1[i] - positionX0[i]) * (positionX1[i] - positionX0[i]);
							// Y
							positionY1[i] = positionY0[i];
							positionY0[i] = aJoints[i].Position.Y;
							velocityY[i] = (positionY1[i] - positionY0[i]) * (positionY1[i] - positionY0[i]);
							// Z
							positionZ1[i] = positionZ0[i];
							positionZ0[i] = aJoints[i].Position.Z;
							velocityZ[i] = (positionZ1[i] - positionZ0[i]) * (positionZ1[i] - positionZ0[i]);
							current_velocity[i] = sqrtf(velocityX[i] + velocityY[i] + velocityZ[i]);
							// exclude the discrete velocity
							if (current_velocity[i] < 0.01) {
								current_total_velocity += current_velocity[i];
								available_joints++;
							}
						}
						// If no joint is available, save the velocity of last frame
						if (available_joints != 0) {
							current_average_velocity = current_total_velocity / available_joints;
						}
						velocityee[0] = current_average_velocity;
						// Debug:output the current average velocity 
						current_average_velocityTXT << frame_count << " " << current_average_velocity << std::endl;
											
						current_total_velocity = 0;					
					}
				}
			} else {
				cerr << "Can't read body data" << endl;
			}
			// 4e. release frame
			pBodyFrame->Release();
		}
		// show image
		cv::imshow("Body Image",mImg);
		// 4c. check keyboard input
		if (cv::waitKey(30) == VK_ESCAPE) {
			break;
		}
	}
	// 3. delete body data array
	delete[] aBodyData;
	// 3. release frame reader
	std::cout << "Release body frame reader" << std::endl;
	pBodyFrameReader->Release();
	pBodyFrameReader = nullptr;
	// 2. release color frame reader
	std::cout << "Release color frame reader" << std::endl;
	pColorFrameReader->Release();
	pColorFrameReader = nullptr;
	// 1c. Close Sensor
	std::cout << "close sensor" << std::endl;
	pSensor->Close();
	// 1d. Release Sensor
	std::cout << "Release sensor" << std::endl;
	pSensor->Release();
	pSensor = nullptr;

	return 0;
}
