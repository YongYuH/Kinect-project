// Created by Yong-Yu Huang 2015/10/22
// Use the haman mask transform the raw point cloud asc file into a human point cloud asc file 
// Input: Raw 3D point cloud captured by Kinect V2 (asc) & 
//		  background color image (bmp) & human color image (bmp)
// Output: Human point cloud

// OpenCV Header
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
// output asc file
#include <fstream>
// process asc file
#include <memory.h>

// Use standard output to view result in the console window
#include <iostream>
using std::cout;
using std::endl;
using std::cin;

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

char* window_name = "Image output";
//char* original_image_file = "axis_color.bmp";
char* original_image = "human_color.bmp";
char* background_image = "background_color.bmp";

char* absdiff_binary_image = "absdiff_binary.bmp";
//char* Background_Remove_Color_image_file = "Background_Remove_Color.bmp";
char* watershed_segment_image = "watershed_segment.bmp";

cv::Mat background_color = cv::imread(background_image);				// get the background image frame
cv::Mat raw_color = cv::imread(original_image);	// the image that is constantly being updated
cv::Mat absdiff_color = cv::Mat(cv::Size(raw_color.rows, raw_color.cols), CV_8UC1);

cv::Mat absdiff_gray = cv::Mat(cv::Size(raw_color.rows, raw_color.cols), CV_8UC1);
cv::Mat absdiff_binary = cv::Mat(cv::Size(raw_color.rows, raw_color.cols), CV_8UC1);

cv::Mat markers(absdiff_binary.size(), CV_8U, cv::Scalar(0));

void release_mat_memory()
{
	background_color.release();
	raw_color.release();
	absdiff_color.release();
	absdiff_gray.release();
	absdiff_binary.release();
	markers.release();
}

void absdiff()
{
	cv::absdiff(background_color, raw_color, absdiff_color);
	cv::cvtColor(absdiff_color, absdiff_gray, CV_RGB2GRAY);
	cv::threshold(absdiff_gray, absdiff_binary, 40, 255, cv::THRESH_BINARY);
	// Save the difference image
	//cv::imwrite("absdiff_color.bmp", absdiff_color);
	cv::imwrite("absdiff_gray.bmp", absdiff_gray);
	cv::imwrite(absdiff_binary_image, absdiff_binary);
}

void watershed()
{
	cv::Mat image = absdiff_color;
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
	cv::imwrite(watershed_segment_image, markers);

	// Release memory of opencv matrix
	image.release();
	gray.release();
	binary.release();
	fg.release();
	bg.release();
	//markers.release();
}

void human_mask();

int main()
{
	absdiff();
	watershed();
	human_mask();
	release_mat_memory();
	return 0;
}

void human_mask()
{
	// Size of input picture, the watershed segmented picture
	const int width = 1920;
	const int height = 1080;
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

	// initialize the input asc file
	std::fstream raw_point;
	// Load the raw point cloud file
	raw_point.open("raw_point.asc", std::ios::in);
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
		raw_point >> points_array[i];
		if (i % 3 == 0) {
			X[count_x] = points_array[i];
			count_x++;
		}
		if (i % 3 == 1) {
			Y[count_y] = points_array[i];
			count_y++;
		}
		if (i % 3 == 2) {
			Z[count_z] = points_array[i];
			count_z++;
		}
	}

	// initialize the output asc file
	std::ofstream human_asc("human_point.asc");
	//std::ofstream colored_human_asc("colored_human_point.asc");

	// output human point cloud above the ankle segmented by bounding box
	for (long int i = 0; i < width * pixel_height_of_ankle; i++) {
		if (-X[i] < 3100 && -X[i] != 0 && -X[i] > 1900 && Y[i] < 690 && Y[i] > -690 && Z[i] > -950) {
			human_asc << X[i] << " " << Y[i] << " " << Z[i] << " " << endl;
		}
		//if (Z[i] < 2800 && Z[i] != 0 && Z[i] > 2350 && X[i] < 800 && X[i] > -690 && Y[i] > -950) {
		//	colored_human_asc << X[i] << " " << Y[i] << " " << Z[i] << " " << endl;
		//}
	}

	// output human point cloud below the ankle segmented by Absdiff  
	for (long int i = width * pixel_height_of_ankle; i < total_pixels; i++) {
		if (idx[i] == 1) {
			if (-X[i] < 3100 && -X[i] != 0 && -X[i] > 1900 && Y[i] < 690 && Y[i] > -690 && Z[i] > -950) {
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
	raw_point.close();
	human_asc.close();
	//colored_human_asc.close();
	// release the memory of opencv matrix
	//markers.release();
	//cv::waitKey(0);
}