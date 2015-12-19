// opencv_practice.cpp : 定義主控台應用程式的進入點。
//
#include "stdafx.h"
#include <iostream>
#include "Plane.h"
#include "opencv/highgui.h"
#include "opencv/cv.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui_c.h"
using namespace std;
#define ROW_SMALL 600
#define ROW_BIG 900
#define COL_SMALL 700
#define COL_BIG 1200
cv::Mat Image;
cv::Mat Imagebg;
cv::Mat Imageresult;
cv::Mat binaryimage;
cv::Mat Image_de;
cv::Mat blurImg;

void onTrackbar(int position);
void filter();
void Mask_center();
void Center_filter();
void cal_center(Point &center);
void PlaneFit(Plane &floor);
void bottle_filter();
void watershed(cv::Mat &input);
void cvFillHoles(cv::Mat &input);
void test(cv::Mat &input, int k);
void corner_filter(Point &center_point);
void Find_contour(cv::Mat &input, int row_small, int row_big, int col_small, int col_big, int &k);

int _tmain(int argc, char** argv)
{
	Image = cv::imread("background_board_gray.bmp", 0);
	//Gray Image1.bmp
	
	Imagebg = cv::imread("background_gray.bmp", 0);
	//Gray Image2.bmp
	cv::subtract(Imagebg, Image, Imageresult);//相減
	//cv::imwrite("subtract.bmp", Imageresult);
	threshold(Imageresult, binaryimage, 20, 255, CV_THRESH_BINARY);
	//cv::imwrite("result.bmp", binaryimage);
	watershed(binaryimage);
	cv::dilate(binaryimage, Image_de, cv::Mat(), cv::Point(-1, -1), 6, 1, 1);
	cv::erode(Image_de, Image_de, cv::Mat(), cv::Point(-1, -1), 5, 1, 1);
	//imwrite("d&e.bmp", Image_de);
	cvFillHoles(Image_de);
	//imwrite("flood.bmp", Image_de);
	medianBlur(Image_de, blurImg, 3);
	
	cv::Mat afterdil;
	cv::dilate(blurImg, afterdil, cv::Mat(), cv::Point(-1, -1), 8, 1, 1);	
	cv::Mat Image_bound;
	cv::subtract(afterdil, blurImg, Image_bound);
	cv::imwrite("mask.bmp", Image_bound);
	int k;
	Find_contour(blurImg, ROW_SMALL, ROW_BIG, COL_SMALL, COL_BIG,k);
	test(blurImg,k);

	cv::destroyAllWindows();
	Image.release();
	Imagebg.release();
	Imageresult.release();
	binaryimage.release();
		
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
	system("pause");
	return 0;
}
