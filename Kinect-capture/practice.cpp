#include "stdafx.h"
#include <iostream>
#include <fstream>
#include "opencv/highgui.h"
#include "opencv/cv.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "Plane.h"
#include <vector>
using namespace std;

int *Sobel1D(int len, int *arr, int masklen);
bool intersection(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2, cv::Point2f &r);

//void filter()
//{
//	// Size of input picture, the watershed segmented picture
//	const int width = 1920;
//	const int height = 1080;
//	const long int total_pixels = width * height;
//
//	cv::Mat img = cv::imread("mask.bmp");
//	// An one dimensional array to index whether the grayscale of each pixel is white
//	short* idx;
//	idx = new short[total_pixels];
//	memset(idx, 0, total_pixels*sizeof(short));
//	// load grayscale of each pixel in the mask
//	for (int j = 0; j < img.rows; j++) {
//		for (int i = 0; i < img.cols; i++) {
//			cv::Scalar intensity = img.at<cv::Vec3b>(j, i);
//			// when the mask is white
//			if (intensity.val[0] == 255) {
//				idx[i + j * img.cols] = 1;
//			}
//		}
//	}
//	
//	// initialize the input asc file
//	std::fstream raw_point;
//	raw_point.open("board_3Dpoints.asc", std::ios::in);
//	
//	double* points_array;
//	points_array = new double[3 * total_pixels];
//	memset(points_array, 0, 3 * total_pixels*sizeof(double));
//
//	double* X;
//	X = new double[total_pixels];
//	memset(X, 0, total_pixels*sizeof(double));
//	int count_x = 0;
//
//	double* Y;
//	Y = new double[total_pixels];
//	memset(Y, 0, total_pixels*sizeof(double));
//	int count_y = 0;
//
//	double* Z;
//	Z = new double[total_pixels];
//	memset(Z, 0, total_pixels*sizeof(double));
//	int count_z = 0;
//
//	int pixel_height_of_ankle = 750;
//	// Save the (x, y, z) points into three arrays
//	for (long int i = 0; i < 3 * total_pixels; i++) {
//		raw_point >> points_array[i];
//		if (i % 3 == 0) {
//			X[count_x] = points_array[i];
//			count_x++;
//		}
//		if (i % 3 == 1) {
//			Y[count_y] = points_array[i];
//			count_y++;
//		}
//		if (i % 3 == 2) {
//			Z[count_z] = points_array[i];
//			count_z++;
//		}
//	}
//	// initialize the output asc file
//	std::ofstream floor_asc("floor_point.asc");
//
//	for (long int i = 0; i < total_pixels; i++) {
//		if (idx[i] == 1) {
//			if (X[i] > -3700 && X[i] != 0 && X[i] < -2000 && Y[i] < 800 && Y[i] > -800 && Z[i] > -950)
//			{
//				floor_asc << X[i] << " " << Y[i] << " " << Z[i] << " " << endl;
//			}
//		}
//	}
//
//	// release the memory of array
//	delete[] idx;
//	delete[] points_array;
//	delete[] X;
//	delete[] Y;
//	delete[] Z;
//	// close the asc file
//	raw_point.close();
//	floor_asc.close();
//	img.release();
//}
//
//void corner_filter(Point &center_point)
//{
//	// Size of input picture, the watershed segmented picture
//	const int width = 1920;
//	const int height = 1080;
//	const long int total_pixels = width * height;
//
//	cv::Mat img1 = cv::imread("four_point_mask34.bmp");
//	cv::Mat img2 = cv::imread("four_point_mask12.bmp");
//	cv::Mat img3 = cv::imread("four_point_mask23.bmp");
//	cv::Mat img4 = cv::imread("four_point_mask14.bmp");
//	// An one dimensional array to index whether the grayscale of each pixel is white
//	short *idx1, *idx2, *idx3, *idx4;
//	idx1 = new short[total_pixels];
//	idx2 = new short[total_pixels];
//	idx3 = new short[total_pixels];
//	idx4 = new short[total_pixels];
//	memset(idx1, 0, total_pixels*sizeof(short));
//	memset(idx2, 0, total_pixels*sizeof(short));
//	memset(idx3, 0, total_pixels*sizeof(short));
//	memset(idx4, 0, total_pixels*sizeof(short));
//	// load grayscale of each pixel in the mask
//	for (int j = 0; j < img1.rows; j++) {
//		for (int i = 0; i < img1.cols; i++) {
//			cv::Scalar intensity = img1.at<cv::Vec3b>(j, i);
//			// when the mask is white
//			if (intensity.val[0] == 255) {
//				idx1[i + j * img1.cols] = 1;
//			}
//		}
//	}
//	for (int j = 0; j < img2.rows; j++) {
//		for (int i = 0; i < img2.cols; i++) {
//			cv::Scalar intensity = img2.at<cv::Vec3b>(j, i);
//			// when the mask is white
//			if (intensity.val[0] == 255) {
//				idx2[i + j * img2.cols] = 1;
//			}
//		}
//	}
//	for (int j = 0; j < img3.rows; j++) {
//		for (int i = 0; i < img3.cols; i++) {
//			cv::Scalar intensity = img3.at<cv::Vec3b>(j, i);
//			// when the mask is white
//			if (intensity.val[0] == 255) {
//				idx3[i + j * img3.cols] = 1;
//			}
//		}
//	}
//	for (int j = 0; j < img4.rows; j++) {
//		for (int i = 0; i < img4.cols; i++) {
//			cv::Scalar intensity = img4.at<cv::Vec3b>(j, i);
//			// when the mask is white
//			if (intensity.val[0] == 255) {
//				idx4[i + j * img4.cols] = 1;
//			}
//		}
//	}
//
//	// initialize the input asc file
//	std::fstream raw_point;
//	raw_point.open("board_3Dpoints.asc", std::ios::in);
//
//	double* points_array;
//	points_array = new double[3 * total_pixels];
//	memset(points_array, 0, 3 * total_pixels*sizeof(double));
//
//	double* X;
//	X = new double[total_pixels];
//	memset(X, 0, total_pixels*sizeof(double));
//	int count_x = 0;
//
//	double* Y;
//	Y = new double[total_pixels];
//	memset(Y, 0, total_pixels*sizeof(double));
//	int count_y = 0;
//
//	double* Z;
//	Z = new double[total_pixels];
//	memset(Z, 0, total_pixels*sizeof(double));
//	int count_z = 0;
//
//	// Save the (x, y, z) points into three arrays
//	for (long int i = 0; i < 3 * total_pixels; i++) {
//		raw_point >> points_array[i];
//		if (i % 3 == 0) {
//			X[count_x] = points_array[i];
//			count_x++;
//		}
//		if (i % 3 == 1) {
//			Y[count_y] = points_array[i];
//			count_y++;
//		}
//		if (i % 3 == 2) {
//			Z[count_z] = points_array[i];
//			count_z++;
//		}
//	}
//
//	// initialize the output asc file
//	std::ofstream corner_asc("corner_point.asc");
//	
//	vector<double> x_corner;
//	vector<double> y_corner;
//	vector<double> z_corner;
//	double x_sum1 = 0; double x_sum2 = 0; double x_sum3 = 0; double x_sum4 = 0;
//	double y_sum1 = 0; double y_sum2 = 0; double y_sum3 = 0; double y_sum4 = 0;
//	double z_sum1 = 0; double z_sum2 = 0; double z_sum3 = 0; double z_sum4 = 0;
//	for (long int i = 0; i < total_pixels; i++) {
//		if (idx1[i] == 1) {
//			{
//				x_corner.push_back(X[i]);
//				y_corner.push_back(Y[i]);
//				z_corner.push_back(Z[i]);
//			}
//		}
//	}
//	for (int i = 0; i < x_corner.size(); i++){
//		x_sum1 += x_corner[i];
//		y_sum1 += y_corner[i];
//		z_sum1 += z_corner[i];
//	}
//	x_sum1 /= x_corner.size();
//	y_sum1 /= y_corner.size();
//	z_sum1 /= z_corner.size();
//	
//	x_corner.clear();
//	y_corner.clear();
//	z_corner.clear();
//	for (long int i = 0; i < total_pixels; i++) {
//		if (idx2[i] == 1) {
//			{
//				x_corner.push_back(X[i]);
//				y_corner.push_back(Y[i]);
//				z_corner.push_back(Z[i]);
//			}
//		}
//	}
//	for (int i = 0; i < x_corner.size(); i++){
//		x_sum2 += x_corner[i];
//		y_sum2 += y_corner[i];
//		z_sum2 += z_corner[i];
//	}
//	x_sum2 /= x_corner.size();
//	y_sum2 /= y_corner.size();
//	z_sum2 /= z_corner.size();
//	
//	x_corner.clear();
//	y_corner.clear();
//	z_corner.clear();
//	for (long int i = 0; i < total_pixels; i++) {
//		if (idx3[i] == 1) {
//			{
//				x_corner.push_back(X[i]);
//				y_corner.push_back(Y[i]);
//				z_corner.push_back(Z[i]);
//			}
//		}
//	}
//	for (int i = 0; i < x_corner.size(); i++){
//		x_sum3 += x_corner[i];
//		y_sum3 += y_corner[i];
//		z_sum3 += z_corner[i];
//	}
//	x_sum3 /= x_corner.size();
//	y_sum3 /= y_corner.size();
//	z_sum3 /= z_corner.size();
//	
//	x_corner.clear();
//	y_corner.clear();
//	z_corner.clear();
//	for (long int i = 0; i < total_pixels; i++) {
//		if (idx4[i] == 1) {
//			{
//				x_corner.push_back(X[i]);
//				y_corner.push_back(Y[i]);
//				z_corner.push_back(Z[i]);
//			}
//		}
//	}
//	for (int i = 0; i < x_corner.size(); i++){
//		x_sum4 += x_corner[i];
//		y_sum4 += y_corner[i];
//		z_sum4 += z_corner[i];
//	}
//	x_sum4 /= x_corner.size();
//	y_sum4 /= y_corner.size();
//	z_sum4 /= z_corner.size();
//	
//	corner_asc << x_sum1 << " " << y_sum1 << " " << z_sum1 << endl;
//	corner_asc << x_sum2 << " " << y_sum2 << " " << z_sum2 << endl;
//	corner_asc << x_sum3 << " " << y_sum3 << " " << z_sum3 << endl;
//	corner_asc << x_sum4 << " " << y_sum4 << " " << z_sum4 << endl;
//	corner_asc << (x_sum1 + x_sum2 + x_sum3+x_sum4)/4 << " " <<
//		(y_sum1 + y_sum2 + y_sum3 + y_sum4) / 4 << " " << 
//		(z_sum1 + z_sum2 + z_sum3 + z_sum4) / 4 << endl;
//	
//	center_point.x = (x_sum1 + x_sum2 + x_sum3 + x_sum4) / 4;
//	center_point.y = (y_sum1 + y_sum2 + y_sum3 + y_sum4) / 4;
//	center_point.z = (z_sum1 + z_sum2 + z_sum3 + z_sum4) / 4;
//
//	// release the memory of array
//	delete[] idx1;
//	delete[] idx2;
//	delete[] idx3;
//	delete[] idx4;
//	delete[] points_array;
//	delete[] X;
//	delete[] Y;
//	delete[] Z;
//	// close the asc file
//	raw_point.close();
//	corner_asc.close();
//	img1.release();
//	img2.release();
//	img3.release();
//	img4.release();
//
//}

void cvFillHoles(cv::Mat &input)
{
	//assume input is uint8 B & W (0 or 1)
	//this function imitates imfill(image,'hole')
	cv::Mat holes = input.clone();
	cv::floodFill(holes, cv::Point2i(0, 0), cv::Scalar(1));
	for (int i = 0; i<input.rows*input.cols; i++)
	{
		if (holes.data[i] == 0)
			input.data[i] = 255;
	}
}

void watershed(cv::Mat &input)
{
	cv::Mat image = input.clone();
	cv::Mat image2;
	cv::Mat gray;
	cv::cvtColor(image, gray, CV_GRAY2BGR,3);
	cv::cvtColor(image, image2, CV_GRAY2BGR, 3);
	
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
	cv::Mat markers(binary.size(), CV_8U, cv::Scalar(0));
	markers = fg + bg;
	cv::cvtColor(markers, markers, CV_BGR2GRAY, 1);
	markers.convertTo(markers, CV_32S);
	cv::watershed(image2, markers);

	markers.convertTo(markers, CV_8U);
	cv::threshold(markers, markers, 128, 255, cv::THRESH_BINARY);
	input = markers.clone();
	//cv::imwrite("water.bmp", markers);

	// Release memory of opencv matrix
	image.release();
	gray.release();
	binary.release();
	fg.release();
	bg.release();
	markers.release();
}

void test(cv::Mat &input,int k)
{
	cv::Mat tmp = input.clone();
	vector<cv::Point2i> white;
	cv::Point2i temp;
	cv::Point2i temp2;
	int sumx = 0;
	int sumy = 0;
	for (int i = 0; i < tmp.rows; i++) {
		for (int j = 0; j < tmp.cols; j++) {
			if ((tmp.at<unsigned char>(i,j)) == 255) {
				temp.x = j;
				temp.y = i;
				white.push_back(temp);
				sumx += temp.x;
				sumy += temp.y;
			}
		}
	}
	// Debug
	//cout << white.size()<<endl;
	int avgx = int(sumx / white.size());
	int avgy = int(sumy / white.size());
	// Debug
	//cout << avgx << " " << avgy << endl;
	for (int j = 0; j < white.size(); j++) {
		for (int i = 0; i < white.size() - 1; i++) {
			if (atan2(double((white[i].y - avgy)) , double((white[i].x - avgx))) <
				atan2(double((white[i + 1].y - avgy)), double((white[i + 1].x - avgx))))
			{
				temp2 = white[i];
				white[i] = white[i + 1];
				white[i + 1] = temp2;

			}
		}
	}
	cv::Point2i *white3x = new cv::Point2i[3 * white.size()];
	
	for (int i = 0; i < 3*white.size(); i++) {
		white3x[i] = white[i%white.size()];
	}
	int Xf, Xb, Yf, Yb,Xsum,Ysum,large;
	int max_bv = 0;
	int *bending_value = new int[white.size()];
	int *bending_value3x = new int[3*white.size()];
	for (int i = white.size(); i < 2 * white.size(); i++) {
		Xf = white3x[i + k].x - white3x[i].x;
		Xb = white3x[i - k].x - white3x[i].x;
		Yf = white3x[i + k].y - white3x[i].y;
		Yb = white3x[i - k].y - white3x[i].y;
		Xsum = abs(Xf + Xb);
		Ysum = abs(Yf + Yb);
		if (Xsum>Ysum) {
			large = Xsum;
		} else {
			large = Ysum;
		}

		bending_value[i - white.size()] = large;
		if (large > max_bv){ max_bv = large; }
	}
	int *sobel,*sobel2;
	sobel=Sobel1D(white.size(), bending_value, 9);
	sobel2 = Sobel1D(white.size(), sobel, 9);

	// Debug
	//ofstream bv("bv.txt");
	//ofstream bvs("bvs.txt");
	//ofstream bvs2("bvs2.txt");
	//ofstream all("all.txt");
	//cout << k << endl;
	//for (int i = 0; i < white.size(); i++) {
	//		bv << bending_value[i] << endl;
	//		bvs << sobel[i] << endl;
	//		bvs2 << sobel2[i] << endl;
	//		all << setw(4) << i << setw(3) << bending_value[i]
	//			<< setw(4) << sobel[i] << setw(5) << sobel2[i] << endl;
	//}

	int mark[4];
	int count = 0;
	vector<int> below_thres;
	vector<int> section1;
	vector<int> section2;
	vector<int> section3;
	vector<int> section4;
	
	for (int i = 0; i < white.size(); i++){
		if (bending_value[i] < max_bv*0.5)
			below_thres.push_back(i);
	}

	for (int i = 0; i < below_thres.size()-1; i++){
		if ((below_thres[i + 1] - below_thres[i])!=1){
			mark[count]=i;
			count++;
		}
	}

	for (int i = 0; i < below_thres.size(); i++){		
		if ((i >= mark[0]+1) && (i < mark[1]))
			section2.push_back(below_thres[i]);
		else if ((i >= mark[1]) && (i < mark[2] ))
			section3.push_back(below_thres[i]);
		else if ((i >= mark[2]) && (i < mark[3]))
			section4.push_back(below_thres[i]);
		else
			section1.push_back(below_thres[i]);
	}

	// Debug
	//ofstream sec1("sec.txt");
	//for (int i = 0; i < below_thres.size(); i++)
	//	sec1 << below_thres[i] << endl;
	//cout << section1.size() + section2.size() + section3.size() + section4.size() << endl;
	
	std::vector<cv::Point2i> points1;
	for (int i = 0; i < section1.size(); i++) {
		points1.push_back(cv::Point2i(white[section1[i]].x, white[section1[i]].y));
	}
	cv::Vec4f line1;
	cv::fitLine(points1, line1, CV_DIST_L2, 0, 0.01, 0.01);
	std::vector<cv::Point2i> points2;
	for (int i = 0; i < section2.size(); i++) {
		points2.push_back(cv::Point2i(white[section2[i]].x, white[section2[i]].y));
	}
	cv::Vec4f line2;
	cv::fitLine(points2, line2, CV_DIST_L2, 0, 0.01, 0.01);
	std::vector<cv::Point2i> points3;
	for (int i = 0; i < section3.size(); i++) {
		points3.push_back(cv::Point2i(white[section3[i]].x, white[section3[i]].y));
	}
	cv::Vec4f line3;
	cv::fitLine(points3, line3, CV_DIST_L2, 0, 0.01, 0.01);
	std::vector<cv::Point2i> points4;
	for (int i = 0; i < section4.size(); i++) {
		points4.push_back(cv::Point2i(white[section4[i]].x, white[section4[i]].y));
	}
	cv::Vec4f line4;
	cv::fitLine(points4, line4, CV_DIST_L2, 0, 0.01, 0.01);
	
	cv::Mat fourbreakline(1080, 1920, CV_8UC3, cv::Scalar(0));
	for (int j = 0; j < below_thres.size(); j++) {					
		cv::Vec3b intensity;
		intensity.val[0] = 255;
		intensity.val[1] = 255;
		intensity.val[2] = 255;
		fourbreakline.at<cv::Vec3b>(cv::Point(white[below_thres[j]].x, white[below_thres[j]].y))
				= intensity;
	}
	//imwrite("444.bmp", fourbreakline);
	cv::Mat fourline(1080, 1920, CV_8UC3, cv::Scalar(0));
	for (int i = 0; i < tmp.rows; i++){
		for (int j = 0; j < tmp.cols; j++){
			if ((tmp.at<unsigned char>(i, j)) == 255){
				cv::Vec3b intensity;
				intensity.val[0]=255;
				intensity.val[1]=255;
				intensity.val[2]=255;
				fourline.at<cv::Vec3b>(cv::Point(j, i)) = intensity;
			}
		}
	}
			
	cv::line(fourline, cv::Point2f(line4[2] + 200 * line4[0], line4[3] + 200 * line4[1]),
		cv::Point2f(line4[2] - 200 * line4[0], line4[3] - 200 * line4[1]),
		CV_RGB(255, 255,0), 1, 8, 0);
	cv::Point2f  line4_1(line4[2], line4[3]); 
	cv::Point2f  line4_2(line4[2]+line4[0], line4[3]+line4[1]);
	cv::line(fourline, cv::Point2f(line3[2] + 200 * line3[0], line3[3] + 200 * line3[1]),
		cv::Point2f(line3[2] - 200 * line3[0], line3[3] - 200 * line3[1]),
		CV_RGB(255, 255,0),1, 8, 0);
	cv::Point2f  line3_1(line3[2], line3[3]);
	cv::Point2f  line3_2(line3[2] + line3[0], line3[3] + line3[1]);
	cv::line(fourline, cv::Point2f(line1[2] + 200 * line1[0], line1[3] + 200 * line1[1]),
		cv::Point2f(line1[2] - 200 * line1[0], line1[3] - 200 * line1[1]),
		CV_RGB(255, 255, 0), 1, 8, 0);
	cv::Point2f  line1_1(line1[2], line1[3]);
	cv::Point2f  line1_2(line1[2] + line1[0], line1[3] + line1[1]);
	cv::line(fourline, cv::Point2f(line2[2] + 200 * line2[0], line2[3] + 200 * line2[1]),
		cv::Point2f(line2[2] - 200 * line2[0], line2[3] - 200 * line2[1]),
		CV_RGB(255, 255,0), 1, 8, 0);
	cv::Point2f  line2_1(line2[2], line2[3]);
	cv::Point2f  line2_2(line2[2] + line2[0], line2[3] + line2[1]);

	cv::Point2f intersection34;
	cv::Point2f intersection12;
	cv::Point2f intersection23;
	cv::Point2f intersection14;
	intersection(line4_1, line4_2, line3_1, line3_2, intersection34);
	intersection(line1_1, line1_2, line2_1, line2_2, intersection12);
	intersection(line2_1, line2_2, line3_1, line3_2, intersection23);
	intersection(line4_1, line4_2, line1_1, line1_2, intersection14);
	cout << intersection34.x << "  " << intersection34.y << endl;
	cout << intersection14.x << "  " << intersection14.y << endl;
	cout << intersection23.x << "  " << intersection23.y << endl;
	cout << intersection12.x << "  " << intersection12.y << endl;

	cv::circle(fourline, intersection34, 2, cv::Scalar(255,0, 0), 2);
	cv::circle(fourline, intersection12, 2, cv::Scalar(255, 0, 0), 2);
	cv::circle(fourline, intersection23, 2, cv::Scalar(255, 0, 0), 2);
	cv::circle(fourline, intersection14, 2, cv::Scalar(255, 0, 0), 2);
	
	// 4個角點的8相鄰
	cv::Mat four_point_mask34(1080, 1920, CV_8UC1, cv::Scalar(0));
	four_point_mask34.at<unsigned char>((int)(intersection34.y), (int)(intersection34.x)) = 255;
	four_point_mask34.at<unsigned char>((int)(intersection34.y)+1, (int)(intersection34.x)) = 255;
	four_point_mask34.at<unsigned char>((int)(intersection34.y)-1, (int)(intersection34.x)) = 255;
	four_point_mask34.at<unsigned char>((int)(intersection34.y), (int)(intersection34.x)+1) = 255;
	four_point_mask34.at<unsigned char>((int)(intersection34.y), (int)(intersection34.x)-1) =255;
	four_point_mask34.at<unsigned char>((int)(intersection34.y)+1, (int)(intersection34.x)+1) = 255;
	four_point_mask34.at<unsigned char>((int)(intersection34.y)-1, (int)(intersection34.x)-1) = 255;
	four_point_mask34.at<unsigned char>((int)(intersection34.y)+1, (int)(intersection34.x)-1) = 255;
	four_point_mask34.at<unsigned char>((int)(intersection34.y)-1, (int)(intersection34.x)+1) = 255;
	cv::imwrite("four_point_mask34.bmp", four_point_mask34);

	cv::Mat four_point_mask12(1080, 1920, CV_8UC1, cv::Scalar(0));
	four_point_mask12.at<unsigned char>((int)(intersection12.y), (int)(intersection12.x)) = 255;
	four_point_mask12.at<unsigned char>((int)(intersection12.y) + 1, (int)(intersection12.x)) = 255;
	four_point_mask12.at<unsigned char>((int)(intersection12.y) - 1, (int)(intersection12.x)) = 255;
	four_point_mask12.at<unsigned char>((int)(intersection12.y), (int)(intersection12.x) + 1) = 255;
	four_point_mask12.at<unsigned char>((int)(intersection12.y), (int)(intersection12.x) - 1) = 255;
	four_point_mask12.at<unsigned char>((int)(intersection12.y) + 1, (int)(intersection12.x) + 1) = 255;
	four_point_mask12.at<unsigned char>((int)(intersection12.y) - 1, (int)(intersection12.x) - 1) = 255;
	four_point_mask12.at<unsigned char>((int)(intersection12.y) + 1, (int)(intersection12.x) - 1) = 255;
	four_point_mask12.at<unsigned char>((int)(intersection12.y) - 1, (int)(intersection12.x) + 1) = 255;
	cv::imwrite("four_point_mask12.bmp", four_point_mask12);

	cv::Mat four_point_mask23(1080, 1920, CV_8UC1, cv::Scalar(0));
	four_point_mask23.at<unsigned char>((int)(intersection23.y), (int)(intersection23.x)) = 255;
	four_point_mask23.at<unsigned char>((int)(intersection23.y) + 1, (int)(intersection23.x)) = 255;
	four_point_mask23.at<unsigned char>((int)(intersection23.y) - 1, (int)(intersection23.x)) = 255;
	four_point_mask23.at<unsigned char>((int)(intersection23.y), (int)(intersection23.x) + 1) = 255;
	four_point_mask23.at<unsigned char>((int)(intersection23.y), (int)(intersection23.x) - 1) = 255;
	four_point_mask23.at<unsigned char>((int)(intersection23.y) + 1, (int)(intersection23.x) + 1) = 255;
	four_point_mask23.at<unsigned char>((int)(intersection23.y) - 1, (int)(intersection23.x) - 1) = 255;
	four_point_mask23.at<unsigned char>((int)(intersection23.y) + 1, (int)(intersection23.x) - 1) = 255;
	four_point_mask23.at<unsigned char>((int)(intersection23.y) - 1, (int)(intersection23.x) + 1) = 255;
	cv::imwrite("four_point_mask23.bmp", four_point_mask23);

	cv::Mat four_point_mask14(1080, 1920, CV_8UC1, cv::Scalar(0));
	four_point_mask14.at<unsigned char>((int)(intersection14.y), (int)(intersection14.x)) = 255;
	four_point_mask14.at<unsigned char>((int)(intersection14.y) + 1, (int)(intersection14.x)) = 255;
	four_point_mask14.at<unsigned char>((int)(intersection14.y) - 1, (int)(intersection14.x)) = 255;
	four_point_mask14.at<unsigned char>((int)(intersection14.y), (int)(intersection14.x) + 1) = 255;
	four_point_mask14.at<unsigned char>((int)(intersection14.y), (int)(intersection14.x) - 1) = 255;
	four_point_mask14.at<unsigned char>((int)(intersection14.y) + 1, (int)(intersection14.x) + 1) = 255;
	four_point_mask14.at<unsigned char>((int)(intersection14.y) - 1, (int)(intersection14.x) - 1) = 255;
	four_point_mask14.at<unsigned char>((int)(intersection14.y) + 1, (int)(intersection14.x) - 1) = 255;
	four_point_mask14.at<unsigned char>((int)(intersection14.y) - 1, (int)(intersection14.x) + 1) = 255;
	cv::imwrite("four_point_mask14.bmp", four_point_mask14);
	
	cout << "Finish finding corner!" << endl;
}

int *Sobel1D(int len,int *arr,int masklen)
{
	int *mask = new int[masklen];
	for (int i = 0; i < (masklen - 1) / 2; i++)
		mask[i] = -1;
	for (int i = ((masklen - 1) / 2)+1; i<masklen; i++)
		mask[i] = 1;
	mask[(masklen - 1) / 2] = 0;

	int *arr3x = new int[3 * len];
	for (int i = 0; i < 3 * len; i++){
		arr3x[i] = arr[i%len];
	}
	
	int *aftermask = new int[len];
	for (int i = 0; i < len; i++)
		aftermask[i] = 0;
	for (int i = len; i < 2*len; i++){
		for (int j = 0; j < masklen; j++)
			aftermask[i - len] += arr3x[i+j - ((masklen - 1) / 2)] * mask[j];
	}
	
	delete []mask;
	delete []arr3x;
	return aftermask;
}

void Find_contour(cv::Mat &input,int row_small,int row_big,int col_small,int col_big,int &k)
{
	int big = row_small;
	int small = row_big;
	cv::Mat contour(input.size(), CV_8UC1, cv::Scalar(0));
	
	for (int i = row_small; i < row_big ; i++)
	{
		for (int j = col_small; j < col_big ; j++)
		{
			if ((input.at<unsigned char>(i, j) == 0) && 
				((input.at<unsigned char>(i + 1, j) == 255) ||
				(input.at<unsigned char>(i - 1, j) == 255) ||
				(input.at<unsigned char>(i, j + 1) == 255) ||
				(input.at<unsigned char>(i, j - 1) == 255)))
			{
				contour.at<unsigned char>(i, j) = 255;
				if (i>big){ big = i; }
				if (i<small){ small = i; }
			}

		}
	}
	k = (big - small) / 2;
	input=contour.clone();
}

// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
bool intersection(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2,cv::Point2f &r)
{
	cv::Point2f x = o2 - o1;
	cv::Point2f d1 = p1 - o1;
	cv::Point2f d2 = p2 - o2;

	float cross = d1.x*d2.y - d1.y*d2.x;
	if (abs(cross) < 1e-8)
		return false;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	r = o1 + d1 * t1;
	return true;
}

void PlaneFit(Plane &floor)
{
	std::fstream floor_point;
	floor_point.open("floor_point.asc", std::ios::in);
	//Plane floor;
	Point temp;
	double x, y, z;
	int i = 0;
	while (floor_point >> temp.x >> temp.y >> temp.z)
	{
		if (i == 0)
		{
			floor.P_origin.push_back(temp);
		}
		if (i > 0)
		{
			if (temp.x == x&&temp.y == y&&temp.z == z)
			{

			}
			else
			{
				floor.P_origin.push_back(temp);
			}
		}
		x = temp.x; y = temp.y; z = temp.z;
		i++;
	}

	////debug
	//std::ofstream del_point("1111.asc");
	//for (int i = 0; i < floor.P_origin.size(); i++){
	//	del_point << floor.P_origin[i].x << " " << floor.P_origin[i].y << " " << floor.P_origin[i].z << endl;
	//}
	////////

	cout << "Start fitting the plane" << endl;
	floor.fitting();
	cout << "Finish fitting the plane" << endl;
	std::ofstream normal_axis("normal_axis.txt");
	normal_axis << "ax + by + cz = d" << endl;
	normal_axis << "coefficient of x: a, " << "coefficient of y: b, " << "coefficient of z:c , " << "constant: d" << endl;
	normal_axis << floor.a << " " << floor.b << " " << floor.c << " " << floor.d << " " << endl;
}

//void cal_center(Point &center)
//{
//	std::fstream center_point;
//	center_point.open("center_point.asc", std::ios::in);
//	Plane floor;
//	Point temp;
//	double x, y, z;
//	double x1 = 0, y1 = 0, z1 = 0;
//	int i = 0;
//	int count = 1;
//	while (center_point >> temp.x >> temp.y >> temp.z)
//	{
//		if (i == 0)
//		{
//			x1 = temp.x; y1 = temp.y; z1 = temp.z;
//		}
//
//		if (i > 0)
//		{
//			if (temp.x == x&&temp.y == y&&temp.z == z)
//			{
//
//			}
//			else
//			{
//				x1 = x1 + temp.x; y1 = y1 + temp.y; z1 = z1 + temp.z;
//				count++;
//			}
//
//		}
//		x = temp.x; y = temp.y; z = temp.z;
//		i++;
//	}
//	cout << x1 / count << " " << y1 / count << " " << z1 / count << endl;
//	center.x = x1 / count;
//	center.y = y1 / count;
//	center.z = z1 / count;
//}
