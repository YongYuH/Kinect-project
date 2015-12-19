#ifndef __matrix_cal_H__
#define __matrix_cal_H__

//#include "StructDefine.h"
//#include "Point3D.h"

class matrixA
{
	/////////////////////////////////////
	//   �s��  |   math01              //
	//   �W��  |   �x�}�B��            //
	//   �@��  |   �ªë�              //
	//   ���  |   2000/9              //
	/////////////////////////////////////
public:
	int row;     //matrix�e��
	int column;    //matrix����
	double **arr;  //����matrix���G���}�C
	bool check;    //���չB��i�_����

	matrixA();
	~matrixA();
	matrixA(int row, int column);
    matrixA(int row, int column, double **array);
	matrixA(int row, int column, double constant);
	matrixA(matrixA& matrix);

	bool Initial(int row, int column);
	bool Initial(int row, int column, double **array);
	bool Initial(int row, int column, double constant);

    matrixA& operator=(const matrixA& matrix);
	matrixA& operator+=(const matrixA& matrix);
	matrixA& operator-=(const matrixA& matrix);
	matrixA operator+(const matrixA& matrix);
	matrixA operator-(const matrixA& matrix);
	matrixA operator*(const matrixA& matrix);
    matrixA operator*(const double& multiplier);
	matrixA operator/(const double& divisor);
	bool operator==(const matrixA& matrix);
	bool operator!=(const matrixA& matrix);

	//Point2D operator*(const Point2D & pt);
	//Point3D operator*(const Point3D & pt);

	matrixA Inverse();
	matrixA PseudoInverse();
	matrixA Transpose();
};

#endif