#include "StdAfx.h"
#include "matrix_cal.h"

#include <math.h>
//#include "imsl.h"  //使用方法參閱IMSL使用手冊

matrixA::matrixA()
{
	arr = 0;
}

matrixA::~matrixA()
{
	if (arr != 0)
	{
		for (int i=0;i<row;i++)
			delete [] arr[i];
		delete [] arr;
	}
}

matrixA::matrixA(int row, int column)
{
	if (row > 0  &&  column > 0)
	{
		this->row = row;
		this->column = column;

		arr = new double * [row];
		for (int i=0;i<row;i++)
			arr[i] = new double [column];
	}
	else
		arr = 0;
}

matrixA::matrixA(int row, int column, double **array)
{
	if (row > 0  &&  column > 0)
	{
		this->row = row;
		this->column = column;

		arr = new double * [row];
		for (int i=0;i<row;i++)
			arr[i] = new double [column];

		for (int i=0;i<row;i++)
			for (int j=0;j<column;j++)
				arr[i][j] = array[i][j];
	}
	else
		arr = 0;
}

matrixA::matrixA(int row, int column, double constant)
{
	if (row > 0  &&  column > 0)
	{
		this->row = row;
		this->column = column;

		arr = new double * [row];
		for (int i=0;i<row;i++)
			arr[i] = new double [column];

		for (int i=0;i<row;i++)
			for (int j=0;j<column;j++)
				arr[i][j] = constant;
	}
	else
		arr = 0;
}

matrixA::matrixA(matrixA& matrix)
{
	row = matrix.row;
	column = matrix.column;

	arr = new double * [row];
	for (int i=0;i<row;i++)
		arr[i] = new double [column];

	for (int i=0;i<row;i++)
		for (int j=0;j<column;j++)
			arr[i][j] = matrix.arr[i][j];
}

bool matrixA::Initial(int row, int column)
{
	return Initial(row, column, double(0));
}

bool matrixA::Initial(int row, int column, double **array)
{
	if (row > 0  &&  column > 0)
	{
		this->row = row;
		this->column = column;

		arr = new double * [row];
		for (int i=0;i<row;i++)
			arr[i] = new double [column];

		for (int i=0;i<row;i++)
			for (int j=0;j<column;j++)
				arr[i][j] = array[i][j];

		return true;
	}
	else
		return false;
}

bool matrixA::Initial(int row, int column, double constant)
{
	if (row > 0  &&  column > 0)
	{
		this->row = row;
		this->column = column;

		arr = new double * [row];
		for (int i=0;i<row;i++)
			arr[i] = new double [column];

		for (int i=0;i<row;i++)
			for (int j=0;j<column;j++)
				arr[i][j] = constant;

		return true;
	}
	else
		return false;
}

matrixA& matrixA::operator=(const matrixA& matrix)
{
	if (arr != 0  &&  matrix.arr != 0)
		if (row == matrix.row  &&  column == matrix.column)
			for (int i=0;i<row;i++)
				for (int j=0;j<column;j++)
					arr[i][j] = matrix.arr[i][j];

	return *this;
}

matrixA& matrixA::operator+=(const matrixA& matrix)
{
	if (arr != 0  &&  matrix.arr != 0)
		if (row == matrix.row  &&  column == matrix.column)
			for (int i=0;i<row;i++)
				for (int j=0;j<column;j++)
					arr[i][j] += matrix.arr[i][j];

	return *this;
}

matrixA& matrixA::operator-=(const matrixA& matrix)
{
	if (arr != 0  &&  matrix.arr != 0)
		if (row == matrix.row  &&  column == matrix.column)
			for (int i=0;i<row;i++)
				for (int j=0;j<column;j++)
					arr[i][j] -= matrix.arr[i][j];

	return *this;
}

matrixA matrixA::operator+(const matrixA& matrix)
{
	if (arr != 0  &&  matrix.arr != 0)
	{
		if (row == matrix.row  &&  column == matrix.column)
		{
			matrixA temp(row, column);

			for (int i=0;i<row;i++)
				for (int j=0;j<column;j++)
					temp.arr[i][j] = arr[i][j] + matrix.arr[i][j];

			return temp;
		}
		else
			return *this;
	}
	else
		return *this;
}

matrixA matrixA::operator-(const matrixA& matrix)
{
	if (arr != 0  &&  matrix.arr != 0)
	{
		if (row == matrix.row  &&  column == matrix.column)
		{
			matrixA temp(row, column);

			for (int i=0;i<row;i++)
				for (int j=0;j<column;j++)
					temp.arr[i][j] = arr[i][j] - matrix.arr[i][j];

			return temp;
		}
		else
			return *this;
	}
	else
		return *this;
}

matrixA matrixA::operator*(const matrixA& matrix)
{
    if (arr != 0  &&  matrix.arr != 0)
	{
		if (column == matrix.row)
		{
			matrixA temp(row, matrix.column, double(0));

			for (int i=0;i<row;i++)
				for (int j=0;j<matrix.column;j++)
					for (int k=0;k<column;k++)
						temp.arr[i][j] += arr[i][k] * matrix.arr[k][j];

			return temp;
		}
		return *this;
	}
	else
		return *this;
}

matrixA matrixA::operator*(const double& multiplier)
{
	if (arr != 0)
	{
		matrixA temp(row, column);

		for (int i=0;i<row;i++)
			for (int j=0;j<column;j++)
				temp.arr[i][j] = arr[i][j] * multiplier;

		return temp;
	}
	else
		return *this;
}

matrixA matrixA::operator/(const double& divisor)
{
	if (arr != 0)
	{
		matrixA temp(row, column);

		for (int i=0;i<row;i++)
			for (int j=0;j<column;j++)
				temp.arr[i][j] = arr[i][j] / divisor;

		return temp;
	}
	else
		return *this;
}

bool matrixA::operator==(const matrixA& matrix)
{
	if (arr != 0  &&  matrix.arr != 0)
	{
		if (row == matrix.row  &&  column == matrix.column)
		{
			for (int i=0;i<row;i++)
				for (int j=0;j<column;j++)
					if (arr[i][j] != matrix.arr[i][j])
						return false;

			return true;
		}
		else
			return false;
	}
	else
		return false;
}

bool matrixA::operator!=(const matrixA& matrix)
{
	if (arr != 0  &&  matrix.arr != 0)
	{
		if (row == matrix.row  &&  column == matrix.column)
		{
			for (int i=0;i<row;i++)
				for (int j=0;j<column;j++)
					if (arr[i][j] != matrix.arr[i][j])
						return true;

			return false;
		}
		else
			return true;
	}
	else
		return true;
}

//Point2D matrixA::operator*(const Point2D& pt)
//{
//	if (arr != 0  ||  row == 3  || column == 3)
//		return Point2D(arr[0][0] * pt.x + arr[0][1] * pt.y + arr[0][2], arr[1][0] * pt.x + arr[1][1] * pt.y + arr[1][2]);
//	else
//		return Point2D(0, 0);
//}

//Point3D matrixA::operator*(const Point3D& pt)
//{
//	if (arr != 0  ||  row == 4  || column == 4)
//		return Point3D(arr[0][0] * pt.x + arr[0][1] * pt.y + arr[0][2] * pt.z + arr[0][3], arr[1][0] * pt.x + arr[1][1] * pt.y + arr[1][2] * pt.z + arr[1][3], arr[2][0] * pt.x + arr[2][1] * pt.y + arr[2][2] * pt.z + arr[2][3]);
//	else
//		return Point3D(0, 0, 0);
//}

matrixA matrixA::Inverse()
{
	if (arr != 0  &&  row == column)
	{
		matrixA temp(row, 2 * column);

		for (int i=0;i<row;i++)
		{
			for (int j=0;j<column;j++)
				temp.arr[i][j] = arr[i][j];

			for (int j=column;j<2*column;j++)
			{
				if (j-column == i)
					temp.arr[i][j] = 1;
				else
					temp.arr[i][j] = 0;
			}
		}

		double divisor, multiplier;
		for (int i=0;i<row;i++)
		{
			divisor = temp.arr[i][i];

			temp.arr[i][i] = 1;
			for (int j=i+1;j<2*column;j++)
			{
				temp.arr[i][j] /= divisor;
			}

			for (int m=0;m<row;m++)
			{
				if (m != i)
				{
					multiplier = - temp.arr[m][i];
					for (int n=i;n<2*column;n++)
					{
						temp.arr[m][n] += multiplier * temp.arr[i][n];
					}
				}
			}
		}

		matrixA sol(row, column);

		for (int i=0;i<row;i++)
			for (int j=0;j<column;j++)
				sol.arr[i][j] = temp.arr[i][j+column];

		return sol;
	}
	else
		return *this;
}

matrixA matrixA::PseudoInverse()
{
	if (arr != 0)
		return (this->Transpose() * (*this)).Inverse() * this->Transpose();
	else
		return *this;
}

matrixA matrixA::Transpose()
{
	if (arr != 0)
	{
		matrixA temp(column, row);

		for (int i=0;i<column;i++)
			for (int j=0;j<row;j++)
				temp.arr[i][j] = arr[j][i];

		return temp;
	}
	else
		return *this;
}
