#ifndef __POINT2D_H__
#define __POINT2D_H__

#include <math.h>

class Point2D
{
public:
	double x;
	double y;

	Point2D() {}
	~Point2D() {}
	Point2D(double x, double y) {this->x = x; this->y = y;}
	Point2D(const Point2D& pt) {x = pt.x; y = pt.y;}
	Point2D(const Point2D& pts, const Point2D& pte) {x = pte.x - pts.x; y = pte.y - pts.y;}

	Point2D& operator=(const Point2D& pt) {x = pt.x; y = pt.y; return *this;}
	Point2D& operator+=(const Point2D& pt) {x += pt.x; y += pt.y; return *this;}
	Point2D& operator-=(const Point2D& pt) {x -= pt.x; y -= pt.y; return *this;}
	Point2D operator+(const Point2D& pt) const {return Point2D(x + pt.x, y + pt.y);}
	Point2D operator-(const Point2D& pt) const {return Point2D(x - pt.x, y - pt.y);}
	Point2D operator*(const double& multiplier) const {return Point2D(x * multiplier, y * multiplier);}
	Point2D operator/(const double& divisor) const {return Point2D(x / divisor, y / divisor);}
	bool operator==(const Point2D& pt) const {if (x == pt.x && y == pt.y) return true; else return false;}
	bool operator!=(const Point2D& pt) const {if (x != pt.x || y != pt.y) return true; else return false;}

	double Norm() {return sqrt(x*x + y*y);}
	double SquareNorm() {return x*x + y*y;}

	double ScalarProduct(const Point2D& pt) const {return x * pt.x + y * pt.y;}

	Point2D Normalize() {if (x == 0 && y == 0) return Point2D(0, 0); else return *this / this->Norm();}
};

class Image2D
{
public:
	int x;
	int y;

	Image2D() {}
	~Image2D() {}
	Image2D(int x, int y) {this->x = x; this->y = y;}
	Image2D(const Image2D& pt) {x = pt.x; y = pt.y;}
	Image2D(const Image2D& pts, const Image2D& pte) {x = pte.x - pts.x; y = pte.y - pts.y;}

	Image2D& operator=(const Image2D& pt) {x = pt.x; y = pt.y; return *this;}
	Image2D& operator+=(const Image2D& pt) {x += pt.x; y += pt.y; return *this;}
	Image2D& operator-=(const Image2D& pt) {x -= pt.x; y -= pt.y; return *this;}
	Image2D operator+(const Image2D& pt) const {return Image2D(x + pt.x, y + pt.y);}
	Image2D operator-(const Image2D& pt) const {return Image2D(x - pt.x, y - pt.y);}

	bool operator==(const Image2D& pt) const {if (x == pt.x && y == pt.y) return true; else return false;}
	bool operator!=(const Image2D& pt) const {if (x != pt.x || y != pt.y) return true; else return false;}

	double Norm() {return sqrt(double(x*x + y*y));}
	int SquareNorm() {return x*x + y*y;}

	int ScalarProduct(const Image2D& pt) const {return x * pt.x + y * pt.y;}

	friend bool CompareX(const Image2D & I1, const Image2D & I2)
	{
		if(I1.x < I2.x)
			return true;
		else if(I1.x == I2.x && I1.y < I2.y)
			return true;
		else
			return false;
	}

	friend bool IsEqual(const Image2D & I1, const Image2D & I2)
	{
		if(I1.x == I2.x && I1.y == I2.y)
			return true;
		else
			return false;
	}
	
};
#endif