#ifndef Plane_H_
#define Plane_H_
#include <fstream>
#include <cmath>
#include <vector>
using std::vector;


//#define PI 3.1415926535897932384626433832795 
const double PI = 3.141592653589793;

// ::::::::: Point class :::::::::
class Point
{
public:
	float x;

	float y;

	float z;

public:
	Point()	{ x = 0; y = 0; z = 0; }			// default constructor
	Point(const float x1, const float y1, const float z1) { x = x1; y = y1; z = z1; }
	Point(const Point & p) { x = p.x, y = p.y, z = p.z; }
	~Point() {}									// destructor
	
	// overloaded operator
	Point & operator=(const Point & p) { x = p.x; y = p.y; z = p.z; return *this; }
	Point operator+(const Point & p) const { return Point( x + p.x, y + p.y, z + p.z); }
	Point operator-(const Point & p) const { return Point( x - p.x, y - p.y, z - p.z); }
	Point operator*(const double n) const { return Point(x * n, y * n, z * n); }
	Point operator/(const double n) const { return Point(x / n, y / n, z / n); }
	Point & operator+=(const Point & p) { x += p.x; y += p.y; z += p.z; return *this; }
	Point & operator-=(const Point & p) { x -= p.x; y -= p.y; z -= p.z; return *this; }
	
	double dis(const Point & p) const 
			{ return sqrt( (x - p.x)*(x - p.x) + (y - p.y)*(y - p.y) + (z - p.z)*(z - p.z) ); }
	friend bool operator==(const Point & p1, const Point & p2) 
			{ if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z) return true; return false; }
	friend std::ifstream & operator>>(std::ifstream &, Point &);
	friend std::ofstream & operator<<(std::ofstream &, const Point &);
	friend bool CompareZ(const Point & p1, const Point & p2);
	friend bool CompareZ2(std::vector<const Point> v1, std::vector<const Point> v2);
	friend bool CompareX_S2B(const Point & p1, const Point & p2);
	friend bool CompareX_B2S(const Point & p1, const Point & p2);
	friend bool CompareY_Increase(const Point & p1, const Point & p2);
	friend bool CompareY_Decrease(const Point & p1, const Point & p2);
};

//============================================================================================||
// ::::::::: CVector class :::::::::			 														  

class CVector
{
public:
	float i;

	float j;

	float k;

public:
	CVector() { i = 0; j = 0; k = 0; }			// default constructor
	CVector(const float i1, const float j1, const float k1) { i = i1, j = j1, k = k1; }
	CVector(const Point & p1, const Point & p2) 
			{ i = p2.x - p1.x; j = p2.y - p1.y; k = p2.z - p1.z; } 
	
	~CVector() {}								// destructor								
	
	// overloaded operator
	CVector & operator=(const CVector & v1) { i = v1.i; j = v1.j; k = v1.k; return *this; }
	
	CVector operator+(const CVector & v1) const 
			{ return CVector( i + v1.i, j + v1.j, k + v1.k); }
	
	CVector operator-(const CVector & v1) const 
			{ return CVector( i - v1.i, j - v1.j, k - v1.k); }	
	
	CVector operator*(const float n) const { return CVector(i * n, j * n, k * n); }
	CVector operator/(const float n) const { return CVector(i / n, j / n, k / n); }
	
	void Set(const Point & p1, const Point & p2)	// reset V : V = p2 - p1
			{ i = p2.x - p1.x; j = p2.y - p1.y; k = p2.z - p1.z; }
	

	// CVector : 010 :: normalize vector
	double absV() { return sqrt(i*i + j*j + k*k); }	// |V| ; abs(V)
	
	// CVector : 011 :: V.V1 = |V||V1|cos(theta)
	double dot(const CVector & v1) const			// V.V1 = |V||V1|cos(theta)
			{ return i*v1.i + j*v1.j + k*v1.k; } 
	
	// CVector : 012 :: V cross V1 = V X V1 = Normal
	CVector cross(const CVector & v1) const		// V cross V1 = V X V1 = Normal
			{ return CVector(j*v1.k - k*v1.j, k*v1.i - i*v1.k, i*v1.j - j*v1.i); }
	
	// CVector : 020 :: Rotations about arbitrary axes
	Point Rotation(const double theta, const Point & Ori, const Point & p);
	// Rotations about arbitrary axes
	float * Rotation(const float theta, const Point & Ori); // return rotation matrix
	
	CVector Rotation(const float theta, const Point & Ori, const CVector &); // return vector

	bool Rotation(const double theta, const Point & Ori, vector<Point> & p);
		// theta 為旋轉角度， Ori 為向量原點，p 為旋轉點群。

	CVector GetBisector(const CVector & v) 
			{  double FirAbsV = sqrt(i*i + j*j + k*k); double SecAbsV = sqrt(v.i*v.i + v.j*v.j + v.k*v.k);
			   return CVector((i/FirAbsV + v.i/SecAbsV)*0.5, (j/FirAbsV + v.j/SecAbsV)*0.5 , (k/FirAbsV + v.k/SecAbsV)*0.5); }

	void normalize(); // Pirson
};

//==========================================================================================||
// ::::::::: Line class :::::::::
class Line3D
{
private:
	CVector direction;

	Point origin;

public:
	Line3D() { direction.i = 0, direction.j = 0, direction.k = 0, 
			 origin.x = 0, origin.y = 0, origin.z = 0; }				// default constructor
	Line3D(const CVector & V, const Point & p) { direction = V, origin = p; }
	Line3D(const Point & p1, const Point & p2) { direction.Set(p1, p2), origin = p1;} 

	~Line3D() {}
	
	CVector & GetDirection() { return direction; }

	Point & GetPoint() { return origin; }

	Line3D & operator=(const Line3D & L1) { direction.i = L1.direction.i , direction.j = L1.direction.j, direction.k = L1.direction.k;
										origin.x = L1.origin.x, origin.y = L1.origin.y, origin.z= L1.origin.z; return *this; }
	
	// 計算兩線交點
	Point GetIntersectPoint(const Line3D &);

	Point GetOtherP(const float & dis);

};
	 
//==========================================================================================||
// ::::::::: Plane class :::::::::																			 

class Plane
{
//private:
public:	
	CVector Normal;								// the normal of Plane

	float D;									// parameters of Plane (Ax + By + Cz = D) 

public:
	Plane() { Normal.i = 0; Normal.j = 0; Normal.k = 0; D = 0; }
	Plane(const Plane &);						// copy consturctot 
	Plane(const CVector &, const Point &);		// define Plane by: Noraml and a Point
	Plane(const Point &, const Point &, const Point &);	// by: 3 Point
	Plane(const CVector &, const CVector &, const Point &);// by: 2 Vector and 1 point
	Plane(const CVector &, const Point &, const Point &);// by: 1 Vector and 2 point
	~Plane() {}

	CVector & GetNormal()	{ return Normal; }	// 取得平面的法向量
				//回傳Normal參考值(可更改本平面法向量)
	void Set(const CVector &, const Point &);	// Set new Plane by Noraml and a Point
	void Set(const float offset);				// Set Plane by offset
	
//	bool Rotate(const double theta,const Plane & pl, const Point & Ori);	// Rotate Plane
				// 沿著pl平面，將法向量旋轉ang度;法向量須平行平面, Ori為旋轉原點
											
	int isOn(const Point & p);					// 判斷點是否在平面上。
				// 回傳 0： 點在平面上
				// 回傳 1： 點在平面外，法向量正方向。
				// 回傳-1： 點在平面外，法向量負方向。
	bool isUpper(const Point & p)				// 判斷點在平面上方或下方 法向量為正
	{ return p.x*Normal.i + p.y*Normal.j + p.z*Normal.k >= D; } // 點在平面上或上方 都回傳true

	Point GetPoint(const Point &p1, const Point &p2, int & d);	//get Point which on the Plane
				// p1 and p2 方向為 p2 - p1; 參數 d 用來判斷獲得的點是否在p1與p2之間
				// 回傳d = 1:	獲得的點在兩點連線外，方向為正。 
				// 回傳d = -1:	獲得的點在兩點連線外，方向為負。
				// 回傳d = 0:	獲得的點在兩點連線內。 
				// 回傳d = 2:	p1 or p2 在平面上。
				// 回傳-2表示(p2 - p1)與平面平行，無交點。
	Point GetPoint(const Point &p1, const Point &p2); // get Point which on the Plane
				// 不做獲得點的方向判斷。
	Point GetPoint(Line3D &); // get Point which on the Plane

	vector<Point> & StitchPoint(const vector< vector<Point> > & p, int t = 3);	//旗縫法 	
				// find the points cross the Plane(by Points and a threshold)
				// p: 要縫補的點資料，t: 點到平面最短距離
	vector<Point> StitchPoint1(const vector< vector<Point> > & p, int t = 3);	//旗縫法 

	vector<Point> & StitchPoint(const vector<Point> & p, int t = 3);		//旗縫法 

	vector<Point> & StitchPoint(const vector<Point> & p, int t = 3 , float dislimit=6);		//旗縫法
	
	//Point Projection(const Point & p);		// get Projective Point which on this plane
	
	// 獲得在此平面上的投影向量
	CVector GetProjection(const CVector & v)
		{ return v - Normal*v.dot(Normal)/Normal.absV(); }
		
	// get the distance from the point to the Plane
	double dis(const Point & p) 
		{ return (p.x*Normal.i + p.y*Normal.j + p.z*Normal.k - D)/Normal.absV(); }
	
	
	//元睿新增
	void fitting();											//平面擬合功能
	Point Projection(const Point & p);                     //某點投影到此平面	
	std::vector<Point> P_origin;
	
	double a, b, c, d;				//ax+by+cz=d    平面方程式
	double t;					//切平面用 前後距離之門檻值
	std::vector<Point>P;		//切割後點群
	bool acc_cut;				//功能開關   開的話為較精確切層  (預設關)
	//平面擬合使用參數
	std::vector<double>P_ERR;	//各原始點與擬合平面的誤差(距離 含正負)
	double ERR_AVG;				//擬合平均誤差
	double ERR_STD;				//擬合誤差標準差
	double ERR_MAX;				//擬合最大誤差

};														

//============================================================================================
// ::::::::: Cylinder class :::::::::

class Cylinder 
{
public:
	double Radius;								// record radius 
	
	double Angle;								// record angle
	
	double Height;								// record height

public:
	Cylinder(const Point & p, const Point & c);	//define cylidner by 1 point and 1 centroid
	Cylinder() { Radius = 0, Angle = 0, Height= 0; }
	Cylinder(const float r,const float a, const float h) {Radius = r, Angle = a, Height = h; }
	Cylinder(const Cylinder & c) { Radius = c.Radius, Angle = c.Angle, Height = c.Height; }
	~Cylinder() {}
	
	Cylinder & operator=(const Cylinder & c)	// assignment
			{ Radius = c.Radius, Angle = c.Angle, Height = c.Height; return *this; }
	bool RecTransf(const Point & p, const Point & c);//transfer 1 point with 1 centroid to cylinder
	Point TransfPoint(const Point & c);			// transfer cylinder to Point
					// c is centroid
	bool friend CompareAng(const Cylinder & c1, const Cylinder & c2);
};


//=============================================================================================
//	::::::::: Sphere Class :::::::::

class Sphere
{

public:

	double phi;

	double theta;

	double radius;

public:

	Sphere() { phi = 0, theta = 0, radius = 0; }

	Sphere(const double & p, const double & t, const double & r)
		{ phi = p, theta = t, radius = r; }

	Sphere(const Point & p, const Point & c);

	Sphere(const Sphere & S)
		{ phi = S.phi, theta = S.theta, radius = S.radius; }

	~Sphere() {}

	Sphere & operator=(const Sphere & S) 
		{ phi = S.phi, theta = S.theta, radius = S.radius; return *this; }

	Point Transf2Point(const Point & c);	
};


//=============================================================================================
//	::::::::: Sphere Class :::::::::

class Triangulation
{
	public:

		Point First;
		Point Second;
		Point Third;

		CVector Normal;
	
	public:
		Triangulation() { Normal.i = 0; Normal.j = 0; Normal.k = 0; }
		~Triangulation() {}

		friend std::ofstream & operator<<(std::ofstream &, const Triangulation &);
};

#endif
