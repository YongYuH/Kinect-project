#include "stdafx.h"
#include <cmath>
#include <fstream>
#include "Plane.h"
#include "Point2D.h"
#include "matrix_cal.h"
#include<algorithm>
//using namespace std;

//======================================================================
// ::::::::: Point methods :::::::::
std::ifstream & operator >>(std::ifstream & ifs, Point & p)
{
	ifs >> p.x >> p.y >> p.z;
	return ifs;
}

std::ofstream & operator <<(std::ofstream & ofs, const Point & p)
{
	ofs << p.x << " " << p.y << " " << p.z;
	return ofs;
}

bool CompareZ(const Point & p1, const Point & p2)
{
	if(p1.z < p2.z)
		return true;
	else if(p1.z == p2.z && p1.x < p2.x)
		return true;
	else if(p1.z == p2.z && p1.x == p2.x && p1.y < p1.y)
		return true;
	else
		return false;
}
bool CompareZ2(std::vector<const Point> v1, std::vector<const Point> v2)
{
	if (v1[0].z < v2[0].z)
		return true;
	else if(v1[0].z == v2[0].z && v1[0].x < v2[0].x)
		return true;
	else if(v1[0].z == v2[0].z && v1[0].x == v2[0].x && v1[0].y < v2[0].y)
		return true;
	else
		return false;
}
bool CompareX_S2B(const Point & p1, const Point & p2)
{
	if(p1.x < p2.x)
		return true;
	else if(p1.x == p2.x && p1.y < p2.y)
		return true;
	else if(p1.x == p2.x && p1.y == p2.y && p1.z < p1.z)
		return true;
	else
		return false;
}

bool CompareX_B2S(const Point & p1, const Point & p2)
{
	if(p1.x > p2.x)
		return true;
	else if(p1.x == p2.x && p1.y > p2.y)
		return true;
	else if(p1.x == p2.x && p1.y == p2.y && p1.z > p1.z)
		return true;
	else
		return false;
}

bool CompareY_Increase(const Point & p1, const Point & p2)
{
	if(p1.y < p2.y)
		return true;
	else if(p1.y == p2.y && p1.x < p2.x)
		return true;
	else if(p1.y == p2.y && p1.x == p2.x && p1.z < p1.z)
		return true;
	else
		return false;
}

bool CompareY_Decrease(const Point & p1, const Point & p2)
{
	if(p1.y > p2.y)
		return true;
	else if(p1.y == p2.y && p1.x > p2.x)
		return true;
	else if(p1.y == p2.y && p1.x == p2.x && p1.z > p1.z)
		return true;
	else
		return false;
}
//======================================================================
// :::::::::  CVector methods :::::::::

bool CVector::Rotation(const double theta, const Point & Ori, vector<Point> & p)
{
	if(i == 0. && j == 0. && k == 0.)
		return false;
	
	double Rx = i/sqrt(i*i + j*j + k*k);
	if(i == 0)
		Rx = 0.;
	double Ry = j/sqrt(i*i + j*j + k*k);
	if(j == 0)
		Ry = 0.;
	double Rz = k/sqrt(i*i + j*j + k*k);
	if(k == 0)
		Rz = 0.;
	
	double Co = cos(theta);
	double Si = sin(theta);
	double V = 1. - cos(theta);
	
	// rotation matrix
	double RotMatrix[16] = 
		{	Rx*Rx*V + Co,		Rx*Ry*V + Rz*Si,	Rx*Rz*V - Ry*Si,	0,
			Rx*Ry*V - Rz*Si,	Ry*Ry*V + Co,		Ry*Rz*V + Rx*Si,	0,
			Rx*Rz*V + Ry*Si,	Ry*Rz*V - Rx*Si,	Rz*Rz*V + Co,		0,	
			(-1)*Ori.x*(Rx*Rx*V + Co) - Ori.y*(Rx*Ry*V - Rz*Si) - Ori.z*(Rx*Rz*V + Ry*Si) + Ori.x,
			(-1)*Ori.x*(Rx*Ry*V + Rz*Si) - Ori.y*(Ry*Ry*V + Co) - Ori.z*(Ry*Rz*V - Rx*Si) + Ori.y,
			(-1)*Ori.x*(Rx*Rz*V - Ry*Si) - Ori.y*(Ry*Rz*V + Rx*Si) - Ori.z*(Rz*Rz*V + Co) + Ori.z,	1 };
	
	Point tp;
	for(int i = 0; i < p.size(); i++)
	{
		tp = p[i];
		p[i].x = RotMatrix[0]*tp.x + RotMatrix[4]*tp.y + RotMatrix[8]*tp.z	+ RotMatrix[12];
		p[i].y = RotMatrix[1]*tp.x + RotMatrix[5]*tp.y + RotMatrix[9]*tp.z	+ RotMatrix[13];
		p[i].z = RotMatrix[2]*tp.x + RotMatrix[6]*tp.y + RotMatrix[10]*tp.z	+ RotMatrix[14];
	}

	return true;
}

Point CVector::Rotation(const double theta, const Point & Ori, const Point & p)
{
	Point AfterRo;
	if(i == 0. && j == 0. && k == 0.)
		return p;
	
	double Rx = i/sqrt(i*i + j*j + k*k);
	if(i == 0)
		Rx = 0.;
	double Ry = j/sqrt(i*i + j*j + k*k);
	if(j == 0)
		Ry = 0.;
	double Rz = k/sqrt(i*i + j*j + k*k);
	if(k == 0)
		Rz = 0.;
	
	double Co = cos(theta);
	double Si = sin(theta);
	double V = 1. - cos(theta);
	
	// rotation matrix
	double RotMatrix[16] = 
		{	Rx*Rx*V + Co,		Rx*Ry*V + Rz*Si,	Rx*Rz*V - Ry*Si,	0,
			Rx*Ry*V - Rz*Si,	Ry*Ry*V + Co,		Ry*Rz*V + Rx*Si,	0,
			Rx*Rz*V + Ry*Si,	Ry*Rz*V - Rx*Si,	Rz*Rz*V + Co,		0,	
			(-1)*Ori.x*(Rx*Rx*V + Co) - Ori.y*(Rx*Ry*V - Rz*Si) - Ori.z*(Rx*Rz*V + Ry*Si) + Ori.x,
			(-1)*Ori.x*(Rx*Ry*V + Rz*Si) - Ori.y*(Ry*Ry*V + Co) - Ori.z*(Ry*Rz*V - Rx*Si) + Ori.y,
			(-1)*Ori.x*(Rx*Rz*V - Ry*Si) - Ori.y*(Ry*Rz*V + Rx*Si) - Ori.z*(Rz*Rz*V + Co) + Ori.z,	1 };
	
	
	AfterRo.x = RotMatrix[0]*p.x + RotMatrix[4]*p.y + RotMatrix[8]*p.z	+ RotMatrix[12];
	AfterRo.y = RotMatrix[1]*p.x + RotMatrix[5]*p.y + RotMatrix[9]*p.z	+ RotMatrix[13];
	AfterRo.z = RotMatrix[2]*p.x + RotMatrix[6]*p.y + RotMatrix[10]*p.z	+ RotMatrix[14];

	return AfterRo;
}

float * CVector::Rotation(const float theta, const Point & Ori)
{
	double Rx = i/sqrt(i*i + j*j + k*k);
	if(i == 0)
		Rx = 0.;
	double Ry = j/sqrt(i*i + j*j + k*k);
	if(j == 0)
		Ry = 0.;
	double Rz = k/sqrt(i*i + j*j + k*k);
	if(k == 0)
		Rz = 0.;
	
	double Co = cos(theta);
	double Si = sin(theta);
	double V = 1. - cos(theta);
	
	// rotation matrix
	float * RotMatrix = new float [16];
	RotMatrix[0] = Rx*Rx*V + Co;	RotMatrix[1] = Rx*Ry*V + Rz*Si; RotMatrix[2] = Rx*Rz*V - Ry*Si; RotMatrix[3] = 0;
	RotMatrix[4] = Rx*Ry*V - Rz*Si; RotMatrix[5] = Ry*Ry*V + Co;	RotMatrix[6] = Ry*Rz*V + Rx*Si;	RotMatrix[7] = 0;
	RotMatrix[8] = Rx*Rz*V + Ry*Si; RotMatrix[9] = Ry*Rz*V - Rx*Si;	RotMatrix[10] = Rz*Rz*V + Co;	RotMatrix[11] = 0;
	RotMatrix[12] = (-1)*Ori.x*(Rx*Rx*V + Co) - Ori.y*(Rx*Ry*V - Rz*Si) - Ori.z*(Rx*Rz*V + Ry*Si) + Ori.x; 
	RotMatrix[13] = (-1)*Ori.x*(Rx*Ry*V + Rz*Si) - Ori.y*(Ry*Ry*V + Co) - Ori.z*(Ry*Rz*V - Rx*Si) + Ori.y; 
	RotMatrix[14] = (-1)*Ori.x*(Rx*Rz*V - Ry*Si) - Ori.y*(Ry*Rz*V + Rx*Si) - Ori.z*(Rz*Rz*V + Co) + Ori.z; 
	RotMatrix[15] = 1;
	
	return RotMatrix;
}

CVector CVector::Rotation(const float theta, const Point & Ori, const CVector & v1)
{
	if(i == 0. && j == 0. && k == 0.)
		return v1;
	
	double Rx = i/sqrt(i*i + j*j + k*k);
	if(i == 0)
		Rx = 0.;
	double Ry = j/sqrt(i*i + j*j + k*k);
	if(j == 0)
		Ry = 0.;
	double Rz = k/sqrt(i*i + j*j + k*k);
	if(k == 0)
		Rz = 0.;
	
	double Co = cos(theta);
	double Si = sin(theta);
	double V = 1. - cos(theta);
	
	// rotation matrix
	double RotMatrix[16] = 
		{	Rx*Rx*V + Co,		Rx*Ry*V + Rz*Si,	Rx*Rz*V - Ry*Si,	0,
			Rx*Ry*V - Rz*Si,	Ry*Ry*V + Co,		Ry*Rz*V + Rx*Si,	0,
			Rx*Rz*V + Ry*Si,	Ry*Rz*V - Rx*Si,	Rz*Rz*V + Co,		0,	
			(-1)*Ori.x*(Rx*Rx*V + Co) - Ori.y*(Rx*Ry*V - Rz*Si) - Ori.z*(Rx*Rz*V + Ry*Si) + Ori.x,
			(-1)*Ori.x*(Rx*Ry*V + Rz*Si) - Ori.y*(Ry*Ry*V + Co) - Ori.z*(Ry*Rz*V - Rx*Si) + Ori.y,
			(-1)*Ori.x*(Rx*Rz*V - Ry*Si) - Ori.y*(Ry*Rz*V + Rx*Si) - Ori.z*(Rz*Rz*V + Co) + Ori.z,	1 };
	
	
	Point p(v1.i + Ori.x, v1.j + Ori.y, v1.k + Ori.z);
	Point AfterRo;

	AfterRo.x = RotMatrix[0]*p.x + RotMatrix[4]*p.y + RotMatrix[8]*p.z	+ RotMatrix[12];
	AfterRo.y = RotMatrix[1]*p.x + RotMatrix[5]*p.y + RotMatrix[9]*p.z	+ RotMatrix[13];
	AfterRo.z = RotMatrix[2]*p.x + RotMatrix[6]*p.y + RotMatrix[10]*p.z	+ RotMatrix[14];

	CVector AfterRoV(AfterRo.x - Ori.x, AfterRo.y - Ori.y, AfterRo.z - Ori.z);

	return AfterRoV;
}

//**********************************
// ::::::::: Line mothods ::::::::::
Point Line3D::GetIntersectPoint(const Line3D & L1)
{
	Point IntersectP;
	double t = ( (L1.direction.i*(origin.y - L1.origin.y) - L1.direction.j*(origin.x - L1.origin.x))
				/(direction.i*L1.direction.j - direction.j*L1.direction.i) );
	
	IntersectP.x = direction.i*t + origin.x;
	IntersectP.y = direction.j*t + origin.y;
	IntersectP.z = direction.k*t + origin.z;

	return IntersectP;
}

Point Line3D::GetOtherP(const float & dis)
{
	Point OtherP;
	
	OtherP.x = dis*direction.i + origin.x;
	OtherP.y = dis*direction.j + origin.y;
	OtherP.z = dis*direction.k + origin.z;

	return OtherP;
}

//======================================================================
// ::::::::: Plane constructor :::::::::

Plane::Plane(const Plane & pl)
{
	Normal = pl.Normal; D = pl.D;
}

Plane::Plane(const CVector & normal1, const Point & p1)
{
	Normal = normal1;
	D = p1.x*Normal.i + p1.y*Normal.j + p1.z*Normal.k;
}

Plane::Plane(const Point & p1, const Point & p2, const Point & p3)
{
	CVector v1(p1, p2);
	CVector v2(p1, p3);

	Normal = v1.cross(v2);
	Normal = Normal/ Normal.absV();

	D = p1.x*Normal.i + p1.y*Normal.j + p1.z*Normal.k;
}

Plane::Plane(const CVector & v1, const CVector & v2, const Point & p)
{
	Normal = v1.cross(v2);
	Normal = Normal/ Normal.absV();

	D = p.x*Normal.i + p.y*Normal.j + p.z*Normal.k;
}

Plane::Plane(const CVector & v, const Point & p1, const Point & p2)
{
	CVector v2(p1, p2);
	
	Normal = v.cross(v2);
	Normal = Normal/ Normal.absV();

	D = p1.x*Normal.i + p1.y*Normal.j + p1.z*Normal.k;
}

//**********************************
// ::::::::: Plane methods :::::::::

void Plane::Set(const float offset)
{
	D += offset*Normal.absV();	// =>Dn - Do = offset * |Normal| => Dn = offset * |Normal| + Do
}


void Plane::Set(const CVector & normal1, const Point & p)
{
	Normal = normal1;
	D = p.x*Normal.i + p.y*Normal.j + p.z*Normal.k; 
}

int Plane::isOn(const Point & p)
{
	float td = p.x*Normal.i + p.y*Normal.j + p.z*Normal.k;
	if(td > D)
		return 1;
	else if(td < D)
		return -1;
	else
		return 0;
}

Point Plane::GetPoint(const Point & p1, const Point & p2, int & d)
{
	Point OnP;
	CVector v(p1, p2);
	if(Normal.dot(v) == 0)			// p1 與 p2 連成的線與平面平行
	{
		d = -2;
		return OnP;
	}

	float td1, td2;
	td1 = p1.x*Normal.i + p1.y*Normal.j + p1.z*Normal.k;
	td2 = p2.x*Normal.i + p2.y*Normal.j + p2.z*Normal.k;
	if(td1 == D)					// p1 on the plane
	{
		d = 2;
		return p1;					
	}

	if(td2 == D)					// p2 on the plane
	{
		d = 2;
		return p2;
	}

	float t = (D - p1.x*Normal.i - p1.y*Normal.j - p1.z*Normal.k)/v.dot(Normal);
	OnP.x = p1.x + t*v.i;
	OnP.y = p1.y + t*v.j;
	OnP.z = p1.z + t*v.k;

	if(td1 * td2 < 0)				// 平面在兩點間
		d = 0;
	if (td1 * td2 > 0)				// 平面在兩點連線外
	{	
		CVector tv(OnP, p1);
		if(v.dot(tv) > 0)			// 平面在(p2 - p1)正方向	
			d = 1;
		else						// 平面在(p2 - p1)負方向
			d = -1;
	}
	return OnP;
}

Point Plane::GetPoint(const Point & p1, const Point & p2)
{
	CVector v(p1, p2);
	float t = (D - p1.x*Normal.i - p1.y*Normal.j - p1.z*Normal.k)/v.dot(Normal);
	
	Point c;
	c.x = p1.x + t*v.i;
	c.y = p1.y + t*v.j;
	c.z = p1.z + t*v.k;
	
	return c;
}

Point Plane::GetPoint(Line3D & L)
{
	const CVector v = L.GetDirection();
	const Point p = L.GetPoint();

	float t = (D - p.x*Normal.i - p.y*Normal.j - p.z*Normal.k)/v.dot(Normal);
	
	Point c;
	c.x = p.x + t*v.i;
	c.y = p.y + t*v.j;
	c.z = p.z + t*v.k;
	
	return c;
}


vector<Point> & Plane::StitchPoint(const vector< vector<Point> > & Origin, int t)
{
	Plane upperPlane(*this);		// upper Plane: offset (t)
	Plane lowerPlane(*this);		// lower Plane: offset (-t)
	
	upperPlane.Set(t);				
	lowerPlane.Set(-t);

	vector<Point> upperP;			// upper points
	vector<Point> lowerP;			// lower points

	Point up, low;
	
	for(int i = 0; i < Origin.size(); i++)
		for(int j = 0; j < Origin[i].size(); j++)
		{
			//若是在upper Plane 與本平面中間的點
			if( !upperPlane.isUpper(Origin[i][j]) &&  isUpper(Origin[i][j]) )
			{
				up = Origin[i][j];		
				upperP.push_back(up);
			}
			
			// 若是在本平面與lower Plane 中間的點
			if( !isUpper(Origin[i][j]) && lowerPlane.isUpper(Origin[i][j]) )
			{
				low = Origin[i][j];
				lowerP.push_back(low);
			}
		}

	vector<Point> * P_OnPlane = new vector<Point>;

	if( upperP.size() == 0 || lowerP.size() == 0 )	// 如果upperP或lowerP其中之一沒有點的話
		return *P_OnPlane;							// return an empty vector<Point>

	float distance, t_dis;
	Point tempP;
	
	// if the size of the upperP >= lowerP (以 upperP 的點為基準)
	if( upperP.size() >= lowerP.size() )
		for(int i = 0; i < upperP.size(); i++)
		{
			CVector td(upperP[i], lowerP[0]);
			distance = td.absV();

			for(int j = 0; j < lowerP.size(); j++)
			{
				CVector td(upperP[i], lowerP[j]);
				t_dis = td.absV();
				
				if( distance > t_dis )	// 找與upperP[i] 距離最短的點
				{
					distance = t_dis;
					tempP = lowerP[j];
				}
			}

			P_OnPlane->push_back( GetPoint(upperP[i], tempP) );
		}
	
	// if the size of the lowerP > upperP (以lowerP 的點為基準)
	else
	{
		for(int i = 0; i < lowerP.size(); i++)
		{
			CVector td(lowerP[i], upperP[0]);
			distance = td.absV();

			for(int j = 0; j < upperP.size(); j++)
			{
				CVector td(lowerP[i], upperP[j]);
				t_dis = td.absV();
				
				if( distance > t_dis )	// 找與lowerP[i]距離最短的點
				{
					distance = t_dis;
					tempP = upperP[j];
				}
			}
			P_OnPlane->push_back( GetPoint(lowerP[i], tempP) );
		}
	}
	
	return *P_OnPlane;
}

vector<Point> Plane::StitchPoint1(const vector< vector<Point> > & Origin, int t)
{
	Plane upperPlane(*this);		// upper Plane: offset (t)
	Plane lowerPlane(*this);		// lower Plane: offset (-t)
	
	upperPlane.Set(t);				
	lowerPlane.Set(-t);

	vector<Point> upperP;			// upper points
	vector<Point> lowerP;			// lower points

	Point up, low;
	
	for(int i = 0; i < Origin.size(); i++)
		for(int j = 0; j < Origin[i].size(); j++)
		{
			//若是在upper Plane 與本平面中間的點
			if( !upperPlane.isUpper(Origin[i][j]) &&  isUpper(Origin[i][j]) )
			{
				up = Origin[i][j];		
				upperP.push_back(up);
			}
			
			// 若是在本平面與lower Plane 中間的點
			if( !isUpper(Origin[i][j]) && lowerPlane.isUpper(Origin[i][j]) )
			{
				low = Origin[i][j];
				lowerP.push_back(low);
			}
		}

	vector<Point> P_OnPlane;

	if( upperP.size() == 0 || lowerP.size() == 0 )	// 如果upperP或lowerP其中之一沒有點的話
		return P_OnPlane;							// return an empty vector<Point>

	float distance, t_dis;
	Point tempP;
	
	// if the size of the upperP >= lowerP (以 upperP 的點為基準)
	if( upperP.size() >= lowerP.size() )
		for(int i = 0; i < upperP.size(); i++)
		{
			CVector td(upperP[i], lowerP[0]);
			distance = td.absV();

			for(int j = 0; j < lowerP.size(); j++)
			{
				CVector td(upperP[i], lowerP[j]);
				t_dis = td.absV();
				
				if( distance > t_dis )	// 找與upperP[i] 距離最短的點
				{
					distance = t_dis;
					tempP = lowerP[j];
				}
			}

			P_OnPlane.push_back( GetPoint(upperP[i], tempP) );
		}
	
	// if the size of the lowerP > upperP (以lowerP 的點為基準)
	else 
	{
		for(int i = 0; i < lowerP.size(); i++)
		{
			CVector td(lowerP[i], upperP[0]);
			distance = td.absV();

			for(int j = 0; j < upperP.size(); j++)
			{
				CVector td(lowerP[i], upperP[j]);
				t_dis = td.absV();
				
				if( distance > t_dis )	// 找與lowerP[i]距離最短的點
				{
					distance = t_dis;
					tempP = upperP[j];
				}
			}
			P_OnPlane.push_back( GetPoint(lowerP[i], tempP) );
		}
	}
	
	return P_OnPlane;
}

vector<Point> & Plane::StitchPoint(const vector<Point> & Origin, int t)
{
	Plane upperPlane(*this);		// upper Plane: offset (t)
	Plane lowerPlane(*this);		// lower Plane: offset (-t)
	
	upperPlane.Set(t);				
	lowerPlane.Set(-t);

	vector<Point> upperP;			// upper points
	vector<Point> lowerP;			// lower points

	Point up, low;
	
	for(int i = 0; i < Origin.size(); i++)
	{
		{
			//若是在upper Plane 與本平面中間的點
			if( !upperPlane.isUpper(Origin[i]) && isUpper(Origin[i]) )
			{
				up = Origin[i];		
				upperP.push_back(up);
			}
			
			// 若是在本平面與lower Plane 中間的點
			if( !isUpper(Origin[i]) && lowerPlane.isUpper(Origin[i]) )
			{
				low = Origin[i];
				lowerP.push_back(low);
			}
		}
	}

	vector<Point> * P_OnPlane = new vector<Point>;

	if( upperP.size() == 0 || lowerP.size() == 0 )	// 如果upperP或lowerP其中之一沒有點的話
		return *P_OnPlane;							// return an empty vector<Point>

	float distance, t_dis;
	Point tempP;
	
	// if the size of the upperP >= lowerP (以 upperP 的點為基準)
	if( upperP.size() >= lowerP.size() )
		for(int i = 0; i < upperP.size(); i++)
		{
			CVector td(upperP[i], lowerP[0]);
			distance = td.absV();
			tempP = lowerP[0];

			for(int j = 0; j < lowerP.size(); j++)
			{
				CVector td(upperP[i], lowerP[j]);
				t_dis = td.absV();
				
				if( distance > t_dis )	// 找與upperP[i] 距離最短的點
				{
					distance = t_dis;
					tempP = lowerP[j];
				}
			}

			P_OnPlane->push_back( GetPoint(upperP[i], tempP) );
		}
	
	// if the size of the lowerP > upperP (以lowerP 的點為基準)
	else
	{
		for(int i = 0; i < lowerP.size(); i++)
		{
			CVector td(lowerP[i], upperP[0]);
			distance = td.absV();
			tempP = upperP[0];

			for(int j = 0; j < upperP.size(); j++)
			{
				CVector td(lowerP[i], upperP[j]);
				t_dis = td.absV();
				
				if( distance > t_dis )	// 找與lowerP[i]距離最短的點
				{
					distance = t_dis;
					tempP = upperP[j];
				}
			}

			P_OnPlane->push_back( GetPoint(lowerP[i], tempP) );
		}
	}
	
	return *P_OnPlane;
}

vector<Point> & Plane::StitchPoint(const vector<Point> & Origin, int t , float DisC)
{
	Plane upperPlane(*this);		// upper Plane: offset (t)
	Plane lowerPlane(*this);		// lower Plane: offset (-t)
	
	upperPlane.Set(t);				
	lowerPlane.Set(-t);

	vector<Point> upperP;			// upper points
	vector<Point> lowerP;			// lower points

	Point up, low;
	
	for(int i = 0; i < Origin.size(); i++)
	{
		{
			//若是在upper Plane 與本平面中間的點
			if( !upperPlane.isUpper(Origin[i]) && isUpper(Origin[i]) )
			{
				up = Origin[i];		
				upperP.push_back(up);
			}
			
			// 若是在本平面與lower Plane 中間的點
			if( !isUpper(Origin[i]) && lowerPlane.isUpper(Origin[i]) )
			{
				low = Origin[i];
				lowerP.push_back(low);
			}
		}
	}

	vector<Point> * P_OnPlane = new vector<Point>;

	if( upperP.size() == 0 || lowerP.size() == 0 )	// 如果upperP或lowerP其中之一沒有點的話
		return *P_OnPlane;							// return an empty vector<Point>

	float distance, t_dis;
	Point tempP;
	
	// if the size of the upperP >= lowerP (以 upperP 的點為基準)
	if( upperP.size() >= lowerP.size() )
		for(int i = 0; i < upperP.size(); i++)
		{
			CVector td(upperP[i], lowerP[0]);
			distance = td.absV();
			tempP = lowerP[0];

			for(int j = 0; j < lowerP.size(); j++)
			{
				CVector td(upperP[i], lowerP[j]);
				t_dis = td.absV();
				
				if( distance > t_dis )	// 找與upperP[i] 距離最短的點
				{
					distance = t_dis;
					tempP = lowerP[j];
				}
			}
			if(distance < DisC)
				P_OnPlane->push_back( GetPoint(upperP[i], tempP) );
		}
	
	// if the size of the lowerP > upperP (以lowerP 的點為基準)
	else
	{
		for(int i = 0; i < lowerP.size(); i++)
		{
			CVector td(lowerP[i], upperP[0]);
			distance = td.absV();
			tempP = upperP[0];

			for(int j = 0; j < upperP.size(); j++)
			{
				CVector td(lowerP[i], upperP[j]);
				t_dis = td.absV();
				
				if( distance > t_dis )	// 找與lowerP[i]距離最短的點
				{
					distance = t_dis;
					tempP = upperP[j];
				}
			}
			if(distance < DisC)
				P_OnPlane->push_back( GetPoint(lowerP[i], tempP) );
		}
	}
	
	return *P_OnPlane;
}
// ============================================================================================
// ::::::::: Cylinder methods ::::::::::

Cylinder::Cylinder(const Point & p, const Point & c)
{
	float tx, ty;
	tx = p.x - c.x;
	ty = p.y - c.y;
	Radius = sqrt( tx*tx + ty*ty );
	Angle = atan2(ty, tx)*180/PI;
	Height = p.z;
}

bool Cylinder::RecTransf(const Point & p, const Point & c)
{
	if(c == p || c.z != p.z) 
		return false;
	float tx, ty;
	tx = p.x - c.x;
	ty = p.y - c.y;
	Radius = sqrt( tx*tx + ty*ty );
	Angle = atan2(ty, tx)*180/PI;
	Height = p.z;

	return true;
}

Point Cylinder::TransfPoint(const Point & c)
{
	Point p;
	double theta = Angle / 180 * PI;
	p.x = Radius * cos(theta) + c.x;
	p.y = Radius * sin(theta) + c.y;
	p.z = c.z;

	return p;
}

bool CompareAng(const Cylinder & c1, const Cylinder & c2)
{
	if(c1.Angle < c2.Angle)
		return true;
	else if(c1.Angle == c2.Angle && c1.Radius < c2.Radius)
		return true;
	else
		return false;
}

// ============================================================================================
// ::::::::: Sphere methods ::::::::::

Point Sphere::Transf2Point(const Point & Cen)
{
	Point P;
	
	double ratio = PI / 180.;
	
	double theta1 = this->theta*ratio;
	double phi1 = this->phi*ratio;

	P.x = radius*sin(theta1)*cos(phi1) + Cen.x;
	P.y = radius*sin(theta1)*sin(phi1) + Cen.y;
	P.z = radius*cos(theta1 + PI ) + Cen.z;
	
	return P;
}

// ============================================================================================
// ::::::::: Triangulation methods ::::::::::

std::ofstream & operator <<(std::ofstream & ofs, const Triangulation & Tri)
{
	ofs << "   facet normal " << Tri.Normal.i << " " << Tri.Normal.j
		<< " "<< Tri.Normal.k << std::endl

		<< "\t outer loop" << std::endl
		<< "\t    vertex " << Tri.First.x << " " << Tri.First.y << " " << Tri.First.z << std::endl
		<< "\t    vertex " << Tri.Second.x << " " << Tri.Second.y << " " << Tri.Second.z << std::endl
		<< "\t    vertex " << Tri.Third.x << " " << Tri.Third.y << " " << Tri.Third.z<< std::endl

		<< "\t endloop" << std::endl
		<< "   endfacet" << std::endl;

	return ofs;
}

void CVector::normalize() // Pirson
{
	double abs = this->absV();
	if( abs == 0)
		return;

	this-> i /= abs;
	this-> j /= abs;
	this-> k /= abs;

	return;
}

void Plane::fitting()
{
	int Number = P_origin.size();
	double **arr, **rlt;				//Pseudo-Inverse所需要用到的矩陣
	if (Number == 0)
		return;

	//設定Pseudo-Inverse所需的動態陣列
	arr = new double *[Number];
	rlt = new double *[Number];
	for (int i = 0; i<Number; i++)
	{
		arr[i] = new double[3];
		rlt[i] = new double[1];
	}
	double a1, b1, c1, d1;
	double a2, b2, c2, d2;
	double a3, b3, c3, d3;
	double a4, b4, c4, d4;
	std::vector<double>P_ERR_TEMP1, P_ERR_TEMP2, P_ERR_TEMP3, P_ERR_TEMP4;
	std::vector<double>ERR_AVG_TEMP(4);
	std::vector<double>ERR_S_TEMP(4);
	//////第一次擬合
	//傳值
	for (int i = 0; i<Number; i++)
	{
		arr[i][0] = P_origin[i].x;
		arr[i][1] = P_origin[i].y;
		arr[i][2] = P_origin[i].z;
		rlt[i][0] = 1.0;
	}
	//Pseudo-Inverse
	matrixA ar(Number, 3, arr);
	matrixA tm1(3, Number);
	matrixA tm2(3, 3);
	matrixA tm3(3, 3);
	matrixA tm4(3, Number);
	matrixA r1(Number, 1, rlt);
	matrixA sol(3, 1);
	tm1 = ar.Transpose();
	tm2 = tm1*ar;
	tm3 = tm2.Inverse();
	tm4 = tm3*tm1;
	sol = tm4*r1;
	a = sol.arr[0][0];
	b = sol.arr[1][0];
	c = sol.arr[2][0];
	d = 1.0;
	//正規化
	double nom_temp = sqrt((a*a + b*b + c*c));
	a /= nom_temp;
	b /= nom_temp;
	c /= nom_temp;
	d /= nom_temp;
	a1 = a;
	b1 = b;
	c1 = c;
	d1 = d;
	//誤差計算
	P_ERR.resize(Number);
	ERR_AVG = 0.;				//擬合平均誤差
	ERR_STD = 0.;				//擬合誤差標準差
	ERR_MAX = 0.;				//擬合最大誤差
	//計算平均誤差
	for (int i = 0; i<Number; i++)
	{
		P_ERR[i] = dis(P_origin[i]);
		ERR_AVG += (P_ERR[i]);
	}
	ERR_AVG /= Number;
	//計算標準差
	for (int i = 0; i<Number; i++)
		ERR_STD += ((P_ERR[i] - (ERR_AVG))*(P_ERR[i] - (ERR_AVG)));
	ERR_STD = sqrt(ERR_STD);
	ERR_STD /= Number;
	ERR_AVG_TEMP[0] = ERR_AVG;
	ERR_S_TEMP[0] = ERR_STD;
	P_ERR_TEMP1 = P_ERR;
	/////////////第二次擬合
	for (int i = 0; i<Number; i++)
	{
		arr[i][0] = P_origin[i].y;
		arr[i][1] = P_origin[i].z;
		arr[i][2] = -1.0;
		rlt[i][0] = -P_origin[i].x;
	}
	//Pseudo-Inverse
	ar.Initial(Number, 3, arr);
	r1.Initial(Number, 1, rlt);
	tm1 = ar.Transpose();
	tm2 = tm1*ar;
	tm3 = tm2.Inverse();
	tm4 = tm3*tm1;
	sol = tm4*r1;
	a = 1.0;
	b = sol.arr[0][0];
	c = sol.arr[1][0];
	d = sol.arr[2][0];
	//正規化
	nom_temp = sqrt((a*a + b*b + c*c));
	a /= nom_temp;
	b /= nom_temp;
	c /= nom_temp;
	d /= nom_temp;
	a2 = a;
	b2 = b;
	c2 = c;
	d2 = d;
	//誤差計算
	P_ERR.resize(Number);
	ERR_AVG = 0.;				//擬合平均誤差
	ERR_STD = 0.;				//擬合誤差標準差
	ERR_MAX = 0.;				//擬合最大誤差
	//計算平均誤差
	for (int i = 0; i<Number; i++)
	{
		P_ERR[i] = dis(P_origin[i]);
		ERR_AVG += (P_ERR[i]);
	}
	ERR_AVG /= Number;
	//計算標準差
	for (int i = 0; i<Number; i++)
		ERR_STD += ((P_ERR[i] - (ERR_AVG))*(P_ERR[i] - (ERR_AVG)));
	ERR_STD = sqrt(ERR_STD);
	ERR_STD /= Number;
	ERR_AVG_TEMP[1] = ERR_AVG;
	ERR_S_TEMP[1] = ERR_STD;
	P_ERR_TEMP2 = P_ERR;
	/////////////第三次擬合
	for (int i = 0; i<Number; i++)
	{
		arr[i][0] = P_origin[i].x;
		arr[i][1] = P_origin[i].z;
		arr[i][2] = -1.0;
		rlt[i][0] = -P_origin[i].y;
	}
	//Pseudo-Inverse
	ar.Initial(Number, 3, arr);
	r1.Initial(Number, 1, rlt);
	tm1 = ar.Transpose();
	tm2 = tm1*ar;
	tm3 = tm2.Inverse();
	tm4 = tm3*tm1;
	sol = tm4*r1;
	a = sol.arr[0][0];
	b = 1.0;
	c = sol.arr[1][0];
	d = sol.arr[2][0];
	//正規化
	nom_temp = sqrt((a*a + b*b + c*c));
	a /= nom_temp;
	b /= nom_temp;
	c /= nom_temp;
	d /= nom_temp;
	a3 = a;
	b3 = b;
	c3 = c;
	d3 = d;
	//誤差計算
	P_ERR.resize(Number);
	ERR_AVG = 0.;				//擬合平均誤差
	ERR_STD = 0.;				//擬合誤差標準差
	ERR_MAX = 0.;				//擬合最大誤差
	//計算平均誤差
	for (int i = 0; i<Number; i++)
	{
		P_ERR[i] = dis(P_origin[i]);
		ERR_AVG += (P_ERR[i]);
	}
	ERR_AVG /= Number;
	//計算標準差
	for (int i = 0; i<Number; i++)
		ERR_STD += ((P_ERR[i] - (ERR_AVG))*(P_ERR[i] - (ERR_AVG)));
	ERR_STD = sqrt(ERR_STD);
	ERR_STD /= Number;
	ERR_AVG_TEMP[2] = ERR_AVG;
	ERR_S_TEMP[2] = ERR_STD;
	P_ERR_TEMP3 = P_ERR;
	/////////////第四次擬合
	for (int i = 0; i<Number; i++)
	{
		arr[i][0] = P_origin[i].x;
		arr[i][1] = P_origin[i].y;
		arr[i][2] = -1.0;
		rlt[i][0] = -P_origin[i].z;
	}
	//Pseudo-Inverse
	ar.Initial(Number, 3, arr);
	r1.Initial(Number, 1, rlt);
	tm1 = ar.Transpose();
	tm2 = tm1*ar;
	tm3 = tm2.Inverse();
	tm4 = tm3*tm1;
	sol = tm4*r1;
	a = sol.arr[0][0];
	b = sol.arr[1][0];
	c = 1.0;
	d = sol.arr[2][0];
	//正規化
	nom_temp = sqrt((a*a + b*b + c*c));
	a /= nom_temp;
	b /= nom_temp;
	c /= nom_temp;
	d /= nom_temp;
	a4 = a;
	b4 = b;
	c4 = c;
	d4 = d;
	//誤差計算
	P_ERR.resize(Number);
	ERR_AVG = 0.;				//擬合平均誤差
	ERR_STD = 0.;				//擬合誤差標準差
	ERR_MAX = 0.;				//擬合最大誤差
	//計算平均誤差
	for (int i = 0; i<Number; i++)
	{
		P_ERR[i] = dis(P_origin[i]);
		ERR_AVG += (P_ERR[i]);
	}
	ERR_AVG /= Number;
	//計算標準差
	for (int i = 0; i<Number; i++)
		ERR_STD += ((P_ERR[i] - (ERR_AVG))*(P_ERR[i] - (ERR_AVG)));
	ERR_STD = sqrt(ERR_STD);
	ERR_STD /= Number;
	ERR_AVG_TEMP[3] = ERR_AVG;
	ERR_S_TEMP[3] = ERR_STD;
	P_ERR_TEMP4 = P_ERR;

	std::vector<double>ERR_S_TEMP_S = ERR_S_TEMP;
	std::sort(ERR_S_TEMP_S.begin(), ERR_S_TEMP_S.end());
	if (ERR_S_TEMP_S[0] == ERR_S_TEMP[0])//表第一組擬合誤差最小
	{
		a = a1;
		b = b1;
		c = c1;
		d = d1;
		P_ERR = P_ERR_TEMP1;
		ERR_AVG = ERR_AVG_TEMP[0];
		ERR_STD = ERR_S_TEMP[0];
	}
	else if (ERR_S_TEMP_S[0] == ERR_S_TEMP[1])//表第二組擬合誤差最小
	{
		a = a2;
		b = b2;
		c = c2;
		d = d2;
		P_ERR = P_ERR_TEMP2;
		ERR_AVG = ERR_AVG_TEMP[1];
		ERR_STD = ERR_S_TEMP[1];
	}
	else if (ERR_S_TEMP_S[0] == ERR_S_TEMP[2])//表第三組擬合誤差最小
	{
		a = a3;
		b = b3;
		c = c3;
		d = d3;
		P_ERR = P_ERR_TEMP3;
		ERR_AVG = ERR_AVG_TEMP[2];
		ERR_STD = ERR_S_TEMP[2];
	}
	else if (ERR_S_TEMP_S[0] == ERR_S_TEMP[3])//表第四組擬合誤差最小
	{
		a = a4;
		b = b4;
		c = c4;
		d = d4;
		P_ERR = P_ERR_TEMP4;
		ERR_AVG = ERR_AVG_TEMP[3];
		ERR_STD = ERR_S_TEMP[3];
	}

	//計算最大誤差
	/*ERR_MAX = fabs(tool_fun::a_min(P_ERR));
	double err_temp = tool_fun::a_max(P_ERR);
	if (err_temp > ERR_MAX)
		ERR_MAX = err_temp;*/


	for (int i = 0; i<Number; i++)
	{
		delete[] arr[i];
		delete[] rlt[i];
	}
	delete[] arr;
	delete[] rlt;
	return;
}

Point Plane::Projection(const Point & p)
{
	Point ProjectiveP;

	double del = a*p.x + b*p.y + c*p.z - d;
	if (del == 0)	// if p on this plane
		return p;

	double n = a*a + b*b + c*c;
	if (n == 0)		// if plane is null
		return ProjectiveP;

	double t = del / n;

	ProjectiveP.x = p.x - t*a;
	ProjectiveP.y = p.y - t*b;
	ProjectiveP.z = p.z - t*c;

	return ProjectiveP;
}
