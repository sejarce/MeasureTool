#pragma once

#include "OgreVector3.h"
#include "Eigen/Geometry"
#include "pcl/point_types.h"

class	gp_Pnt;
class	gp_Dir;
class	gp_Vec;
class	gp_Quaternion;

class	MathUtil
{
public:

	static Ogre::Vector3 ToOgre(const Eigen::Vector3f& vec);

	static Ogre::Vector3 ToOgre(const pcl::PointXYZ& vec);

	static Ogre::Vector3 ToOgre(const gp_Pnt& pnt);

public:

	static Eigen::Vector3f ToEigen(const Ogre::Vector3& vec);

	static Eigen::Vector3f ToEigen(pcl::PointXYZ& vec);

public:

	static	pcl::PointXYZ	ToPCL(const Ogre::Vector3& vec);

	static	pcl::PointXYZ	ToPCL(const Eigen::Vector3f& vec);

public:

	static	gp_Pnt		ToOCCTPnt(const Ogre::Vector3& vec);

	static	gp_Dir		ToOCCTDir(const Ogre::Vector3& vec);

	static	gp_Vec		ToOCCTVec(const Ogre::Vector3& vec);

	static	gp_Quaternion	ToOCCT(const Ogre::Quaternion& rot);
public:

	static	Ogre::Vector3		ProjectPointOnLine(const Ogre::Vector3& point, const Ogre::Vector3& param1, const Ogre::Vector3& dir);

	static	Eigen::Vector3f		ProjectPointOnLine(const Eigen::Vector3f& point, const Eigen::Vector3f& param1, const Eigen::Vector3f& dir);

	static	Ogre::Vector3		ProjectPointOnPlane(const Ogre::Vector3& point, const Ogre::Plane& pln);

	/*
	*@brief 递归法，将任意多边形凸面化
	*/
	static	bool	ConvexPolygon(const Ogre::Vector3& plnNormal, std::vector<Ogre::Vector3>& pointList);

	/*
	*@brief 根据3个特征点对空间进行分区
	*/
	static	int		RegionalismFromFeaturePoints(const Ogre::Vector3& crothPoint, const Ogre::Vector3& leftPoint, const Ogre::Vector3& rightPoint, const Ogre::Vector3& currPoint);

	/*
	*@brief 对若干点组成的点集采用最小二乘法进行垢面构面：二次曲面f(x,y) = a1*x2+a2*xy+a3*y2+a4*x+a5*y+a6
	*@		实质上是一个多元线性回归最小二乘解的问题
	*/
	static bool	CreateQuadricFromPoints(std::vector<double>& coefficient, const std::vector<Ogre::Vector3>& points);

	/*
	*@brief 高斯消元法解 n元1次方程组
	*		A为系数矩阵，x为解向量，若成功，返回true，否则返回false，并将x清空。
	*/
	static bool Gauss(const std::vector<std::vector<double>>& A, std::vector<double>& x);
};