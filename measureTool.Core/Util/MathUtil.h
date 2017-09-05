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
	*@brief �ݹ鷨������������͹�滯
	*/
	static	bool	ConvexPolygon(const Ogre::Vector3& plnNormal, std::vector<Ogre::Vector3>& pointList);

	/*
	*@brief ����3��������Կռ���з���
	*/
	static	int		RegionalismFromFeaturePoints(const Ogre::Vector3& crothPoint, const Ogre::Vector3& leftPoint, const Ogre::Vector3& rightPoint, const Ogre::Vector3& currPoint);

	/*
	*@brief �����ɵ���ɵĵ㼯������С���˷����й��湹�棺��������f(x,y) = a1*x2+a2*xy+a3*y2+a4*x+a5*y+a6
	*@		ʵ������һ����Ԫ���Իع���С���˽������
	*/
	static bool	CreateQuadricFromPoints(std::vector<double>& coefficient, const std::vector<Ogre::Vector3>& points);

	/*
	*@brief ��˹��Ԫ���� nԪ1�η�����
	*		AΪϵ������xΪ�����������ɹ�������true�����򷵻�false������x��ա�
	*/
	static bool Gauss(const std::vector<std::vector<double>>& A, std::vector<double>& x);
};