#include "MathUtil.h"

#include "Ogre.h"

#include "gp_Pnt.hxx"
#include "gp_Dir.hxx"
#include "gp_Quaternion.hxx"

Ogre::Vector3 MathUtil::ToOgre(const Eigen::Vector3f& vec)
{
	return{ vec[0], vec[1], vec[2] };
}

Ogre::Vector3 MathUtil::ToOgre(const pcl::PointXYZ& vec)
{
	return{ vec.data[0], vec.data[1], vec.data[2] };
}

Ogre::Vector3 MathUtil::ToOgre(const gp_Pnt& pnt)
{
	return{ static_cast<Ogre::Real>( pnt.X() ), static_cast<Ogre::Real>( pnt.Y() ), static_cast<Ogre::Real>( pnt.Z() ) };
}

Eigen::Vector3f MathUtil::ToEigen(const Ogre::Vector3& vec)
{
	return{ vec[0], vec[1], vec[2] };
}

Eigen::Vector3f MathUtil::ToEigen(pcl::PointXYZ& vec)
{
	return vec.getVector3fMap();
}

pcl::PointXYZ MathUtil::ToPCL(const Ogre::Vector3& vec)
{
	return{ vec[0], vec[1], vec[2] };
}

pcl::PointXYZ MathUtil::ToPCL(const Eigen::Vector3f& vec)
{
	return{ vec[0], vec[1], vec[2] };
}

gp_Pnt MathUtil::ToOCCTPnt(const Ogre::Vector3& vec)
{
	return{ vec.x, vec.y, vec.z };
}

gp_Dir MathUtil::ToOCCTDir(const Ogre::Vector3& vec)
{
	return{ vec.x, vec.y, vec.z };
}

gp_Vec MathUtil::ToOCCTVec(const Ogre::Vector3& vec)
{
	return{ vec.x, vec.y, vec.z };
}

gp_Quaternion MathUtil::ToOCCT(const Ogre::Quaternion& rot)
{
	return{ rot.x, rot.y, rot.z, rot.w };
}

Ogre::Vector3 MathUtil::ProjectPointOnLine(const Ogre::Vector3& point, const Ogre::Vector3& param1, const Ogre::Vector3& dir)
{
	auto vecA = point - param1;
	
	auto dis = vecA.dotProduct(dir);

	auto pc = param1 + dir * dis;

	return pc;
}

Eigen::Vector3f MathUtil::ProjectPointOnLine(const Eigen::Vector3f& point, const Eigen::Vector3f& param1, const Eigen::Vector3f& dir)
{
	auto vecA = point - param1;

	auto dis = vecA.dot(dir);

	auto pc = param1 + dir * dis;

	return pc;
}

Ogre::Vector3 MathUtil::ProjectPointOnPlane(const Ogre::Vector3& point, const Ogre::Plane& pln)
{
	auto plnOrigin = -pln.normal * pln.d;

	auto plnToTarget = point - plnOrigin;
	auto dis = plnToTarget.dotProduct(pln.normal);

	return point - pln.normal * dis;
}


bool MathUtil::ConvexPolygon(const Ogre::Vector3& plnNormal, std::vector<Ogre::Vector3>& pointList)
{
	if (pointList.size() < 3)
		return false;

	std::vector<Ogre::Vector3> tempList;
	tempList.reserve(pointList.size());
	for (auto cur : pointList)
	{
		tempList.emplace_back(cur);
	}

	pointList.clear();
	for (int i = 0; i < tempList.size(); i++)
	{
		auto curPos = tempList[i];
		auto index = (i - 1) >= 0 ? (i - 1) : (tempList.size() + i - 1);
		auto lastPos = tempList[index];
		index = (i + 1) < tempList.size() ? (i + 1) : (i + 1 - tempList.size());
		auto nextPos = tempList[index];

		auto last_cur = curPos - lastPos;
		auto nor = last_cur.crossProduct(plnNormal);
		Ogre::Plane plane(nor, curPos);
		auto f = nor.dotProduct(nextPos) + plane.d;
		if (f < 0)
		{
			pointList.emplace_back(curPos);
		}
	}

	if (pointList.size() != tempList.size())
	{
		tempList.clear();
		std::vector<Ogre::Vector3>().swap(tempList);
		return ConvexPolygon(plnNormal, pointList);
	}
	return true;
}

int MathUtil::RegionalismFromFeaturePoints(const Ogre::Vector3& crothPoint, const Ogre::Vector3& leftPoint, const Ogre::Vector3& rightPoint, const Ogre::Vector3& currPoint)
{
	//上半身
	if (currPoint.y > crothPoint.y && currPoint.x > leftPoint.x && currPoint.x < rightPoint.x )
		return 0;
	//左手 或 一部分左脚
	if (currPoint.x < leftPoint.x)
		return -1;
	//左脚 或 一部分左手
	if (currPoint.y < crothPoint.y && currPoint.x < crothPoint.x)
		return -1;
	//右手 或 一部分右脚
	if (currPoint.x > rightPoint.x)
		return 1;
	//右脚 或 一部分右手
	if (currPoint.y < crothPoint.y && currPoint.x > crothPoint.x)
		return 1;
}

bool MathUtil::CreateQuadricFromPoints(std::vector<double>& coefficient, const std::vector<Ogre::Vector3>& points)
{
	assert(coefficient.size() == 6);
	auto n = points.size();
	if (n < 6)
		return false;
	//二次曲面f(x, y) = a1*x2 + a2*xy + a3*y2 + a4*x + a5*y + a6
	//构造超静定方程组 Gu = P: 其中G为n*6,n为points.size();u，P为n阶列阵
	//G, P
	std::vector<std::vector<double>> Gu_p;
	Gu_p.reserve(n);
	for (int i = 0; i < n; i++)
	{
		std::vector<double> g;
		g.resize(7);
		g[0] = points[i].x*points[i].x;
		g[1] = points[i].x*points[i].y;
		g[2] = points[i].y*points[i].y;
		g[3] = points[i].x;
		g[4] = points[i].y;
		g[5] = 1.f;
		
		g[6] = -points[i].z;
		Gu_p.emplace_back(g);
	}
	//I = (Gu-P)T (Gu-P), 对I，分别对coefficient[j]求偏导数
	//Σ(G[i]*u - P[i])^2
	std::vector<std::vector<double>> der_G;
	for (int i = 0; i < 6; i++)
	{
		//double g[7] = { 0 };
		std::vector<double> g;
		for (int j = 0; j < 7; j++)
		{
			g.push_back(0);
		}
		for (int j = 0; j < n; j++)
		{
			auto cc = 2.f * Gu_p[j][i];
			for (int k = 0; k < 7; k++)
			{
				g[k] += Gu_p[j][k];
			}
		}
		der_G.emplace_back(g);
	}
	// 高斯消元法 解多元一次方程组 der_G
	return Gauss(der_G, coefficient);
}

bool MathUtil::Gauss(const std::vector<std::vector<double>>& A, std::vector<double>& x)
{
	int n = A.size();
	int m = A[0].size();
	assert(m - n == 1);
	x.resize(n);
	//复制系数矩阵，防止修改原矩阵
	std::vector<std::vector<double>> Atemp(n);
	for (int i = 0; i < n; i++)
	{
		std::vector<double> temp(m);
		for (int j = 0; j < m; j++)
		{
			temp[j] = A[i][j];
		}
		Atemp[i] = temp;
		temp.clear();
	}
	//第一次消元
	for (int i = 1; i < n; i++)
	{
		if (abs(Atemp[0][0]) < DBL_EPSILON)
		{
			x.clear();
			return false;
		}
		double l = Atemp[i][0] / Atemp[0][0];
		for (int j = 0; j < m; j++)
		{
			Atemp[i][j] = Atemp[i][j] - l*Atemp[0][j];
		}
	}
	//剩余的消元
	for (int k = 1; k < n - 1; k++)
	{
		for (int i = k + 1; i < n; i++)
		{
			if (Atemp[k][k] < DBL_EPSILON)
			{
				x.clear();
				return false;
			}
			double l = Atemp[i][k] / Atemp[k][k];
			for (int j = k; j < m; j++)
			{
				Atemp[i][j] = Atemp[i][j] - l*Atemp[k][j];
			}
		}
	}
	//回代
	x[n - 1] = Atemp[n - 1][m - 1] / Atemp[n - 1][m - 2];
	for (int k = n - 2; k >= 0; k--)
	{
		double s = 0.0;
		for (int j = k + 1; j < n; j++)
		{
			s += Atemp[k][j] * x[j];
		}
		x[k] = (Atemp[k][m - 1] - s) / Atemp[k][k];
	}
	return true;
}
