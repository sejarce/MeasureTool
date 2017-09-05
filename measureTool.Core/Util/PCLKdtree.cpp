#include "PCLKdtree.h"

#include "Util/MathUtil.h"
#include "Util/CpuTimer.h"

#include "pcl/point_cloud.h"
#include "pcl/kdtree/kdtree_flann.h"

#include <boost/make_shared.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>

#include "Ogre.h"

class PCLKdtree::Imp
{
public:
	using PointType = pcl::PointXYZ;
	using PointCloudType = pcl::PointCloud<PointType>;
	using KdtreeType = pcl::KdTreeFLANN<PointType>;

public:
	struct VertexProperty
	{
		//unsigned int index;
		Ogre::Vector3 postion;
	};

	using EdgeWeightProperty = boost::property<boost::edge_weight_t, float>;

	using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeightProperty>;

	// �ڵ�������
	using VertexDescriptor = boost::graph_traits<Graph>::vertex_descriptor;
	// ��������
	using EdgeDescriptor = boost::graph_traits<Graph>::edge_descriptor;

	// �ڽ������һЩ������
	using VertexIterator = boost::graph_traits<Graph>::vertex_iterator;
	using EdgeIterator = boost::graph_traits<Graph>::edge_iterator;
	using AdjacencyIterator = boost::graph_traits<Graph>::adjacency_iterator;
	using OutEdgeIterator = boost::graph_traits<Graph>::out_edge_iterator;

public:
	std::unique_ptr<KdtreeType> Kdtree_;
	std::unique_ptr<Graph> graph_;

public:
	float getDistance(Ogre::Vector3& v1, Ogre::Vector3& v2) const
	{
		return (v1 - v2).length();
	}

	/*
	*@ �Ż�����·��
	*	�������ԣ��Գ�ʼDijkstra·��C�ϵĵ�{Pi}��ȡ���߶�(Pi-k,...Pi,...Pi+k}���Ż�Piλ�ã�ʹ�Ż��������(Pi-k,...Pi,...Pi+k}Ϊ����������Pi-k��Pi+k֮��������������ߣ�k����ѡ1��2��3...�ȣ��Ӳ������������
	*	���岽��:
	*	��1���ڳ�ʼ·��(Pi-k,...Pi,...Pi+k)��������㣬�����ֲ��������棬��ͼ1(a)��ʾ��
	*	��������������㣬����PiΪ���ģ�Pi�ϵķ�����Ϊz�ᣬ��ϵõ���������
	*	z(x,y)=b1x2+ b2xy + b3y2+ b4x+ b5y+ b6 .
	*	����{Pi-k,...Pi,...Pi+k}�ϵĵ�Pj����Pj�ķ�����Nj��������Tj���õ�����Vj=Nj��Tj����Pj����VjͶӰ�����������棬�õ��ڵ�������ʾ����������·��C�Ⱦ���������ߣ�������ɢ�㣬�Ӷ����Թ����ֲ��������档
	*	��2��չ���ֲ��������浽��άƽ�棬1(b)��ʾ������ά��piͶӰ��ֱ��pi-kpi+k���õ���p'i����¼p'i���ڵ��������ڣ������������ꣻ�����ø��������꣬ӳ�䵽��ά�������棬�Ӷ��Ż�Pi��λ�á���p'i�ڶ�Ӧ�����μ����������꣬���ں��������������У���
	*��3����kȡk+1�����k���󣩻�k-1�����k�ѱȽϴ󣩣���ת��1���������Ż���ֱ�����������Ż�����������ܳ��Ȳ���С��һ������ֵ��
	*/

	std::vector<Ogre::Vector3> optimizePointPath(const PointPath& pp)
	{
		std::vector<Ogre::Vector3> optimizeList;
		optimizeList.reserve(pp.size());

		for (auto cur : pp)
		{
			optimizeList.emplace_back(cur.second);
		}

		//Ѱ������㼯
		int k = 5;
		const int KK = 4;
		for (int i = 0; i < optimizeList.size(); i++)
		{
			if (i - k < 0 || i + k > optimizeList.size())
				continue;

			//i-k..i..i+k
			//std::map<int, Ogre::Vector3> pointsQuadricSurface;
			//for (int j = i - k; j <= i+k; j++)
			//{
			//	std::vector<int> pointIdxNKNSearch(KK);
			//	std::vector<float> pointNKNSquaredDistance(KK);
			//	auto searchPoint = Kdtree_->getInputCloud()->points[j];
			//	pointsQuadricSurface[j] = MathUtil::ToOgre(searchPoint);
			//	if ((*Kdtree_).nearestKSearch(searchPoint, KK, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
			//	{
			//		for (size_t ii = 0; ii < pointIdxNKNSearch.size(); ++ii)
			//		{
			//			auto pointIndex = pointIdxNKNSearch[ii];
			//			pointsQuadricSurface[pointIndex] = MathUtil::ToOgre(Kdtree_->getInputCloud()->points[pointIndex]);
			//		}
			//	}
			//}

			//��i��ľֲ�����ϵ
			//i��
			auto localO = optimizeList[i];
			//������
			std::vector<int> pointIdxNKNSearch(KK);
			std::vector<float> pointNKNSquaredDistance(KK);
			//std::map<int, Ogre::Vector3> pointsQuadricSurface;
			std::vector<Ogre::Vector3> pointsQuadricSurface;
			if ((*Kdtree_).nearestKSearch(MathUtil::ToPCL(localO), KK, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
			{
				for (size_t ii = 0; ii < pointIdxNKNSearch.size(); ++ii)
				{
					auto pointIndex = pointIdxNKNSearch[ii];
					//pointsQuadricSurface[pointIndex] = MathUtil::ToOgre(Kdtree_->getInputCloud()->points[pointIndex]);
					pointsQuadricSurface.emplace_back(MathUtil::ToOgre(Kdtree_->getInputCloud()->points[pointIndex]));
				}
			}
			auto localONormal = Ogre::Vector3{ 0.0, 0.0, 0.0 };
			for (int j = 0; j < KK; ++j)
			{
				auto v0 = pointsQuadricSurface[j] - localO;
				float angle = FLT_MAX;
				Ogre::Vector3 v1;
				for (int jj = 0; jj < KK && jj != j; ++jj)
				{
					auto v = pointsQuadricSurface[jj] - localO;
					auto ang = v.angleBetween(v0);
					if (ang.valueRadians() < angle && ang.valueRadians() > 0)
					{
						angle = ang.valueRadians();
						v1 = v;
					}
				}
				localONormal += v1.crossProduct(v0).normalise();
			}
			localONormal.normalise();
			//������
			auto localOTangent = optimizeList[i + k] - optimizeList[i - k];
			localOTangent.normalise();
			auto localOTangent2 = localONormal.crossProduct(localOTangent);
			//worldתlocal matrix
			Ogre::Matrix4 world2local;
			//world2local.setTrans(-localO);
			Ogre::Quaternion quat(localOTangent2, localOTangent, localONormal);
			world2local.makeInverseTransform(localO, { 1.0, 1.0, 1.0 }, quat);
			
			//pointsQuadricSurface ����������� f(x,y) = a1*x2+a2*xy+a3*y2+a4*x+a5*y+a6
			//ȡi-k�㣬i+k�㣬i�㣬�Լ�i����������ɵ�
			std::map<int, Ogre::Vector3> map_pointsI2KQuadric;
			for (int j = i - k; j <= i+k; j++)
			{
				std::vector<int> pointIdxNKNSearch(KK);
				std::vector<float> pointNKNSquaredDistance(KK);
				auto searchPoint = Kdtree_->getInputCloud()->points[j];
				map_pointsI2KQuadric[j] = MathUtil::ToOgre(searchPoint);
				if ((*Kdtree_).nearestKSearch(searchPoint, KK, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
				{
					for (size_t ii = 0; ii < pointIdxNKNSearch.size(); ++ii)
					{
						auto pointIndex = pointIdxNKNSearch[ii];
						map_pointsI2KQuadric[pointIndex] = MathUtil::ToOgre(Kdtree_->getInputCloud()->points[pointIndex]);
					}
				}
			}

			//�������㼯��������С���˹����������
			std::vector<Ogre::Vector3> vec_pointsI2KQuadric;
			for (auto cur : map_pointsI2KQuadric)
			{
				auto localpos = world2local * cur.second;
				vec_pointsI2KQuadric.emplace_back(localpos);
			}
			std::vector<double> coe(6);
			MathUtil::CreateQuadricFromPoints(coe, vec_pointsI2KQuadric);
			
			//���¹����ֲ���������
			std::vector<Ogre::Vector3> localQuadricListsA;
			std::vector<Ogre::Vector3> localQuadricListsB;
			for (int ja = i+1, jb = i-1 ; ja <= i+k, jb >= i-k; ja++, jb--)
			{
				{
					auto mpos = world2local * optimizeList[ja];
					auto x = mpos.x;
					auto y = mpos.y;
					float z = coe[0] * x*x + coe[1] * x*y + coe[2] * y*y + coe[3] * x + coe[4] * y + coe[5];
					optimizeList[ja].z = z;

					localQuadricListsA.emplace_back(Ogre::Vector3{ x, y, z });

					//Pj����Vj
					auto y1 = y + 0.01f;	//1cm
					float z1 = coe[0] * x*x + coe[1] * x*y1 + coe[2] * y1*y1 + coe[3] * x + coe[4] * y1 + coe[5];
					localQuadricListsA.emplace_back(Ogre::Vector3{ x, y1, z1 });

					auto y2 = y - 0.01f;
					float z2 = coe[0] * x*x + coe[1] * x*y2 + coe[2] * y2*y2 + coe[3] * x + coe[4] * y2 + coe[5];
					localQuadricListsA.emplace_back(Ogre::Vector3{ x, y2, z2 });
				}
				{
					auto mpos = optimizeList[jb];
					auto x = mpos.x;
					auto y = mpos.y;
					float z = coe[0] * x*x + coe[1] * x*y + coe[2] * y*y + coe[3] * x + coe[4] * y + coe[5];
					optimizeList[jb].z = z;

					localQuadricListsB.emplace_back(Ogre::Vector3{ x, y, z });

					//Pj����Vj
					auto y1 = y + 0.01f;	//1cm
					float z1 = coe[0] * x*x + coe[1] * x*y1 + coe[2] * y1*y1 + coe[3] * x + coe[4] * y1 + coe[5];
					localQuadricListsB.emplace_back(Ogre::Vector3{ x, y1, z1 });

					auto y2 = y - 0.01f;
					float z2 = coe[0] * x*x + coe[1] * x*y2 + coe[2] * y2*y2 + coe[3] * x + coe[4] * y2 + coe[5];
					localQuadricListsB.emplace_back(Ogre::Vector3{ x, y2, z2 });
				}
			}

			//�����������ڵ�localQuadricLists[k*3]����0��0��0��չ��
			//չ��ѡ�����ĵ�ֱ���ٽ���
			{
				auto centerPos = world2local * optimizeList[i];

				std::vector<Ogre::Vector3> aroundPoses;
				aroundPoses.emplace_back(localQuadricListsA[0]);
				auto y1 = centerPos.y + 0.01f;	//1cm
				float z1 = coe[0] * centerPos.x*centerPos.x + coe[1] * centerPos.x*y1 + coe[2] * y1*y1 + coe[3] * centerPos.x + coe[4] * y1 + coe[5];
				aroundPoses.emplace_back(Ogre::Vector3{ centerPos.x, y1, z1 });
				aroundPoses.emplace_back(localQuadricListsB[0]);
				auto y2 = centerPos.y - 0.01f;
				float z2 = coe[0] * centerPos.x*centerPos.x + coe[1] * centerPos.x*y2 + coe[2] * y2*y2 + coe[3] * centerPos.x + coe[4] * y2 + coe[5];
				aroundPoses.emplace_back(Ogre::Vector3{ centerPos.x, y2, z2 });

				//��нǺ�
				auto angle_sum = 0.f;
				
				std::vector<Ogre::Vector3> v;
				for (auto cur : aroundPoses)
				{
					v.emplace_back((cur - centerPos).normalisedCopy());
				}				
				angle_sum += v[0].dotProduct(v[1]);
				angle_sum += v[1].dotProduct(v[2]);
				angle_sum += v[2].dotProduct(v[3]);
				angle_sum += v[3].dotProduct(v[0]);

				//չ����Χ��

				
			}
			//�� i ==>> i+k ����չ��


			//�� i ==>> i-k ����չ��
		}
		return optimizeList;
	}
};

PCLKdtree::PCLKdtree() :
	ImpUPtr_(std::make_unique<Imp>())
{
}

PCLKdtree::~PCLKdtree()
{
}

void PCLKdtree::Build(const Ogre::MeshPtr& meshPtr)
{
	auto& imp_ = *ImpUPtr_;

	imp_.Kdtree_ = std::make_unique<Imp::KdtreeType>();

	//read vectexData form mesh
	std::vector<Ogre::VertexData*> vertexDataList;

	if (meshPtr->sharedVertexData)
	{
		vertexDataList.emplace_back(meshPtr->sharedVertexData);
	}

	for (auto subIndex = 0; subIndex < meshPtr->getNumSubMeshes(); ++subIndex)
	{
		auto curSub = meshPtr->getSubMesh(subIndex);

		if (!curSub->useSharedVertices)
		{
			vertexDataList.push_back(curSub->vertexData);
		}
	}

	auto totalVertexCount = 0U;
	for (auto curVD : vertexDataList)
	{
		auto vbuf = curVD->vertexBufferBinding->getBuffer(0);
		totalVertexCount += static_cast<uint32_t>(vbuf->getNumVertices());
	}

	//biuld PointCloud 
	auto rawCloud = boost::make_shared<Imp::PointCloudType>();
	{
		rawCloud->reserve(totalVertexCount);

		for (auto curVD : vertexDataList)
		{
			auto vbuf = curVD->vertexBufferBinding->getBuffer(0);
			auto pBuf = static_cast<uint8_t*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

			auto posEle = curVD->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
			float* pVal;
			auto pntSize = vbuf->getNumVertices();
			auto vSize = curVD->vertexDeclaration->getVertexSize(0);
			for (auto curPntIndex = 0; curPntIndex < pntSize; ++curPntIndex)
			{
				posEle->baseVertexPointerToElement(pBuf, &pVal);
				auto& curVer = *reinterpret_cast<Ogre::Vector3*>(pVal);

				rawCloud->push_back(MathUtil::ToPCL(curVer));

				pBuf += vSize;
			}

			vbuf->unlock();
		}
	}

	//build kdtree
	imp_.Kdtree_->setInputCloud(rawCloud);	
	std::cout << "kdtree build ok!" << std::endl;

	imp_.graph_ = std::make_unique<Imp::Graph>(rawCloud->size());
	std::cout << "init Graph ok!" << std::endl;

	//add edges
	Imp::VertexIterator vertexIt, vertexEnd;
	//vertexIt = vertices(*imp_.graph_).first;
	//vertexEnd = vertices(*imp_.graph_).second;
	boost::tie(vertexIt, vertexEnd) = vertices(*imp_.graph_);
	Imp::PointCloudType::iterator rawCloudIt = rawCloud->begin();

	std::cout << "boost::tie ok!" << std::endl;

	const int K = 8;
	int index = 0;
	for (; vertexIt != vertexEnd; ++vertexIt, ++rawCloudIt, ++index)
	{
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);
		auto searchPoint = rawCloud->points[index];

		if ((*imp_.Kdtree_).nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			//std::cout << "nearestKSearch ok!" << std::endl;

			for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
			{
				auto offset = pointIdxNKNSearch[i] - index;
				if (offset > 0)
				{
					//std::cout << "begin add_edge!" << std::endl;
					boost::add_edge(*vertexIt, *vertexIt + offset, sqrtf(pointNKNSquaredDistance[i]), *imp_.graph_);
					//std::cout << "add_edge once!" << std::endl;
				}
			}
		}
	}
	//graph build ok!
	std::cout << "graph build ok!" << std::endl;
}

PCLKdtree::PointPath PCLKdtree::computerShortestPath(unsigned int si, unsigned int ei)
{
	auto& imp_ = *ImpUPtr_;
	PointPath pp;

	std::vector<Imp::VertexDescriptor> p(boost::num_vertices(*imp_.graph_));
	std::vector<float> d(boost::num_vertices(*imp_.graph_));
	Imp::VertexIterator vertexIt = boost::vertices(*imp_.graph_).first;
	Imp::VertexDescriptor s = boost::vertex(*vertexIt + si, *imp_.graph_);

	//std::cout << "start:" << s << std::endl;

	dijkstra_shortest_paths(*imp_.graph_, s,
		predecessor_map(boost::make_iterator_property_map(p.begin(), boost::get(boost::vertex_index, *imp_.graph_))).
		distance_map(boost::make_iterator_property_map(d.begin(), boost::get(boost::vertex_index, *imp_.graph_))));

	Imp::VertexDescriptor e = boost::vertex(*vertexIt + ei, *imp_.graph_);

	//std::cout << "end:" << e << std::endl;

	std::cout << "distance(" << si << "<-" << ei << ") = " << d[e] << std::endl;
	
	auto& curPntCloud = imp_.Kdtree_->getInputCloud()->points;

	//std::cout << e;
	auto pathPoint = std::make_pair(e, MathUtil::ToOgre(curPntCloud[e]));
	pp.emplace_back(pathPoint);
	while (e != s)
	{
		//std::cout << "<-" << p[e];
		e = p[e];
		auto pathPoint = std::make_pair(e, MathUtil::ToOgre(curPntCloud[e]));
		pp.emplace_back(pathPoint);
	}
	//std::cout << std::endl;

	return pp;
}

int PCLKdtree::findPointIndex(const Ogre::Vector3& pt)
{
	auto& imp_ = *ImpUPtr_;
	
	const int K = 1;
	pcl::PointXYZ searchPoint = MathUtil::ToPCL(pt);
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	if ((*imp_.Kdtree_).nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{
		if (pointNKNSquaredDistance[0] < 4e-6)
			return pointIdxNKNSearch[0];
	}
	return 0;
}