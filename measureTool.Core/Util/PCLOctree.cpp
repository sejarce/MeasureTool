#include "PCLOctree.h"

#include "Util/MathUtil.h"
#include "Util/CpuTimer.h"

#include "pcl/point_cloud.h"
#include "pcl/octree/octree.h"
#include "pcl/octree/impl/octree_iterator.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/segmentation/extract_clusters.h"

#include "Ogre.h"

#include <map>

class	PCLOctree::Imp
{
public:

	using	PointType = pcl::PointXYZ;
	using	PointCloudType = pcl::PointCloud<PointType>;
	using	OctreeType = pcl::octree::OctreePointCloudSearch<PointType>;
	using	NodeBoxMap = std::map<pcl::octree::OctreeNode*, Ogre::AxisAlignedBox>;	

	using	RayQueryResult = std::multimap<float, OctreeType::LeafNode*>;
	using	PlaneQueryResult = std::vector<OctreeType::LeafNode*>;

public:

	std::unique_ptr<OctreeType>	Octree_;
	NodeBoxMap					NodeBoxList_;

	bool						UseFeature_{ false };
	Ogre::Vector3				LeftArmpit_;
	Ogre::Vector3				RightArmpit_;
	Ogre::Vector3				Crotch_;

public:

	static void	RayQuery(RayQueryResult& result, const NodeBoxMap& nodeBoxMap, pcl::octree::OctreeNode* curNode, const Ogre::Ray& ray)
	{
		auto& curBox = nodeBoxMap.at(curNode);

		auto intRet = ray.intersects(curBox);

		if ( !intRet.first )
		{
			return;
		}

		if ( curNode->getNodeType() == pcl::octree::BRANCH_NODE )
		{
			auto branchNode = static_cast<OctreeType::BranchNode*>( curNode );

			for ( auto index = 0U; index < 8; ++index )
			{
				auto curChild = branchNode->getChildPtr(index);
				if ( curChild )
				{
					RayQuery(result, nodeBoxMap, curChild, ray);
				}
			}
		}
		else
		{
			auto leafNode = static_cast<OctreeType::LeafNode*>( curNode );
			result.emplace(intRet.second, leafNode);
		}
	}

	static void	PlaneQuery(PlaneQueryResult& result, const NodeBoxMap& nodeBoxMap, pcl::octree::OctreeNode* curNode, const Ogre::Plane& pln)
	{
		auto& curBox = nodeBoxMap.at(curNode);

		auto intRet = Ogre::Math::intersects(pln, curBox);
		if ( !intRet )
		{
			return;
		}

		if ( curNode->getNodeType() == pcl::octree::BRANCH_NODE )
		{
			auto branchNode = static_cast<OctreeType::BranchNode*>( curNode );

			for ( auto index = 0U; index < 8; ++index )
			{
				auto curChild = branchNode->getChildPtr(index);
				if ( curChild )
				{
					PlaneQuery(result, nodeBoxMap, curChild, pln);
				}
			}
		}
		else
		{
			auto leafNode = static_cast<OctreeType::LeafNode*>( curNode );
			result.push_back(leafNode);
		}
	}

public:

	void	_UpdateByFeature(const Ogre::Vector3& featurePnt, PointCloudType::Ptr& pntList, const Ogre::Plane& queryPln, const Ogre::Vector3& queryPnt, float upperRange, float underRange)
	{
		Ogre::Plane featurePln(Ogre::Vector3::UNIT_X, featurePnt);

		auto featurePontOnQueryPln = MathUtil::ProjectPointOnPlane(featurePnt, queryPln);
		auto reflectFeaturePoint = MathUtil::ProjectPointOnPlane(featurePontOnQueryPln, featurePln);
		auto featureDir = ( reflectFeaturePoint - featurePnt ).normalisedCopy();
		Ogre::Ray featureRay(featurePnt, featureDir);

		auto intRes = Ogre::Math::intersects(featureRay, queryPln);
		if ( !intRes.first )
		{
			return;
		}

		auto projectFeaturePoint = featureRay.getPoint(intRes.second);

		auto featureDelta = projectFeaturePoint - featurePnt;
		auto disY = featureDelta.dotProduct(Ogre::Vector3::UNIT_Y);

		if ( ( disY > 0 && disY < upperRange ) || ( disY < 0 && disY > underRange ) )
		{
			auto baseVec = queryPnt - projectFeaturePoint;
			auto baseVal = baseVec.dotProduct(Ogre::Vector3::UNIT_X);

			auto itor = std::remove_if(pntList->begin(), pntList->end(), [&](const PointType& curPnt)
			{
				auto curVec = MathUtil::ToOgre(curPnt) - projectFeaturePoint;
				auto curVal = curVec.dotProduct(Ogre::Vector3::UNIT_X);

				return baseVal * curVal < 0;
			});
			pntList->erase(itor, pntList->end());
		}
	}

	void	UpdateByFeature(PointCloudType::Ptr& pntList, const Ogre::Plane& queryPln, const Ogre::Vector3& queryPnt)
	{
		if ( !UseFeature_ )
		{
			return;
		}

		_UpdateByFeature(LeftArmpit_, pntList, queryPln, queryPnt, 0.03f, -0.1f);
		_UpdateByFeature(RightArmpit_, pntList, queryPln, queryPnt, 0.03f, -0.1f);
		_UpdateByFeature(Crotch_, pntList, queryPln, queryPnt, 0.03f, -0.2f);
	}
};

PCLOctree::PCLOctree() :ImpUPtr_(std::make_unique<Imp>())
{

}

PCLOctree::~PCLOctree()
{

}

void PCLOctree::Build(const Ogre::MeshPtr& meshPtr, float leafSize, float resolution)
{
	auto& imp_ = *ImpUPtr_;

	imp_.Octree_ = std::make_unique<Imp::OctreeType>(resolution);

	std::vector<Ogre::VertexData*> vertexDataList;

	if ( meshPtr->sharedVertexData )
	{
		vertexDataList.push_back(meshPtr->sharedVertexData);
	}

	for ( auto subIndex = 0; subIndex < meshPtr->getNumSubMeshes(); ++subIndex )
	{
		auto curSub = meshPtr->getSubMesh(subIndex);

		if ( !curSub->useSharedVertices )
		{
			vertexDataList.push_back(curSub->vertexData);
		}
	}

	auto totalVertexCount = 0U;
	for ( auto curVD : vertexDataList )
	{
		auto vbuf = curVD->vertexBufferBinding->getBuffer(0);
		totalVertexCount += static_cast<uint32_t>( vbuf->getNumVertices() );
	}

	auto rawCloud = boost::make_shared<Imp::PointCloudType>();
	{
		rawCloud->reserve(totalVertexCount);

		for ( auto curVD : vertexDataList )
		{
			auto vbuf = curVD->vertexBufferBinding->getBuffer(0);
			auto pBuf = static_cast<uint8_t*>( vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY) );

			auto posEle = curVD->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
			float* pVal;
			auto pntSize = vbuf->getNumVertices();
			auto vSize = curVD->vertexDeclaration->getVertexSize(0);
			for ( auto curPntIndex = 0; curPntIndex < pntSize; ++curPntIndex )
			{
				posEle->baseVertexPointerToElement(pBuf, &pVal);
				auto& curVer = *reinterpret_cast<Ogre::Vector3*>( pVal );

				rawCloud->push_back(MathUtil::ToPCL(curVer));

				pBuf += vSize;
			}

			vbuf->unlock();
		}
	}

	//auto filterCloud = boost::make_shared<Imp::PointCloudType>();
	//{
	//	pcl::VoxelGrid<pcl::PointXYZ> sor;
	//	sor.setInputCloud(rawCloud);
	//	sor.setLeafSize(leafSize, leafSize, leafSize);
	//	sor.filter(*filterCloud);
	//}

	imp_.Octree_->setInputCloud(rawCloud);
	imp_.Octree_->addPointsFromInputCloud();

	for ( auto itor = imp_.Octree_->begin(); itor != imp_.Octree_->end(); ++itor )
	{
		Eigen::Vector3f bmin, bmax;

		imp_.Octree_->getVoxelBounds(itor, bmin, bmax);

		imp_.NodeBoxList_.emplace(*itor, Ogre::AxisAlignedBox(MathUtil::ToOgre(bmin), MathUtil::ToOgre(bmax)));
	}
}

PCLOctree::RayQueryOpt PCLOctree::RayQuery(const Ogre::Ray& ray, float resolution) const
{
	auto& imp_ = *ImpUPtr_;

	Imp::RayQueryResult boxQueryResult;
	Imp::RayQuery(boxQueryResult, imp_.NodeBoxList_, *(imp_.Octree_->begin()), ray);

	if (boxQueryResult.empty())
	{
		return PCLOctree::RayQueryOpt{ false, 0, Ogre::Vector3{ 0, 0, 0 } };
	}

	PCLOctree::RayQueryOpt queryPnt{ false, 0, Ogre::Vector3{ 0, 0, 0 } };

	auto squareRayPickingDeviation = resolution * resolution;

	auto egOrigin = MathUtil::ToEigen(ray.getOrigin());
	auto egDir = MathUtil::ToEigen(ray.getDirection());

	auto& pointCloud = *( imp_.Octree_->getInputCloud() );
	auto minSquVal = std::numeric_limits<float>::max();

	for ( auto& cur : boxQueryResult )
	{
		auto leafNode = cur.second;

		auto& leafContainer = leafNode->getContainer();
		auto& indices = leafContainer.getPointIndicesVector();
		for ( auto& curIndex : indices )
		{
			Eigen::Vector3f curPnt = pointCloud[curIndex].getVector3fMap();
			auto projP = MathUtil::ProjectPointOnLine(curPnt, egOrigin, egDir);

			if ( ( projP - curPnt ).squaredNorm() > squareRayPickingDeviation )
			{
				continue;
			}

			auto curVal = ( curPnt - egOrigin ).squaredNorm();
			if (minSquVal > curVal)
			{
				minSquVal = curVal;
				//queryPnt = MathUtil::ToOgre(curPnt);
				queryPnt = std::make_tuple(true, curIndex, MathUtil::ToOgre(curPnt));
			}
		}

		if ( std::get<0>(queryPnt) )
		{
			break;
		}
	}

	return queryPnt;
}

PCLOctree::PlaneQueryVector PCLOctree::PlaneQuery(const Ogre::Plane& pln, const Ogre::Vector3& queryPnt, float resolution) const
{
	auto& imp_ = *ImpUPtr_;

	auto squareDeviation = resolution * resolution;

	Imp::PlaneQueryResult plnResult;
	Imp::PlaneQuery(plnResult, imp_.NodeBoxList_, *( imp_.Octree_->begin() ), pln);

	auto clusters = boost::make_shared<Imp::PointCloudType>();

	for ( auto curNode : plnResult )
	{
		auto& indices = curNode->getContainer().getPointIndicesVector();
		auto& curPntCloud = imp_.Octree_->getInputCloud()->points;
		for ( auto curIndex : indices )
		{
			auto curPnt = MathUtil::ToOgre(curPntCloud[curIndex]);
			auto plnToPnt = curPnt - queryPnt;
			auto dis = plnToPnt.dotProduct(pln.normal);
			if ( std::abs(dis) > resolution )
			{
				continue;
			}
			auto projectPnt = curPnt - pln.normal * dis;

			clusters->points.push_back(MathUtil::ToPCL(projectPnt));
		}
	}

	if ( clusters->empty() )
	{
		return{};
	}

	//根据预设的特征点，区域划分
	imp_.UpdateByFeature(clusters, pln, queryPnt);

	pcl::EuclideanClusterExtraction<Imp::PointType> ec;
	ec.setClusterTolerance(0.01f);
	ec.setMinClusterSize(100);
	ec.setInputCloud(clusters);

	std::vector<pcl::PointIndices> clusterIndices;
	ec.extract(clusterIndices);

	std::multimap<float, PlaneQueryVector> clusterList;
	Ogre::AxisAlignedBox medBox;
	PlaneQueryVector medList;
	for ( auto& cur : clusterIndices )
	{
		Ogre::AxisAlignedBox curBox;
		PlaneQueryVector curList;
		for ( auto& curIndex : cur.indices )
		{
			auto curPnt = MathUtil::ToOgre(clusters->points[curIndex]);
			curBox.merge(curPnt);
			curList.push_back(curPnt);
		}

		auto curCenter = curBox.getCenter();
		if ( ( curCenter.x > imp_.RightArmpit_.x + 1e-2 )
			&& ( curCenter.x < imp_.LeftArmpit_.x - 1e-2 )
			&& ( curCenter.y > imp_.Crotch_.y ) )
		{
			medBox.merge(curBox);
			std::copy(curList.begin(), curList.end(), std::back_inserter(medList));
		}
		else
		{
			auto curDis = curBox.distance(queryPnt);
			clusterList.emplace(curDis, curList);
		}
	}

	if ( !medList.empty() )
	{
		auto curDis = medBox.distance(queryPnt);
		clusterList.emplace(curDis, std::move(medList));
	}

	if ( clusterList.empty() )
	{
		return{};
	}

	auto itor = clusterList.begin();

	if ( itor->first > resolution )
	{
		return{};
	}

	return itor->second;
}

void PCLOctree::SetFeaturePoint(const Ogre::Vector3& leftArmpit, const Ogre::Vector3& rightArmpit, const Ogre::Vector3& crotch)
{
	auto& imp_ = *ImpUPtr_;

	imp_.UseFeature_ = true;
	imp_.LeftArmpit_ = leftArmpit;
	imp_.RightArmpit_ = rightArmpit;
	imp_.Crotch_ = crotch;
}
