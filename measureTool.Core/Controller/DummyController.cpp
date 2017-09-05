#pragma warning(disable:4996)

#include "DummyController.h"

#include "Render/Camera/MayaCamera.h"
#include "Render/Extension/RadiusRaySceneQuery.h"
#include "Render/QueryUtil.h"
#include "Render/VisibilityFlag.h"
#include "Render/Extension/SimpleBoundingBox.h"

#include "Util/MathUtil.h"
#include "Util/CpuTimer.h"

#include "pcl/point_cloud.h"
#include "pcl/octree/octree.h"
#include "pcl/octree/impl/octree_iterator.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/segmentation/extract_clusters.h"

#include "Ogre.h"
#include "Compositor/OgreCompositorManager2.h"
#include "Compositor/OgreCompositorWorkspace.h"
#include "OgreMeshSerializer.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem/fstream.hpp>

static const auto MaxSelectedSize = 5000U;

class 	DummyController::Imp
{
public:

	using	OctreeType = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>;
	using BoxPair = std::tuple<SimpleBoundingBox*, OctreeType::LeafNode*>;

public:

	Ogre::RenderWindow*			RT_{};
	MayaCameraSPtr 				CameraListener_{};
	Ogre::SceneManager*			Smgr_{};
	Ogre::CompositorWorkspace*	WorkSpce_{};
	Ogre::AnimationState*		AnimationState_{};
	Ogre::Entity*				HumanEntity_{};
	Ogre::Entity*				SelecetedEntity_{};
	Ogre::MeshPtr				SelectedMesh_;

	std::unique_ptr<OctreeType>	Octree_;
	std::vector<BoxPair>		BBList_;

public:

	static std::vector<std::tuple<Ogre::Vector3, Ogre::ColourValue>>	LoadPointCloud(boost::filesystem::ifstream& ifs)
	{
		std::string line;
		std::vector<std::string> split;

		std::vector<std::tuple<Ogre::Vector3, Ogre::ColourValue>> ret;

		for ( auto index = 1; index <= 10; ++index )
		{
			std::getline(ifs, line);

			if ( index == 2 )
			{
				boost::algorithm::split(split, line, boost::algorithm::is_any_of(" "));
				ret.reserve(std::stoi(split[2]));
			}
		}

		while ( ifs )
		{
			line.clear();
			split.clear();

			std::getline(ifs, line);
			if ( line.empty() )
			{
				continue;
			}

			boost::algorithm::split(split, line, boost::algorithm::is_any_of(" "));
			split.erase(std::remove_if(split.begin(), split.end(), [](const std::string& str)
			{
				return str.empty();
			}), split.end());

			auto px = std::stof(split[0]);
			auto py = std::stof(split[1]);
			auto pz = std::stof(split[2]);
			auto cr = std::stof(split[3]);
			auto cg = std::stof(split[4]);
			auto cb = std::stof(split[5]);

			ret.emplace_back(Ogre::Vector3(px, py, pz), Ogre::ColourValue(cr / 255.f, cg / 255.f, cb / 255.f));
		}

		return ret;
	}

	static Ogre::MeshPtr	LoadMesh()
	{
		auto retMesh = Ogre::MeshManager::getSingleton().createManual("testPly", "General");

		for ( auto index = 1; index <= 20; ++index )
		{
			boost::filesystem::ifstream ifs("Data/Models/testPly/" + std::to_string(index) + ".ply");

			auto vList = LoadPointCloud(ifs);

			auto curSubMesh = retMesh->createSubMesh(std::to_string(index));
			curSubMesh->operationType = Ogre::RenderOperation::OT_POINT_LIST;
			curSubMesh->useSharedVertices = false;
			curSubMesh->vertexData = new Ogre::VertexData;

			auto offset = 0;
			auto decl = curSubMesh->vertexData->vertexDeclaration;
			auto& posEle = decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
			offset += static_cast<decltype( offset )>( Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3) );
			auto& clrEle = decl->addElement(0, offset, Ogre::VET_COLOUR_ARGB, Ogre::VES_DIFFUSE);
			offset += static_cast<decltype( offset )>( Ogre::VertexElement::getTypeSize(Ogre::VET_COLOUR_ARGB) );

			auto vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(decl->getVertexSize(0), vList.size(), Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY, true);
			{
				auto pBuf = static_cast<uint8_t*>( vBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD) );

				for ( auto& cur : vList )
				{
					auto& pos = std::get<0>(cur);
					auto& clr = std::get<1>(cur);

					float* pVal;
					Ogre::RGBA* pClrVal;

					posEle.baseVertexPointerToElement(pBuf, &pVal);
					pVal[0] = pos.x;
					pVal[1] = pos.y;
					pVal[2] = pos.z;

					clrEle.baseVertexPointerToElement(pBuf, &pClrVal);
					*pClrVal = clr.getAsARGB();

					pBuf += decl->getVertexSize(0);
				}

				vBuf->unlock();

				curSubMesh->vertexData->vertexBufferBinding->setBinding(0, vBuf);
				curSubMesh->vertexData->vertexCount = vBuf->getNumVertices();
			}
		}

		retMesh->_updateBoundsFromVertexBuffers();

		Ogre::MeshSerializer ms;
		ms.exportMesh(retMesh.get(), "plyTest.mesh", Ogre::MESH_VERSION_1_8);

		return retMesh;
	}

	static void	SaveToPly(const std::vector<Ogre::Vector3>& vList, const std::string& fileName)
	{
		boost::filesystem::ofstream ofs(fileName, std::ios::trunc);

		ofs << "ply\n";
		ofs << "format ascii 1.0\n";
		ofs << "element vertex " << vList.size() << "\n";
		ofs << "property float x\n";
		ofs << "property float y\n";
		ofs << "property float z\n";
		ofs << "end_header\n";
		for ( auto& cur : vList )
		{
			ofs << cur[0] << " " << cur[1] << " " << cur[2] << "\n";
		}
	}

public:

	void	BuildOctree(const Ogre::MeshPtr& mesh)
	{
		auto bound = mesh->getBounds();

		std::vector<Ogre::VertexData*> vdList;

		if ( mesh->sharedVertexData )
		{
			vdList.push_back(mesh->sharedVertexData);
		}

		for ( auto subIndex = 0; subIndex < mesh->getNumSubMeshes(); ++subIndex )
		{
			auto curSub = mesh->getSubMesh(subIndex);

			if ( !curSub->useSharedVertices )
			{
				vdList.push_back(curSub->vertexData);
			}
		}

		auto vCount = 0U;
		for ( auto curVD : vdList )
		{
			auto vbuf = curVD->vertexBufferBinding->getBuffer(0);
			vCount += static_cast<uint32_t>(vbuf->getNumVertices());
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr rawCloud(new pcl::PointCloud<pcl::PointXYZ>);
		{
			rawCloud->reserve(vCount);

			for ( auto curVD : vdList )
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

		pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud(new pcl::PointCloud<pcl::PointXYZ>);
		{
			pcl::VoxelGrid<pcl::PointXYZ> sor;
			sor.setInputCloud(rawCloud);
			sor.setLeafSize(0.005f, 0.005f, 0.005f);
			//sor.setLeafSize(0.01f, 0.01f, 0.01f);
			sor.filter(*filterCloud);
		}

		Octree_ = std::make_unique<OctreeType>(0.05);
		Octree_->setInputCloud(filterCloud);
		Octree_->addPointsFromInputCloud();

		auto boxNode = HumanEntity_->getParentSceneNode()->createChildSceneNode();

		for ( OctreeType::Iterator itor = Octree_->begin(); itor != Octree_->end(); ++itor )
		{
			if ( !itor.isLeafNode() )
			{
				continue;
			}

			Eigen::Vector3f bmin, bmax;

			Octree_->getVoxelBounds(itor, bmin, bmax);

			auto box = SimpleBoundingBoxFactory::CreateInstance(Smgr_);
			box->SetupBoundingBox({ Ogre::Vector3(bmin[0], bmin[1], bmin[2]), Ogre::Vector3(bmax[0], bmax[1], bmax[2]) });
			boxNode->attachObject(box);

			auto leafNode = static_cast<OctreeType::LeafNode*>( *itor );
			BBList_.emplace_back(box, leafNode);
			box->setVisible(false);
		}
	}

	void	UpdateSelected(const std::vector<Ogre::Vector3>& centerList, bool save)
	{
		Ogre::Vector3 centroid = Ogre::Vector3::ZERO;
		for ( auto& cur : centerList )
		{
			centroid += cur;
		}
		centroid /= centerList.size();

		std::vector<Ogre::Vector3> front, back;
		for ( auto deg = 0; deg < 60; ++deg )
		{
			Ogre::Quaternion qua;
			qua.FromAngleAxis(Ogre::Radian(Ogre::Degree(deg * 3)), Ogre::Vector3::UNIT_Y);
			auto dir = qua * Ogre::Vector3::UNIT_X;

			std::vector<Ogre::Vector3> tmpFront, tmpBack;

			for ( auto& cur : centerList )
			{
				auto proj = MathUtil::ProjectPointOnLine(cur, centroid, dir);
				auto sd = cur.squaredDistance(proj);
				if ( sd > 25e-6 )
				{
					continue;
				}

				auto curDir = proj - centroid;
				if ( curDir.dotProduct(dir) > 0 )
				{
					tmpFront.push_back(cur);
				}
				else
				{
					tmpBack.push_back(cur);
				}
			}

			Ogre::Vector3 frontPnt = Ogre::Vector3::ZERO, backPnt = Ogre::Vector3::ZERO;

			if ( !tmpFront.empty() )
			{
				for ( auto& cur : tmpFront )
				{
					frontPnt += cur;
				}
				frontPnt /= tmpFront.size();
				auto proj = MathUtil::ProjectPointOnLine(frontPnt, centroid, dir);
				front.push_back(proj);
			}
			
			if ( !tmpBack.empty() )
			{
				for ( auto& cur : tmpBack )
				{
					backPnt += cur;
				}
				backPnt /= tmpBack.size();
				auto proj = MathUtil::ProjectPointOnLine(backPnt, centroid, dir);
				back.push_back(proj);
			}
		}

		std::copy(back.begin(), back.end(), std::back_inserter(front));

		if ( save )
		{
			
			SaveToPly(front, "param.ply");
		}

		auto subMesh = SelectedMesh_->getSubMesh(0);
		auto curSize = static_cast<decltype( MaxSelectedSize )>( front.size() );
		auto maxSize = std::min(curSize, MaxSelectedSize);

		SelectedMesh_->sharedVertexData->vertexCount = maxSize;

		auto decl = SelectedMesh_->sharedVertexData->vertexDeclaration;

		auto posEle = decl->findElementBySemantic(Ogre::VES_POSITION);

		Ogre::AxisAlignedBox box;

		auto vBuf = SelectedMesh_->sharedVertexData->vertexBufferBinding->getBuffer(0);
		auto pBuf = static_cast<uint8_t*>( vBuf->lock(Ogre::HardwareBuffer::HBL_WRITE_ONLY) );
		{
			float* pVal;

			for ( auto index = 0U; index < maxSize; ++index )
			{
				posEle->baseVertexPointerToElement(pBuf, &pVal);
				auto& curPnt = *reinterpret_cast<Ogre::Vector3*>( pVal );
				curPnt = front[index];
				box.merge(curPnt);

				pBuf += vBuf->getVertexSize();
			}

			vBuf->unlock();
		}

		SelectedMesh_->_setBounds(box);
	}
};

DummyController::DummyController(Ogre::RenderWindow *rt) :ImpUPtr_(new Imp)
{
	auto& imp_ = *ImpUPtr_;
	imp_.RT_ = rt;

	Ogre::Vector3 v1(1.f, 1.f, 1.f);
	v1.normalise();

	imp_.Smgr_ = Ogre::Root::getSingletonPtr()->createSceneManager(Ogre::ST_GENERIC, 1, Ogre::INSTANCING_CULLING_SINGLETHREAD, "DummyScene");

	auto camera = imp_.Smgr_->createCamera("MainCamera");
	camera->setAutoAspectRatio(true);

	auto comMgr = Ogre::Root::getSingletonPtr()->getCompositorManager2();

	imp_.WorkSpce_ = comMgr->addWorkspace(imp_.Smgr_, imp_.RT_, camera, "measureWorkspace", true);
	{
		auto bgMat = Ogre::MaterialManager::getSingleton().getByName("measureTool/Background");
		auto texUnit = bgMat->getTechnique(0)->getPass(0)->getTextureUnitState(0);

		Ogre::Image bgImage;
		{
			std::ifstream ifs("qml/res/bg.png", std::ios::binary);
			auto fhds = new Ogre::FileStreamDataStream(&ifs, false);

			Ogre::DataStreamPtr dataPtr(fhds);

			bgImage.load(dataPtr, "png");
		}

		auto texture = Ogre::TextureManager::getSingletonPtr()->createManual("BGTexture", "General",
																			 Ogre::TEX_TYPE_2D,
																			 bgImage.getWidth(), bgImage.getHeight(), 0,
																			 bgImage.getFormat(), Ogre::TU_STATIC);
		texture->getBuffer()->blitFromMemory(bgImage.getPixelBox());
		texUnit->setTexture(texture);
	}

	auto light = imp_.Smgr_->createLight();
	imp_.Smgr_->setAmbientLight(Ogre::ColourValue(1.f, 1.f, 1.f));
	auto lightNode = imp_.Smgr_->getRootSceneNode()->createChildSceneNode();
	lightNode->setPosition({ 2000.f, 8000.f, 5000.f });
	lightNode->attachObject(light);
	light->setDirection({ -1.f, -1.f, -1.f });
	light->setDiffuseColour(1.f, 1.f, 1.f);

	auto testMesh = Ogre::MeshManager::getSingleton().load("plyTest.mesh", "General");

	imp_.HumanEntity_ = imp_.Smgr_->createEntity(testMesh);
	imp_.HumanEntity_->addQueryFlags(EVF_QueryVisible);

	imp_.HumanEntity_->setMaterialName("Mat/Base/VertexColor");
	auto bounds = imp_.HumanEntity_->getMesh()->getBounds();
	auto scaleNode = imp_.Smgr_->getRootSceneNode()->createChildSceneNode();
	auto adjCenterNode = scaleNode->createChildSceneNode();
	adjCenterNode->attachObject(imp_.HumanEntity_);

	auto size = bounds.getSize();
	auto center = bounds.getCenter();

	auto deltaY = center.y - size.y / 2;
	adjCenterNode->setPosition(-center.x, -deltaY, -center.z);
	auto maxSize = std::max({ size.x, size.y, size.z });
	auto scale = 200 / maxSize;
	scaleNode->setScale({ scale, scale, scale });

	auto mayaCamera = std::make_shared<MayaCamera>(camera, imp_.RT_);
	mayaCamera->SetPosAndTarget({ 0.f, 150.f, 300.f }, { 0.f, 100.f, 0.f });

	camera->setNearClipDistance(10.f);
	camera->setFarClipDistance(1000.f);

	imp_.CameraListener_ = mayaCamera;

	imp_.BuildOctree(testMesh);

	imp_.SelectedMesh_ = Ogre::MeshManager::getSingleton().createManual("selected", "General");
	{
		imp_.SelectedMesh_->sharedVertexData = new Ogre::VertexData;
		auto subMesh = imp_.SelectedMesh_->createSubMesh();
		subMesh->operationType = Ogre::RenderOperation::OT_POINT_LIST;
		subMesh->useSharedVertices = true;

		auto decl = imp_.SelectedMesh_->sharedVertexData->vertexDeclaration;

		auto& posEle = decl->addElement(0, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);

		auto vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(decl->getVertexSize(0), MaxSelectedSize, Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY, true);
		imp_.SelectedMesh_->sharedVertexData->vertexBufferBinding->setBinding(0, vBuf);
		imp_.SelectedMesh_->sharedVertexData->vertexCount = MaxSelectedSize;
		imp_.SelectedMesh_->_setBounds(Ogre::AxisAlignedBox(Ogre::Vector3::ZERO, Ogre::Vector3::UNIT_SCALE));
	}

	imp_.SelecetedEntity_ = imp_.Smgr_->createEntity(imp_.SelectedMesh_);
	imp_.SelecetedEntity_->setMaterialName("measureTool/SelectedColor");
	adjCenterNode->attachObject(imp_.SelecetedEntity_);


	{//grid
		auto gridMesh = Ogre::MeshManager::getSingleton().createManual("grid", "General");
		{
			gridMesh->sharedVertexData = new Ogre::VertexData;
			auto subMesh = gridMesh->createSubMesh();
			subMesh->operationType = Ogre::RenderOperation::OT_TRIANGLE_FAN;
			subMesh->useSharedVertices = true;

			auto decl = gridMesh->sharedVertexData->vertexDeclaration;

			auto offset = 0;
			auto& posEle = decl->addElement(0, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
			offset += static_cast<decltype( offset )>( Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3) );
			auto& texEle = decl->addElement(0, offset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES);

			std::vector<std::tuple<Ogre::Vector3, Ogre::Vector2>> vList;
			auto halfSize = 0.5f;
			auto texScale = 25.f;
			vList.emplace_back(Ogre::Vector3(-halfSize, 0, -halfSize), Ogre::Vector2(0.f, texScale));
			vList.emplace_back(Ogre::Vector3(-halfSize, 0, halfSize), Ogre::Vector2(0.f, 0.f));
			vList.emplace_back(Ogre::Vector3(halfSize, 0, halfSize), Ogre::Vector2(texScale, 0.f));
			vList.emplace_back(Ogre::Vector3(halfSize, 0, -halfSize), Ogre::Vector2(texScale, texScale));

			auto vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(decl->getVertexSize(0), vList.size(), Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, true);
			{
				auto pBuf = static_cast<uint8_t*>( vBuf->lock(Ogre::HardwareBuffer::HBL_WRITE_ONLY) );
				float* pVal;

				for ( auto index = 0; index < vList.size(); ++index )
				{
					auto& curPnt = vList[index];
					auto& curPos = std::get<0>(curPnt);
					auto& curTex = std::get<1>(curPnt);

					posEle.baseVertexPointerToElement(pBuf, &pVal);
					*reinterpret_cast<Ogre::Vector3*>( pVal ) = curPos;

					texEle.baseVertexPointerToElement(pBuf, &pVal);
					*reinterpret_cast<Ogre::Vector2*>( pVal ) = curTex;

					pBuf += vBuf->getVertexSize();
				}
				vBuf->unlock();
			}

			gridMesh->sharedVertexData->vertexBufferBinding->setBinding(0, vBuf);
			gridMesh->sharedVertexData->vertexCount = vList.size();
			gridMesh->_updateBoundsFromVertexBuffers();
		}

		auto gridEn = imp_.Smgr_->createEntity(gridMesh);
		gridEn->setMaterialName("measureTool/Grid");

		auto node = imp_.Smgr_->getRootSceneNode()->createChildSceneNode();
		node->attachObject(gridEn);
		node->setScale(250, 1, 250);
	}
}

DummyController::~DummyController()
{
	auto& imp_ = *ImpUPtr_;

	Unload();

	imp_.CameraListener_.reset();

	auto comMgr = Ogre::Root::getSingletonPtr()->getCompositorManager2();
	comMgr->removeWorkspace(imp_.WorkSpce_);
	Ogre::Root::getSingletonPtr()->destroySceneManager(imp_.Smgr_);
}

void DummyController::_FrameQueue(const Ogre::FrameEvent& fevt)
{
	auto& imp_ = *ImpUPtr_;

	auto& evtRecorder = GetSysEventRecorder();

	auto ray = QueryUtil::GetRayFromCamera(imp_.CameraListener_->GetCamera(), SysEventRecorder::CachedMouseState(), imp_.RT_);

	Ogre::Vector3 origin, dir;
	{
		auto transform = imp_.HumanEntity_->getParentSceneNode()->_getFullTransformUpdated();
		auto transformInv = transform.inverse();
		auto rot = transformInv.extractQuaternion();

		origin = transformInv * ray.getOrigin();
		dir = rot * ray.getDirection();
	}
	auto endP = origin + dir;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::AlignedPointXYZVector vec;

	auto ret = imp_.Octree_->getIntersectedVoxelCenters(MathUtil::ToEigen(origin), MathUtil::ToEigen(dir), vec);
	if ( ret == 0 )
	{
		return;
	}

	std::vector<Ogre::Vector3> centerList;
	for ( auto& cur : vec )
	{
		centerList.push_back(MathUtil::ToOgre(cur));
	}

	std::map<float, std::tuple<SimpleBoundingBox*, Imp::OctreeType::LeafNode*>> bList;

	for ( auto& curBox : imp_.BBList_ )
	{
		auto box = std::get<0>(curBox);
		auto itor = std::get<1>(curBox);

		//box->setVisible(false);
		box->SetColor(Ogre::ColourValue::White);

		auto& bb = box->GetAABox();

		centerList.erase(std::remove_if(centerList.begin(), centerList.end(), [&](const Ogre::Vector3& center)
		{
			if ( bb.contains(center) )
			{
				bList.emplace(bb.getCenter().squaredDistance(origin), curBox);
				return true;
			}

			return false;
		}), centerList.end());
	}

	auto eps = 5e-3;
	auto squaredEps = eps * eps;
	auto egOrigin = MathUtil::ToEigen(origin);
	auto egEnd = MathUtil::ToEigen(endP);
	auto egDir = MathUtil::ToEigen(dir);

	auto& pc = *imp_.Octree_->getInputCloud();

	auto found = false;
	auto minSquVal = std::numeric_limits<float>::max();
	Ogre::Vector3 queryPnt;

	for ( auto& cur : bList )
	{
		auto box = std::get<0>(cur.second);
		auto leafNode = std::get<1>(cur.second);

		auto& leafContainer = leafNode->getContainer();
		auto& indices = leafContainer.getPointIndicesVector();
		for ( auto& curIndex : indices )
		{
			Eigen::Vector3f curPnt = pc[curIndex].getVector3fMap();
			auto projP = MathUtil::ProjectPointOnLine(curPnt, egOrigin, egDir);

			if ( ( projP - curPnt ).squaredNorm() > squaredEps )
			{
				continue;
			}
			found = true;

			auto curVal = ( curPnt - egOrigin ).squaredNorm();
			if ( minSquVal > curVal )
			{
				minSquVal = curVal;
				queryPnt = MathUtil::ToOgre(curPnt);
			}
		}

		if ( found )
		{
			break;
		}
	}

	if ( found )
	{
		Ogre::Ray ray(origin, dir);
		auto plnPnt = ray.getPoint(std::sqrt(minSquVal));
		Ogre::Plane pln(Ogre::Vector3::UNIT_Y, plnPnt);
		auto dirPnt = plnPnt + pln.normal;

		std::vector<SimpleBoundingBox*> plnBoxList;

		pcl::PointCloud<pcl::PointXYZ>::Ptr toCluster(new pcl::PointCloud<pcl::PointXYZ>);
		for ( auto& cur : imp_.BBList_ )
		{
			auto curB = std::get<0>(cur);
			auto curVoxel = std::get<1>(cur);
			auto& box = curB->GetAABox();
			auto boxCenter = box.getCenter();
			if ( Ogre::Math::intersects(pln, box) )
			{
				auto& indices = curVoxel->getContainer().getPointIndicesVector();
				auto& curPntCloud = imp_.Octree_->getInputCloud()->points;
				for ( auto curIndex : indices )
				{
					auto curPnt = MathUtil::ToOgre(curPntCloud[curIndex]);
					auto plnToPnt = curPnt - plnPnt;
					auto dis = plnToPnt.dotProduct(pln.normal);
 					if ( std::abs(dis) > 5e-3 )
 					{
 						continue;
 					}
					auto projectPnt = curPnt - pln.normal * dis;

					toCluster->points.push_back(MathUtil::ToPCL(projectPnt));
				}

				plnBoxList.push_back(curB);
			}
		}

		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(0.01);
		ec.setMinClusterSize(100);
		ec.setInputCloud(toCluster);

		std::vector<pcl::PointIndices> cluster_indices;
		ec.extract(cluster_indices);

		std::multimap<float, std::vector<Ogre::Vector3>> clusterList;
		for ( auto& cur : cluster_indices )
		{
			Ogre::AxisAlignedBox curBox;
			std::vector<Ogre::Vector3> curList;
			for ( auto& curIndex : cur.indices )
			{
				auto curPnt = MathUtil::ToOgre(toCluster->points[curIndex]);
				curBox.merge(curPnt);
				curList.push_back(curPnt);
			}
			auto curDis = curBox.distance(queryPnt);
			clusterList.emplace(curDis, curList);
		}

		if ( clusterList.empty() )
		{
			std::vector<Ogre::Vector3> saveList;
			for ( auto& curPnt : toCluster->points )
			{
				saveList.push_back(MathUtil::ToOgre(curPnt));
			}
			imp_.SaveToPly(saveList, "error.ply");

			Ogre::LogManager::getSingleton().logMessage("error cluster");

			imp_.SelecetedEntity_->setVisible(false);
		}
		else
		{
			auto itor = clusterList.begin();

			if ( itor->first < 1e-2 )
			{
				auto& pntList = itor->second;

				if ( evtRecorder.HasMousePressed(OIS::MB_Right) )
				{
					imp_.SaveToPly(pntList, "save.ply");
					imp_.UpdateSelected(pntList, true);
				}
				else
				{
					imp_.UpdateSelected(pntList, false);
				}

				imp_.SelecetedEntity_->setVisible(true);
			}
			else
			{
				imp_.SelecetedEntity_->setVisible(false);
			}
		}
	}
	else
	{
		imp_.SelecetedEntity_->setVisible(false);
	}
}
