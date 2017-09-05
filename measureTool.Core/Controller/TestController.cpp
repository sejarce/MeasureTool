#pragma warning(disable:4996)

#include "TestController.h"

#include "Render/Camera/MayaCamera.h"
#include "Render/Extension/RadiusRaySceneQuery.h"
#include "Render/QueryUtil.h"
#include "Render/VisibilityFlag.h"
#include "Render/Extension/SimpleBoundingBox.h"
#include "Render/Extension/Line3D.h"
#include "Render/Extension/Point3D.h"
#include "Render/Extension/Circle3D.h"
#include "Render/Extension/Torus.h"

#include "Util/MathUtil.h"
#include "Util/MeshUtil.h"
#include "Util/CpuTimer.h"

#include "Ogre.h"
#include "Compositor/OgreCompositorManager2.h"
#include "Compositor/OgreCompositorWorkspace.h"
#include "OgreMeshSerializer.h"

#include "TColgp_Array1OfPnt.hxx"
#include "GeomAPI_PointsToBSpline.hxx"
#include "Geom_BSplineCurve.hxx"

#include <boost/filesystem/fstream.hpp>
#include <boost/algorithm/string.hpp>

class 	TestController::Imp
{

public:

	Ogre::RenderWindow*			RT_{};
	MayaCameraSPtr 				CameraListener_{};
	Ogre::SceneManager*			Smgr_{};
	Ogre::CompositorWorkspace*	WorkSpce_{};

public:

	static void	CreatePointCloud(const std::wstring& filePath, const std::string& meshName)
	{
		boost::filesystem::ifstream ifs(filePath);

		std::string line;
		std::vector<std::string> split;

		std::vector<Ogre::Vector3> vlist;
		std::vector<int> ilist;

		auto vCount = 0, fCount = 0;
		for ( auto index = 1; index <= 10; ++index )
		{
			std::getline(ifs, line);

			if ( index == 4 )
			{
				boost::algorithm::split(split, line, boost::algorithm::is_any_of(" "));
				split.erase(std::remove_if(split.begin(), split.end(), [](const std::string& str)
				{
					return str.empty();
				}), split.end());

				vCount = std::stoi(split[2]);

				line.clear();
				split.clear();
			}

			if ( index == 8 )
			{
				boost::algorithm::split(split, line, boost::algorithm::is_any_of(" "));
				split.erase(std::remove_if(split.begin(), split.end(), [](const std::string& str)
				{
					return str.empty();
				}), split.end());

				fCount = std::stoi(split[2]);

				line.clear();
				split.clear();
			}
		}

		for ( auto index = 0; index < vCount; ++index )
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

			vlist.emplace_back(px, py, pz);
		}

		for ( auto index = 0; index < fCount; ++index )
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

			auto f0 = std::stoi(split[1]);
			auto f1 = std::stoi(split[2]);
			auto f2 = std::stoi(split[3]);

			ilist.push_back(f0);
			ilist.push_back(f1);
			ilist.push_back(f2);
		}

		auto lineMesh = Ogre::MeshManager::getSingleton().createManual(meshName, "General");
		{
			lineMesh->sharedVertexData = new Ogre::VertexData;
			lineMesh->sharedVertexData->vertexCount = vlist.size();
			auto decl = lineMesh->sharedVertexData->vertexDeclaration;

			auto subMesh = lineMesh->createSubMesh();
			subMesh->operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
			subMesh->useSharedVertices = true;

			auto& posEle = decl->addElement(0, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);

			auto vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(decl->getVertexSize(0), vlist.size(), Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
			{
				auto pBuf = static_cast<uint8_t*>( vBuf->lock(Ogre::HardwareBuffer::HBL_WRITE_ONLY) );
				float* pVal;

				for ( auto index = 0; index < vlist.size(); ++index )
				{
					auto& curPnt = vlist[index];

					posEle.baseVertexPointerToElement(pBuf, &pVal);
					*reinterpret_cast<Ogre::Vector3*>( pVal ) = curPnt;

					pBuf += vBuf->getVertexSize();
				}

				vBuf->unlock();
			}

			lineMesh->sharedVertexData->vertexBufferBinding->setBinding(0, vBuf);
			lineMesh->_updateBoundsFromVertexBuffers();

			auto iBuf = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(Ogre::HardwareIndexBuffer::IT_16BIT, ilist.size(), Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
			{
				auto pBuf = static_cast<short*>( iBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD) );

				for ( auto cur : ilist )
				{
					*pBuf = cur;
					++pBuf;
				}

				iBuf->unlock();
			}

			subMesh->indexData = new Ogre::IndexData;
			subMesh->indexData->indexBuffer = iBuf;
			subMesh->indexData->indexCount = iBuf->getNumIndexes();
		}

		Ogre::MeshSerializer ms;
		ms.exportMesh(lineMesh.get(), meshName + ".mesh", Ogre::MESH_VERSION_1_8);
	}

public:

	void	CreateGrid()
	{
		auto mesh = Ogre::MeshManager::getSingleton().createManual("grid", "General");
		{
			mesh->sharedVertexData = new Ogre::VertexData;
			auto subMesh = mesh->createSubMesh();
			subMesh->operationType = Ogre::RenderOperation::OT_TRIANGLE_FAN;
			subMesh->useSharedVertices = true;

			auto decl = mesh->sharedVertexData->vertexDeclaration;

			auto offset = 0;
			auto& posEle = decl->addElement(0, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
			offset += static_cast<decltype( offset )>( Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3) );
			auto& texEle = decl->addElement(0, offset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES);

			std::vector<std::tuple<Ogre::Vector3, Ogre::Vector2>> vList;
			auto halfSize = 0.5f;
			auto texScale = 3.f;
			vList.emplace_back(Ogre::Vector3(-halfSize, 0, -halfSize), Ogre::Vector2(0.f, texScale));
			vList.emplace_back(Ogre::Vector3(-halfSize, 0, halfSize), Ogre::Vector2(0.f, 0.f));
			vList.emplace_back(Ogre::Vector3(halfSize, 0, halfSize), Ogre::Vector2(texScale, 0.f));
			vList.emplace_back(Ogre::Vector3(halfSize, 0, -halfSize), Ogre::Vector2(texScale, texScale));

			auto vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(decl->getVertexSize(0), vList.size(), Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY, true);
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

			mesh->sharedVertexData->vertexBufferBinding->setBinding(0, vBuf);
			mesh->sharedVertexData->vertexCount = vList.size();
			mesh->_updateBoundsFromVertexBuffers();
		}

		auto en = Smgr_->createEntity(mesh);
		en->setMaterialName("measureTool/Grid");

		auto node = Smgr_->getRootSceneNode()->createChildSceneNode();
		node->attachObject(en);
		node->setScale(50, 50, 50);
	}


	void	CreateBillBoard()
	{
		auto billboardSet = Smgr_->createBillboardSet();
		billboardSet->setBillboardType(Ogre::BBT_ORIENTED_COMMON);
		billboardSet->setCommonDirection(Ogre::Vector3::UNIT_Y);
		billboardSet->setMaterialName("measureTool/Arrow");
		billboardSet->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN + 1);
		auto billboard = billboardSet->createBillboard(Ogre::Vector3(0, 50, 0));
		billboard->setDimensions(50, 50);
		auto node = Smgr_->getRootSceneNode()->createChildSceneNode();
		node->attachObject(billboardSet);
	}

	void	CreateCircle()
	{
		auto zCircle = Circle3DFactory::CreateInstance(Smgr_);
		zCircle->SetColor(Ogre::ColourValue(1.f, 1.f, 0.f, .5f));

		auto xCircle = Circle3DFactory::CreateInstance(Smgr_);
		xCircle->SetColor(Ogre::ColourValue(1.f, 0.f, 1.f, .5f));

		auto rotZNode = Smgr_->getRootSceneNode()->createChildSceneNode();
		{
			rotZNode->attachObject(zCircle);
		}
		auto showXNode = rotZNode->createChildSceneNode();
		{
			showXNode->setOrientation(Ogre::Vector3::UNIT_Z.getRotationTo(Ogre::Vector3::UNIT_X));
			showXNode->attachObject(xCircle);
		}
		auto rotXNode = showXNode->createChildSceneNode();
		rotZNode->setScale(50, 50, 50);

		xCircle->SetDegree(360);
		zCircle->SetDegree(360);

		auto func = [&](int rotx, int rotz)
		{
			xCircle->SetDegree(rotx);
			{
				Ogre::Quaternion xRot;
				Ogre::Degree deg(rotx);
				xRot.FromAngleAxis(deg, Ogre::Vector3::UNIT_Z);
				rotXNode->setOrientation(xRot);
			}
			
			zCircle->SetDegree(rotz);
			{
				Ogre::Quaternion zRot;
				Ogre::Degree deg(rotz);
				zRot.FromAngleAxis(deg, Ogre::Vector3::UNIT_Z);
				rotZNode->setOrientation(zRot);
			}
		};

		func(45, 45);
	}

	void	CreateTorus()
	{
		auto en = TorusFactory::CreateInstance(Smgr_);

		auto node = Smgr_->getRootSceneNode()->createChildSceneNode();
		node->attachObject(en);
		node->setScale(50, 50, 50);
	}

	void	TestPipe()
	{
		std::vector<Ogre::Vector3> pntList;
		{
			for ( auto deg = 0; deg < 360; deg += 20 )
			{
				Ogre::Degree od(deg);
				pntList.emplace_back(std::cos(od.valueRadians()), std::sin(od.valueRadians()), 0.f);
			}
		}

		auto build = MeshUtil::BuildPipe(pntList, 6, .05f, true);
		auto& ver = std::get<0>(build);
		auto& face = std::get<1>(build);
		
		boost::filesystem::ofstream ofs("test2.ply", std::ios::trunc);
		ofs << "ply\n";
		ofs << "format ascii 1.0\n";
		ofs << "element vertex " << ver.size() << "\n";
		ofs << "property float x\n";
		ofs << "property float y\n";
		ofs << "property float z\n";
		ofs << "element face " << face.size()/3 << "\n";
		ofs << "property list uchar int vertex_indices\n";
		ofs << "end_header\n";

		for ( auto& cur : ver )
		{
			ofs << cur.x << " " << cur.y << " " << cur.z << "\n";
		}

		for ( auto index = 0; index < face.size(); index += 3 )
		{
			ofs << 3 << " " << face[index] << " " << face[index + 1] << " " << face[index + 2] << "\n";
		}
		
	}

	void	TestShape()
	{
		auto info = MeshUtil::LoadPly(L"param.ply");
		auto& verList = std::get<0>(info);

		CpuTimer ct;

		TColgp_Array1OfPnt pntList(1, verList.size());
		{
			auto index = 0;
			for ( auto& cur : verList )
			{
				pntList.SetValue(index + 1, { cur.x, cur.y, cur.z });
				++index;
			}
		}

		GeomAPI_PointsToBSpline approx(pntList, 3, 3, GeomAbs_G1, 5e-3);
		auto k = approx.Curve();

		auto fp = k->FirstParameter();
		auto lp = k->LastParameter();

		std::vector<Ogre::Vector3> newPntList;
		newPntList.reserve(100);
		auto step = ( lp - fp ) / newPntList.capacity();
		gp_Pnt pnt;
		for ( auto index = 0; index < newPntList.capacity(); ++index )
		{
			k->D0(step*index, pnt);
			newPntList.emplace_back(static_cast<float>( pnt.X() ), static_cast<float>( pnt.Y() ), static_cast<float>( pnt.Z() ));
		}

		auto buildInfo = MeshUtil::BuildPipe(newPntList, 6, .002, true);

		ct.Print();

		MeshUtil::SaveToPly(L"build.ply", std::get<0>(buildInfo), std::get<1>(buildInfo));
		

		auto i = 0;
	}

	void	TestWholeBody(const std::wstring& filePath, const std::string& meshName)
	{
		std::vector<std::tuple<Ogre::Vector3, Ogre::ColourValue>> vlist;
		std::map<int, decltype( vlist )> splitVlist;
		{
			boost::filesystem::ifstream ifs(filePath);

			std::string line;
			std::vector<std::string> split;

			while ( ifs )
			{
				line.clear();
				split.clear();

				std::getline(ifs, line);
				if ( line.empty() )
				{
					continue;
				}

				boost::algorithm::split(split, line, boost::algorithm::is_any_of(","));
				split.erase(std::remove_if(split.begin(), split.end(), [](const std::string& str)
				{
					return str.empty();
				}), split.end());

				if ( split.empty() )
				{
					continue;
				}

				Ogre::Vector3 pos;
				{
					pos.x = std::stof(split[4]);
					pos.z = -std::stof(split[5]);
					pos.y = std::stof(split[6]);
				}

				Ogre::ColourValue clr;
				{
					clr.r = std::stof(split[7]);
					clr.g = std::stof(split[8]);
					clr.b = std::stof(split[9]);
				}

				vlist.emplace_back(pos, clr);

				auto partID = std::stoi(split[2]);
				auto& curPartVList = splitVlist[partID];
				curPartVList.emplace_back(pos, clr);
			}
		}

		{
			boost::filesystem::ofstream ofs(meshName + ".ply", std::ios::trunc);

			ofs << "ply\n";
			ofs << "format ascii 1.0\n";
			ofs << "element vertex " << vlist.size() << "\n";
			ofs << "property float x\n";
			ofs << "property float y\n";
			ofs << "property float z\n";
			ofs << "property uchar red\n";
			ofs << "property uchar green\n";
			ofs << "property uchar blue\n";
			ofs << "end_header\n";
			for ( auto& cur : vlist )
			{
				auto& pos = std::get<0>(cur);
				auto& clr = std::get<1>(cur);

				auto cr = static_cast<int>( clr.r * 255 );
				auto cg = static_cast<int>( clr.g * 255 );
				auto cb = static_cast<int>( clr.b * 255 );

				ofs << pos[0] << " " << pos[1] << " " << pos[2] << " " << cr << " " << cg << " " << cb << "\n";
			}
		}

		for ( auto& curPart : splitVlist )
		{
			boost::filesystem::ofstream ofs(meshName + "_" + std::to_string(curPart.first) + ".ply", std::ios::trunc);

			ofs << "ply\n";
			ofs << "format ascii 1.0\n";
			ofs << "element vertex " << splitVlist.size() << "\n";
			ofs << "property float x\n";
			ofs << "property float y\n";
			ofs << "property float z\n";
			ofs << "property uchar red\n";
			ofs << "property uchar green\n";
			ofs << "property uchar blue\n";
			ofs << "end_header\n";
			for ( auto& cur : curPart.second )
			{
				auto& pos = std::get<0>(cur);
				auto& clr = std::get<1>(cur);

				auto cr = static_cast<int>( clr.r * 255 );
				auto cg = static_cast<int>( clr.g * 255 );
				auto cb = static_cast<int>( clr.b * 255 );

				ofs << pos[0] << " " << pos[1] << " " << pos[2] << " " << cr << " " << cg << " " << cb << "\n";
			}
		}

		auto mesh = Ogre::MeshManager::getSingleton().createManual(meshName, "General");
		{
			mesh->sharedVertexData = new Ogre::VertexData;
			mesh->sharedVertexData->vertexCount = vlist.size();
			auto decl = mesh->sharedVertexData->vertexDeclaration;

			auto subMesh = mesh->createSubMesh();
			subMesh->operationType = Ogre::RenderOperation::OT_POINT_LIST;
			subMesh->useSharedVertices = true;

			auto& posEle = decl->addElement(0, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
			auto& clrEle = decl->addElement(0, Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3), Ogre::VET_COLOUR_ARGB, Ogre::VES_DIFFUSE);

			auto vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(decl->getVertexSize(0), vlist.size(), Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
			{
				auto pBuf = static_cast<uint8_t*>( vBuf->lock(Ogre::HardwareBuffer::HBL_WRITE_ONLY) );
				float* pVal;
				Ogre::RGBA* pClrVal;
				for ( auto index = 0; index < vlist.size(); ++index )
				{
					auto& curPnt = std::get<0>(vlist[index]);
					auto& curClr = std::get<1>(vlist[index]);

					posEle.baseVertexPointerToElement(pBuf, &pVal);
					*reinterpret_cast<Ogre::Vector3*>( pVal ) = curPnt;

					clrEle.baseVertexPointerToElement(pBuf, &pClrVal);
					*pClrVal = curClr.getAsARGB();

					pBuf += vBuf->getVertexSize();
				}

				vBuf->unlock();
			}

			mesh->sharedVertexData->vertexBufferBinding->setBinding(0, vBuf);
			mesh->_updateBoundsFromVertexBuffers();
		}

		Ogre::MeshSerializer ms;
		ms.exportMesh(mesh.get(), meshName + ".mesh", Ogre::MESH_VERSION_1_8);
	}

	void	TestResult(const std::wstring& filePath)
	{
		std::map<int, std::vector<Ogre::Vector3>> resultMap;
		{
			boost::filesystem::ifstream ifs(filePath);

			std::string line;
			std::vector<std::string> split;

			while ( ifs )
			{
				line.clear();
				split.clear();

				std::getline(ifs, line);
				if ( line.empty() )
				{
					continue;
				}

				boost::algorithm::split(split, line, boost::algorithm::is_any_of(","));
				split.erase(std::remove_if(split.begin(), split.end(), [](const std::string& str)
				{
					return str.empty();
				}), split.end());

				if ( split.empty() )
				{
					continue;
				}

				auto index = std::stoi(split[0]);

				Ogre::Vector3 pos;
				{
					pos.x = std::stof(split[2]);
					pos.z = -std::stof(split[3]);
					pos.y = std::stof(split[4]);
				}

				auto& vList = resultMap[index];

				vList.emplace_back(pos);
			}
		}

		std::map<int, std::string> nameMap;
		{
			nameMap.emplace(1, "body_height");
			nameMap.emplace(2, "chest_width");
			nameMap.emplace(3, "bace_width");
			nameMap.emplace(4, "shoulder_width");
			nameMap.emplace(5, "chest_dimension");
			nameMap.emplace(6, "waist_dimension");
			nameMap.emplace(7, "wrist_dimension");
			nameMap.emplace(8, "knee_dimension");
			nameMap.emplace(9, "hip_dimension");
			nameMap.emplace(10, "arm_dimension");
			nameMap.emplace(11, "arm_length");
			nameMap.emplace(12, "bp_distance");
			nameMap.emplace(13, "leg_dimension");
		}

		auto noneCount = 0;
		for ( auto& curRes : resultMap )
		{
			auto name = nameMap[curRes.first];
			if ( name.empty() )
			{
				name = "none_" + std::to_string(noneCount++);
			}

			auto& vList = curRes.second;

			{
				boost::filesystem::ofstream ofs(name + ".ply", std::ios::trunc);

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

			{
				auto mesh = Ogre::MeshManager::getSingleton().createManual(name, "General");

				mesh->sharedVertexData = new Ogre::VertexData;
				mesh->sharedVertexData->vertexCount = vList.size();
				auto decl = mesh->sharedVertexData->vertexDeclaration;

				auto subMesh = mesh->createSubMesh();
				subMesh->operationType = Ogre::RenderOperation::OT_POINT_LIST;
				subMesh->useSharedVertices = true;

				auto& posEle = decl->addElement(0, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);

				auto vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(decl->getVertexSize(0), vList.size(), Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
				{
					auto pBuf = static_cast<uint8_t*>( vBuf->lock(Ogre::HardwareBuffer::HBL_WRITE_ONLY) );
					float* pVal;
					for ( auto index = 0; index < vList.size(); ++index )
					{
						auto& curPnt = vList[index];

						posEle.baseVertexPointerToElement(pBuf, &pVal);
						*reinterpret_cast<Ogre::Vector3*>( pVal ) = curPnt;

						pBuf += vBuf->getVertexSize();
					}

					vBuf->unlock();
				}

				mesh->sharedVertexData->vertexBufferBinding->setBinding(0, vBuf);
				mesh->_updateBoundsFromVertexBuffers();

				Ogre::MeshSerializer ms;
				ms.exportMesh(mesh.get(), name + ".mesh", Ogre::MESH_VERSION_1_8);
			}
		}
	}
};

TestController::TestController(Ogre::RenderWindow *rt) :ImpUPtr_(new Imp)
{
	auto& imp_ = *ImpUPtr_;

	imp_.RT_ = rt;

	imp_.Smgr_ = Ogre::Root::getSingletonPtr()->createSceneManager(Ogre::ST_GENERIC, 1, Ogre::INSTANCING_CULLING_SINGLETHREAD, "TestSceneListener");

	auto camera = imp_.Smgr_->createCamera("MainCamera");
	camera->setAutoAspectRatio(true);

	auto comMgr = Ogre::Root::getSingletonPtr()->getCompositorManager2();
	comMgr->createBasicWorkspaceDef("TestSceneListener", Ogre::ColourValue(.5, .5, .5));

	imp_.WorkSpce_ = comMgr->addWorkspace(imp_.Smgr_, imp_.RT_, camera, "TestSceneListener", true);

	auto mayaCamera = std::make_shared<MayaCamera>(camera, imp_.RT_);
	mayaCamera->SetPosAndTarget({ 0, 100, 150.f }, { 0.f, 0.f, 0.f });

	camera->setNearClipDistance(10.f);
	camera->setFarClipDistance(1000.f);

	imp_.CreateGrid();

	//imp_.TestResult(LR"(C:\Users\Administrator\Desktop\手工测量工具\测量结果参考\measurementResult.txt)");
	MeshUtil::LoadTestMesh(LR"(C:\Users\Administrator\Desktop\手工测量工具\TestData\男\1\)", "male_1", true);
	MeshUtil::LoadTestMesh(LR"(C:\Users\Administrator\Desktop\手工测量工具\TestData\男\2\)", "male_2", true);
	MeshUtil::LoadTestMesh(LR"(C:\Users\Administrator\Desktop\手工测量工具\TestData\女\1\)", "female_1", true);
	MeshUtil::LoadTestMesh(LR"(C:\Users\Administrator\Desktop\手工测量工具\TestData\女\2\)", "female_2", true);
	

	imp_.CameraListener_ = mayaCamera;
}

TestController::~TestController()
{
	auto& imp_ = *ImpUPtr_;

	Unload();

	imp_.CameraListener_.reset();

	auto comMgr = Ogre::Root::getSingletonPtr()->getCompositorManager2();
	comMgr->removeWorkspace(imp_.WorkSpce_);
	Ogre::Root::getSingletonPtr()->destroySceneManager(imp_.Smgr_);
}

void TestController::_FrameQueue(const Ogre::FrameEvent& fevt)
{
	auto& imp_ = *ImpUPtr_;

	auto& evtRecorder = GetSysEventRecorder();

	auto ray = QueryUtil::GetRayFromCamera(imp_.CameraListener_->GetCamera(), SysEventRecorder::CachedMouseState(), imp_.RT_);
}
