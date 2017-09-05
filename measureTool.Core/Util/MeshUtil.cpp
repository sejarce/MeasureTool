#include "MeshUtil.h"
#include "MathUtil.h"

#include "proto/YZMPointCloud.pb.h"

#include "Ogre.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/algorithm/string.hpp>

class	MeshUtilImp
{
public:

	using	PlyVector = std::vector<std::tuple<Ogre::Vector3, Ogre::ColourValue>>;

public:

	static PlyVector	LoadPlyPointCloud(boost::filesystem::ifstream& ifs)
	{
		std::string line;
		std::vector<std::string> split;

		std::vector<std::tuple<Ogre::Vector3, Ogre::ColourValue>> ret;

		auto verSize = 0;

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

			if ( split.empty() )
			{
				continue;
			}

			if ( split.size() == 1 && split[0] == "end_header" )
			{
				break;
			}

			if ( split.size() == 3 && split[0] == "element"
				&& split[1] == "vertex" )
			{
				verSize = std::stoi(split[2]);
				ret.reserve(verSize);
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

			auto cr = 255;
			auto cg = 255;
			auto cb = 255;

			if ( split.size() > 3 )
			{
				auto cr = std::stof(split[3]);
				auto cg = std::stof(split[4]);
				auto cb = std::stof(split[5]);
			}

			ret.emplace_back(Ogre::Vector3(px, py, pz), Ogre::ColourValue(cr / 255.f, cg / 255.f, cb / 255.f));

			if ( ret.size() == verSize )
			{
				break;
			}
		}

		return ret;
	}

	static PlyVector	LoadTxtPointCloud(boost::filesystem::ifstream& ifs)
	{
		std::string line;
		std::vector<std::string> split;

		std::vector<std::tuple<Ogre::Vector3, Ogre::ColourValue>> ret;

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

			auto px = std::stof(split[2]);
			auto py = std::stof(split[4]);
			auto pz = -std::stof(split[3]);
			
			auto cr = std::stof(split[5]);
			auto cg = std::stof(split[6]);
			auto cb = std::stof(split[7]);

			ret.emplace_back(Ogre::Vector3(px, py, pz), Ogre::ColourValue(cr, cg, cb));
		}

		return ret;
	}

	static void	SaveToPly(const PlyVector& vList, const std::string& fileName)
	{
		boost::filesystem::ofstream ofs(fileName, std::ios::trunc);

		ofs << "ply\n";
		ofs << "format ascii 1.0\n";
		ofs << "element vertex " << vList.size() << "\n";
		ofs << "property float x\n";
		ofs << "property float y\n";
		ofs << "property float z\n";
		ofs << "property uchar red\n";
		ofs << "property uchar green\n";
		ofs << "property uchar blue\n";
		ofs << "end_header\n";
		for ( auto& cur : vList )
		{
			auto& pos = std::get<0>(cur);
			auto& clr = std::get<1>(cur);

			auto cr = static_cast<int>( clr.r * 255 );
			auto cg = static_cast<int>( clr.g * 255 );
			auto cb = static_cast<int>( clr.b * 255 );

			ofs << pos[0] << " " << pos[1] << " " << pos[2] << " " << cr << " " << cg << " " << cb << "\n";
		}
	}
};

Ogre::MeshPtr MeshUtil::LoadTestMesh(const std::wstring folerPath, const std::string meshName, bool save)
{
	auto retMesh = Ogre::MeshManager::getSingleton().createManual(meshName, "General");

	MeshUtilImp::PlyVector totalList;

	for ( auto index = 1; index <= 20; ++index )
	{
		auto path = folerPath + std::to_wstring(index) + L"bgFiltered.txt";
		boost::filesystem::ifstream ifs(path);

		auto vList = MeshUtilImp::LoadTxtPointCloud(ifs);
		std::copy(vList.begin(), vList.end(), std::back_inserter(totalList));

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

	if ( save )
	{
		Ogre::MeshSerializer ms;
		ms.exportMesh(retMesh.get(), meshName +".mesh", Ogre::MESH_VERSION_1_8);

		MeshUtilImp::SaveToPly(totalList, meshName + ".ply");
	}
	

	return retMesh;
}

void MeshUtil::SaveToPly(const std::wstring& filePath, const std::vector<Ogre::Vector3>& verList, const std::vector<int>& indList)
{
	boost::filesystem::ofstream ofs(filePath, std::ios::trunc);

	ofs << "ply\n";
	ofs << "format ascii 1.0\n";
	ofs << "element vertex " << verList.size() << "\n";
	ofs << "property float x\n";
	ofs << "property float y\n";
	ofs << "property float z\n";
	ofs << "element face " << indList.size() / 3 << "\n";
	ofs << "property list uchar int vertex_indices\n";
	ofs << "end_header\n";

	for ( auto& cur : verList )
	{
		ofs << cur.x << " " << cur.y << " " << cur.z << "\n";
	}

	for ( auto index = 0; index < indList.size(); index += 3 )
	{
		ofs << 3 << " " << indList[index] << " " << indList[index + 1] << " " << indList[index + 2] << "\n";
	}
}

std::tuple<std::vector<Ogre::Vector3>, std::vector<int>> MeshUtil::LoadPly(const std::wstring& fileName)
{
	std::vector<Ogre::Vector3> verList;
	std::vector<int> indList;

	boost::filesystem::ifstream ifs(fileName);
	auto info = MeshUtilImp::LoadPlyPointCloud(ifs);

	verList.reserve(info.size());
	for ( auto& cur : info )
	{
		verList.push_back(std::get<0>(cur));
	}

	return std::make_tuple(verList, indList);
}

class	LineInfo
{
public:

	Ogre::Vector3	P1;
	Ogre::Vector3	P2;
	Ogre::Vector3	Dir;
	Ogre::Plane		Pln1;
	Ogre::Plane		Pln2;
	float			len;

	Ogre::Vector3	pm2;	//¬‰”⁄÷±œﬂ”“±ﬂ
};


std::tuple<std::vector<Ogre::Vector3>, std::vector<int>> MeshUtil::BuildPipe(const std::vector<Ogre::Vector3>& pntList, int segment, float radius, bool closed)
{
	std::vector<Ogre::Vector3> buildVertexList;
	std::vector<int> buildIndexList;

	std::vector<LineInfo> lines;
	std::adjacent_find(pntList.begin(), pntList.end(), [&](const Ogre::Vector3& v1, const Ogre::Vector3& v2)
	{
		LineInfo li;
		li.P1 = v1;
		li.P2 = v2;
		li.len = ( v2 - v1 ).length();
		li.Dir = ( v2 - v1 ).normalisedCopy();
		lines.push_back(li);

		return false;
	});

	/*
	if ( closed )
	{
		LineInfo li;
		li.P1 = pntList.back();
		li.P2 = pntList.front();
		li.Dir = ( li.P2 - li.P1 ).normalisedCopy();

		lines.push_back(li);
	}
	*/
	auto rereferv1 = lines[0].P1 - lines[lines.size() / 2].P1;
	auto rereferv2 = lines[lines.size() - 1].P1 - lines[lines.size() / 2].P1;
	auto referNormal = rereferv1.crossProduct(rereferv2).normalisedCopy();
	
	//change by aoj 2017/06/20
	std::adjacent_find(lines.begin(), lines.end(), [&referNormal](LineInfo& l1, LineInfo& l2)
	{
		auto l21 = (l2.P2 - l1.P1).normalisedCopy();//(l1.Dir + l2.Dir).normalisedCopy();
		auto len = (l2.P2 - l1.P1).length();
		auto pos = l21*(l1.len * len / (l1.len + l2.len)) + l1.P1;
		auto pm = (pos - l1.P2).normalisedCopy();

		auto pt = l2.Dir.crossProduct(l1.Dir);

		if (pt.dotProduct(referNormal) < 0)
			l1.pm2 = -pm;
		else
			l1.pm2 = pm;

		auto plnNormal = pt.crossProduct(pm).normalisedCopy();

		Ogre::Plane pln(plnNormal, l1.P2);
		l1.Pln2 = pln;
		l2.Pln1 = pln;

		return false;
	});

	if ( closed )
	{
		auto& l1 = lines.back();
		auto& l2 = lines.front();

		auto l21 = (l2.P2 - l1.P1).normalisedCopy(); //(l1.Dir + l2.Dir).normalisedCopy();
		auto len = (l2.P2 - l1.P1).length();
		auto pos = l21*(l1.len * len / (l1.len + l2.len)) + l1.P1;
		auto pm = (pos - l1.P2).normalisedCopy();
		auto pt = l2.Dir.crossProduct(l1.Dir);

		if (pt.dotProduct(referNormal) < 0)
			l1.pm2 = -pm;
		else
			l1.pm2 = pm;

		auto plnNormal = pt.crossProduct(pm).normalisedCopy();

		Ogre::Plane pln(plnNormal, l1.P2);
		l1.Pln2 = pln;
		l2.Pln1 = pln;
	}
	else
	{
		lines.front().Pln1.redefine(lines.front().Dir, lines.front().P1);
		lines.back().Pln2.redefine(lines.back().Dir, lines.back().P2);
	}

	//////////////////////////////////////////////////////////////////////////
	
	auto step = 360 / segment;

	for (auto curIndex = 0U; curIndex < lines.size(); ++curIndex)
	{
		auto& curLine = lines[curIndex];

		if (closed)
		{
			auto pln2 = curLine.Pln2;
			auto startVec = curLine.pm2;


			if (curIndex == 0)
			{
				auto pln1 = curLine.Pln1;
				auto startVec = lines.back().pm2;

				for (auto index = 0; index < segment; ++index)
				{
					Ogre::Degree deg(index * step);
					Ogre::Quaternion rot;
					rot.FromAngleAxis(deg, pln1.normal);

					auto vec = rot * startVec;
					auto ver0 = curLine.P1 + vec*radius;
					buildVertexList.emplace_back(ver0);
				}
			}
			if (curIndex != lines.size() - 1)
			{
				for (auto index = 0; index < segment; ++index)
				{
					Ogre::Degree deg(index * step);
					Ogre::Quaternion rot;
					rot.FromAngleAxis(deg, pln2.normal);

					auto vec = rot * startVec;
					auto ver0 = curLine.P2 + vec*radius;
					buildVertexList.emplace_back(ver0);
				}
			}
		}
		else
		{
			if (curIndex == 0)
			{
				auto pln1 = curLine.Pln1;
				auto startVec = curLine.pm2;

				for (auto index = 0; index < segment; ++index)
				{
					Ogre::Degree deg(index * step);
					Ogre::Quaternion rot;
					rot.FromAngleAxis(deg, pln1.normal);

					auto vec = rot * startVec;
					auto ver0 = curLine.P2 + vec*radius;
					buildVertexList.emplace_back(ver0);
				}
			}
			if (curIndex != lines.size() - 1)
			{
				auto pln2 = curLine.Pln2;
				auto startVec = curLine.pm2;

				for (auto index = 0; index < segment; ++index)
				{
					Ogre::Degree deg(index * step);
					Ogre::Quaternion rot;
					rot.FromAngleAxis(deg, pln2.normal);

					auto vec = rot * startVec;
					auto ver0 = curLine.P2 + vec*radius;
					buildVertexList.emplace_back(ver0);
				}
			}
			else
			{
				auto pln2 = curLine.Pln2;
				auto startVec = lines[curIndex - 1].pm2;

				for (auto index = 0; index < segment; ++index)
				{
					Ogre::Degree deg(index * step);
					Ogre::Quaternion rot;
					rot.FromAngleAxis(deg, pln2.normal);

					auto vec = rot * startVec;
					auto ver0 = curLine.P2 + vec*radius;
					buildVertexList.emplace_back(ver0);
				}
			}
		}

		auto curBegin = segment * curIndex;
		auto postBegin = curBegin + segment;
		if (closed)
		{
			if (curIndex == lines.size() - 1)
			{
				postBegin = 0;
			}
		}

		for (auto curSeg = 0; curSeg < segment; ++curSeg)
		{
			if (curSeg != segment - 1)
			{
				buildIndexList.push_back(postBegin + curSeg);
				buildIndexList.push_back(curBegin + curSeg);
				buildIndexList.push_back(postBegin + curSeg + 1);
				buildIndexList.push_back(postBegin + curSeg + 1);
				buildIndexList.push_back(curBegin + curSeg);
				buildIndexList.push_back(curBegin + curSeg + 1);
			}
			else
			{
				buildIndexList.push_back(postBegin + curSeg);
				buildIndexList.push_back(curBegin + curSeg);
				buildIndexList.push_back(postBegin);
				buildIndexList.push_back(postBegin);
				buildIndexList.push_back(curBegin + curSeg);
				buildIndexList.push_back(curBegin);
			}
		}
	}

	return std::make_tuple(buildVertexList, buildIndexList);
}

Ogre::MeshPtr MeshUtil::BuildMesh(const yzm::PointCloud& pointCloud, const std::string meshName)
{
	auto retMesh = Ogre::MeshManager::getSingleton().createManual(meshName, "General");

	auto curSubMesh = retMesh->createSubMesh();
	curSubMesh->operationType = Ogre::RenderOperation::OT_POINT_LIST;
	curSubMesh->useSharedVertices = false;
	curSubMesh->vertexData = new Ogre::VertexData;

	auto osgMesh = pointCloud.point_type() == yzm::PointCloud_EPntType_OSG;
	Ogre::VertexElementType ogreColorType;
	if (osgMesh)
	{
		ogreColorType = Ogre::VET_COLOUR_ABGR;
	}
	else
	{
		ogreColorType = Ogre::VET_COLOUR_ARGB;
	}

	auto offset = 0;
	auto decl = curSubMesh->vertexData->vertexDeclaration;
	auto& posEle = decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
	offset += static_cast<decltype( offset )>( Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3) );
	auto& clrEle = decl->addElement(0, offset, ogreColorType, Ogre::VES_DIFFUSE);
	offset += static_cast<decltype(offset)>(Ogre::VertexElement::getTypeSize(ogreColorType));

	auto vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(decl->getVertexSize(0), pointCloud.point_list_size(), Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY, true);
	{
		auto pBuf = static_cast<uint8_t*>( vBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD) );

		for ( auto& curVertex : pointCloud.point_list() )
		{
			float* pVal;
			Ogre::RGBA* pClrVal;

			posEle.baseVertexPointerToElement(pBuf, &pVal);

			if ( !osgMesh )
			{
				pVal[0] = curVertex.x();
				pVal[1] = curVertex.y();
				pVal[2] = curVertex.z();
			}
			else
			{
				pVal[0] = curVertex.x();
				pVal[1] = curVertex.z();
				pVal[2] = -curVertex.y();
			}

			Ogre::ColourValue clr(curVertex.r(), curVertex.g(), curVertex.b());

			clrEle.baseVertexPointerToElement(pBuf, &pClrVal);
			if (!osgMesh)
			{
				*pClrVal = clr.getAsARGB();
			}
			else
			{
				*pClrVal = clr.getAsABGR();
			}

			pBuf += decl->getVertexSize(0);
		}

		vBuf->unlock();

		curSubMesh->vertexData->vertexBufferBinding->setBinding(0, vBuf);
		curSubMesh->vertexData->vertexCount = vBuf->getNumVertices();
	}

	retMesh->_updateBoundsFromVertexBuffers();

	return retMesh;
}

bool MeshUtil::ImportMesh(yzm::PointCloud& pointCloud, std::string& buf)
{
	pointCloud.Clear();

	auto dataStream = new Ogre::MemoryDataStream(reinterpret_cast<void*>( &buf[0] ), buf.size(), false, true);
	Ogre::DataStreamPtr dataPtr(dataStream);

	auto mesh = Ogre::MeshManager::getSingleton().createManual("import", "General");

	try
	{
		Ogre::MeshSerializer ms;
		ms.importMesh(dataPtr, mesh.get());
	}
	catch ( ... )
	{
		return false;
	}

	pointCloud.set_point_type(yzm::PointCloud_EPntType_Ogre);

	std::vector<Ogre::VertexData*> vdList;
	{
		if ( mesh->sharedVertexData )
		{
			vdList.push_back(mesh->sharedVertexData);
		}

		for ( auto curIndex = 0; curIndex < mesh->getNumSubMeshes(); ++curIndex )
		{
			auto curSubmesh = mesh->getSubMesh(curIndex);

			if ( !curSubmesh->useSharedVertices )
			{
				vdList.push_back(curSubmesh->vertexData);
			}
		}
	}

	for ( auto curVD : vdList )
	{
		auto decl = curVD->vertexDeclaration;
		auto posEle = decl->findElementBySemantic(Ogre::VES_POSITION);
		auto clrEle = decl->findElementBySemantic(Ogre::VES_DIFFUSE);
		if ( !clrEle )
		{
			continue;
		}

		auto clrType = clrEle->getBestColourVertexElementType();
		auto vBuf = curVD->vertexBufferBinding->getBuffer(0);
		auto pBuf = static_cast<uint8_t*>( vBuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY) );

		float* pVal;
		Ogre::RGBA* pClrVal;

		for ( auto curIndex = 0; curIndex < curVD->vertexCount; ++curIndex )
		{
			auto newVer = pointCloud.add_point_list();

			posEle->baseVertexPointerToElement(pBuf, &pVal);
			auto& pos = *reinterpret_cast<Ogre::Vector3*>( pVal );
			newVer->set_x(pos.x);
			newVer->set_y(pos.y);
			newVer->set_z(pos.z);

			clrEle->baseVertexPointerToElement(pBuf, &pClrVal);
			Ogre::ColourValue clr;
			if ( clrType == Ogre::VET_COLOUR_ABGR )
			{
				clr.setAsARGB(*pClrVal);
			}
			else if ( clrType == Ogre::VET_COLOUR_ARGB )
			{
				clr.setAsARGB(*pClrVal);
			}
			newVer->set_r(clr.r);
			newVer->set_g(clr.g);
			newVer->set_b(clr.b);

			pBuf += vBuf->getVertexSize();
		}
	}
	
	Ogre::MeshManager::getSingleton().remove(mesh.staticCast<Ogre::Resource>());

	return true;
}
