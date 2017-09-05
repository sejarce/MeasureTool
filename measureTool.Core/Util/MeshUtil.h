#pragma once

#include "OgrePrerequisites.h"

#include <string>
#include <vector>

namespace yzm
{
class	PointCloud;
}

class	MeshUtil
{
public:

	static Ogre::MeshPtr	LoadTestMesh(const std::wstring folerPath, const std::string meshName, bool save);
	
	static	void	SaveToPly(const std::wstring& filePath, const std::vector<Ogre::Vector3>& verList, const std::vector<int>& indList);

	static	std::tuple<std::vector<Ogre::Vector3>, std::vector<int>>	LoadPly(const std::wstring& fileName);

	static	std::tuple<std::vector<Ogre::Vector3>, std::vector<int>>	BuildPipe(const std::vector<Ogre::Vector3>& pntList, int segment, float radius, bool closed);

	static Ogre::MeshPtr	BuildMesh(const yzm::PointCloud& pointCloud, const std::string meshName);

	static bool				ImportMesh(yzm::PointCloud& pointCloud, std::string& buf);
};