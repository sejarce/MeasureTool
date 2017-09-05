#include "DebugUtil.h"

#include "Ogre.h"

#include <boost/filesystem/fstream.hpp>

void DebugUtil::SaveToPly(const std::vector<Ogre::Vector3>& vList, const std::string& fileName)
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