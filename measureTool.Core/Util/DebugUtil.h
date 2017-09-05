#pragma once

#include "OgrePrerequisites.h"

class	DebugUtil
{
public:

	static	void	SaveToPly(const std::vector<Ogre::Vector3>& vList, const std::string& fileName);
};