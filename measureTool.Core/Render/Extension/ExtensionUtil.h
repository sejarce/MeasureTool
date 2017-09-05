#pragma once

#include "OgrePrerequisites.h"

class	ExtensionUtil
{
	class	Imp;

public:

	static	void	Init();

	static	void	UnInit();

	static	void CalcBoundsFromVertexBuffer(Ogre::VertexData* vertexData, Ogre::AxisAlignedBox& outAABB, Ogre::Real& outRadius, bool extendOnly /*= false*/);
};