#pragma once

#include "PCLKdtreeFwd.h"

#include "OgrePrerequisites.h"

#include <memory>

#include <boost/optional.hpp>

class PCLKdtree
{
	class Imp;
	std::unique_ptr<Imp> ImpUPtr_;
public:
	using PointPath = std::vector<std::pair<int, Ogre::Vector3>>;

public:
	PCLKdtree();
	~PCLKdtree();

public:
	void	Build(const Ogre::MeshPtr& meshPtr);

	/*
	*@breif �������������������������
	*/
	PointPath	computerShortestPath(unsigned int si, unsigned int ei);

	int findPointIndex(const Ogre::Vector3& pt);
};