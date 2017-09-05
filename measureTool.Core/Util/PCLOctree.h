#pragma once

#include "PCLOctreeFwd.h"

#include "OgrePrerequisites.h"

#include <memory>

#include <boost/optional.hpp>

class	PCLOctree
{
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	//using	RayQueryOpt = boost::optional<Ogre::Vector3>;
	using	RayQueryOpt = std::tuple<bool , int, Ogre::Vector3>;
	using	PlaneQueryVector = std::vector<Ogre::Vector3>;

public:

	PCLOctree();

	~PCLOctree();

public:

	void	Build(const Ogre::MeshPtr& meshPtr, float leafSize, float resolution);

	RayQueryOpt	RayQuery(const Ogre::Ray& ray, float resolution) const;

	PlaneQueryVector	PlaneQuery(const Ogre::Plane& pln, const Ogre::Vector3& queryPnt, float resolution) const;

	void	SetFeaturePoint(const Ogre::Vector3& leftArmpit, const Ogre::Vector3& rightArmpit, const Ogre::Vector3& crotch);
};