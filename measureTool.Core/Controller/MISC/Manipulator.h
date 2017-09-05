#pragma once

#include "Ogre.h"
#include "ROM/IROM.h"

#include <boost/optional.hpp>

class	Manipulator
{
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	Manipulator(Ogre::SceneNode* parentNode);

	~Manipulator();

public:

	enum EObjType
	{
		EOT_Arrow,
		EOT_ZRing,
		EOT_XRing
	};

public:

	void	SetPosition(const Ogre::Vector3& pos);

	Ogre::Vector3	GetPosition() const;

	Ogre::Quaternion	GetOrientation() const;

	void	SetVisible(bool val);

	void	SetRotation(float xDegree, float zDegree);

	std::tuple<float, float>	GetRotation() const;

	float	GetXDegree() const;

	float	GetZDegree() const;

	void	RotateX(float rad);

	void	RotateZ(float rad);

	void	SetState(EObjType obj, IROM::EPickingState state);

	boost::optional<EObjType>	QueryRay(const Ogre::Ray& ray);
};