#pragma once

#include "RadiusRaySceneQueryFwd.h"

#include "OgreSceneQuery.h"

class RadiusRaySceneQuery : public Ogre::RaySceneQuery
{
public:

	RadiusRaySceneQuery(Ogre::SceneManager* creator);

	~RadiusRaySceneQuery();

public:

	virtual void execute(Ogre::RaySceneQueryListener* listener) override;

public:

	void	SetRadius(float val);

	float	GetRadius() const;

	void	SetRqRange(int first, int last);

private:

	bool _Execute(Ogre::ObjectData objData, size_t numNodes, Ogre::RaySceneQueryListener* listener);

private:

	float	Radius_ = 1.f;
};