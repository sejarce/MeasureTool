#pragma once

#include "Render/IFrameListener.h"
#include "Render/Camera/MayaCamera.h"

#include "FrameEvent/FrameEvent.h"

#include "OgrePrerequisites.h"

#include "Util/PCLOctree.h"


class EditHeightController : public IFrameListener
{
	class 	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	EditHeightController(Ogre::RenderWindow* rt 
		, const MayaCameraSPtr& camera
		, Ogre::SceneNode* node
		, PCLOctreeSPtr Octree_);

	~EditHeightController();

private:

	virtual void	_FrameQueue(const Ogre::FrameEvent& fevt) override;
};

