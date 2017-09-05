#pragma once

#include "Render/IFrameListener.h"
#include "Render/Camera/MayaCamera.h"

#include "FrameEvent/FrameEvent.h"

#include "OgrePrerequisites.h"

#include "Util/PCLOctree.h"

class EditLineLengthController : public IFrameListener
{
	class 	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	EditLineLengthController(Ogre::RenderWindow* rt
		, const MayaCameraSPtr& camera
		, Ogre::SceneNode* node
		, PCLOctreeSPtr octree);

	~EditLineLengthController();

private:

	virtual void	_FrameQueue(const Ogre::FrameEvent& fevt) override;
};
