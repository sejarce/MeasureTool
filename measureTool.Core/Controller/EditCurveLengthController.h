#pragma once
#include "Render/IFrameListener.h"
#include "Render/Camera/MayaCamera.h"

#include "FrameEvent/FrameEvent.h"

#include "Util/PCLOctree.h"
#include "Util/PCLKdtree.h"
#include "ROM/IROM.h"

#include "OgrePrerequisites.h"

class EditCurveLengthController : public IFrameListener
{
	class 	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	EditCurveLengthController(Ogre::RenderWindow* rt
		, const MayaCameraSPtr& camera
		, Ogre::SceneNode* node
		, PCLOctreeSPtr octree
		, PCLKdtreeSPtr kdtree);

	~EditCurveLengthController();

private:

	virtual void	_FrameQueue(const Ogre::FrameEvent& fevt) override;
};