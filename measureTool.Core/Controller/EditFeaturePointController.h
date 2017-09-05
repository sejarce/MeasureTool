#pragma once

#include "Render/IFrameListener.h"
#include "Render/Camera/MayaCamera.h"

#include "DOM/DocumentFwd.h"

#include "OgrePrerequisites.h"

class EditFeaturePointController : public IFrameListener
{
	class 	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	EditFeaturePointController(Ogre::RenderWindow* rt
		, const MayaCameraSPtr& camera
		, Ogre::SceneNode* node
		, DocumentSPtr& doc);

	~EditFeaturePointController();

private:

	virtual void	_FrameQueue(const Ogre::FrameEvent& fevt) override;
};

