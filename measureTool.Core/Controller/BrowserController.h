#pragma once

#include "Render/IFrameListener.h"
#include "Render/Camera/MayaCamera.h"

#include "FrameEvent/FrameEvent.h"

#include "OgrePrerequisites.h"

class BrowserController : public IFrameListener
{
	class 	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	BrowserController(Ogre::RenderWindow* rt, const MayaCameraSPtr& camera);

	~BrowserController();

private:

	virtual void _FrameQueue(const Ogre::FrameEvent& fevt) override;

	virtual void _FrameStart(const Ogre::FrameEvent& fevt) override;

	virtual void _HandleFrameEventImmediately(const FrameEvent& frameEvent) override;
};

