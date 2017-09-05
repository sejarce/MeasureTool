#pragma once

#include "Render/IFrameListener.h"

#include "OgrePrerequisites.h"

class DummyController : public IFrameListener
{
	class 	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	DummyController(Ogre::RenderWindow* rt);

	~DummyController();

private:

	virtual void	_FrameQueue(const Ogre::FrameEvent& fevt) override;
};

