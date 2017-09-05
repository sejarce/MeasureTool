#pragma once

#include "Render/IFrameListener.h"

#include "OgrePrerequisites.h"

class TestController : public IFrameListener
{
	class 	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	TestController(Ogre::RenderWindow* rt);

	~TestController();

private:

	virtual void	_FrameQueue(const Ogre::FrameEvent& fevt) override;
};

