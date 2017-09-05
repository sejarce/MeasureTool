#pragma once

#include "Render/IFrameListener.h"

#include "OgrePrerequisites.h"

class RootController : public IFrameListener
{
	class 	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	RootController(Ogre::RenderWindow* rt);

	~RootController();

private:

	virtual void	_FrameQueue(const Ogre::FrameEvent& fevt) override;

	virtual void	_HandleFrameEventImmediately(const FrameEvent& frameEvent) override;

};

