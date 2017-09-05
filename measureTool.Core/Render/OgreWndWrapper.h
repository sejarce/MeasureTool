#pragma once

#include "SysEventFwd.h"
#include "FrameEvent/FrameEventFwd.h"

#include <stdint.h>

namespace Ogre
{
class	RenderWindow;
}

class	OgreWndWrapper
{
public:

	OgreWndWrapper(Ogre::RenderWindow* wnd);

	~OgreWndWrapper();

public:

	void	Resize(uint32_t width, uint32_t height);

	uint32_t	GetWidth() const;

	uint32_t	GetHeight() const;

	Ogre::RenderWindow*	GetRenderWindow() const;

	void	PostSysEvent(const SSysEvent& evt);

	void	PostFrameEvent(const FrameEvent& evt);

	void	Destory();

private:

	Ogre::RenderWindow*	Wnd_{};
};