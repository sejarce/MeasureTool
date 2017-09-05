#include "OgreWndWrapper.h"
#include "OgreEnv.h"

#include "Ogre.h"

OgreWndWrapper::OgreWndWrapper(Ogre::RenderWindow* wnd)
{
	Wnd_ = wnd;
}

OgreWndWrapper::~OgreWndWrapper()
{

}

void OgreWndWrapper::Resize(uint32_t width, uint32_t height)
{
	Wnd_->windowMovedOrResized();
}

uint32_t OgreWndWrapper::GetWidth() const
{
	return Wnd_->getWidth();
}

uint32_t OgreWndWrapper::GetHeight() const
{
	return Wnd_->getHeight();
}

Ogre::RenderWindow* OgreWndWrapper::GetRenderWindow() const
{
	return Wnd_;
}

void OgreWndWrapper::PostSysEvent(const SSysEvent& evt)
{
	OgreEnv::GetInstance().PostSysEventTo3D(*this, evt);
}

void OgreWndWrapper::PostFrameEvent(const FrameEvent& evt)
{
	OgreEnv::GetInstance().PostFrameEventTo3D(*this, evt);
}

void OgreWndWrapper::Destory()
{
	Ogre::Root::getSingletonPtr()->destroyRenderTarget(Wnd_);
}
