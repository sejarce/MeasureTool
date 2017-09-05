#pragma once

#include "IFrameListenerFwd.h"
#include "OgreWndWrapperFwd.h"
#include "SysEventFwd.h"
#include "FrameEvent/FrameEventFwd.h"

#include <memory>
#include <functional>

namespace Ogre
{
class	SceneManager;
}

class OgreEnv
{
	class Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

	OgreEnv();

public:

	using	ThreadTask = std::function<void()>;

public:

	~OgreEnv();

public:

	static OgreEnv&	GetInstance();

public:

	void	Init();

	void	UnInit();

	void	RenderOneFrame();

	Ogre::SceneManager*	GetGlobalSmgr() const;

public:

	OgreWndWrapperUPtr	CreateRenderWindow(uint32_t handle, uint32_t width, uint32_t height);

	void	PostSysEventTo3D(const OgreWndWrapper& wnd, const SSysEvent& evt);

	void	PostFrameEventTo3D(const OgreWndWrapper& wnd, const FrameEvent& evt);
	
	void	PostFrameEventTo3D(const FrameEvent& evt); 

	void	PostFrameEventToUI(const FrameEvent& evt);

	void	SetUIFrameEventResponser(const FrameEventResponser& responser);

public:

	void	AddFrameListener(const IFrameListenerSPtr& frameListener);

	void	RemoveFrameListener(const IFrameListenerSPtr& frameListener);

	void	PostThreadTask(const ThreadTask& task);
};