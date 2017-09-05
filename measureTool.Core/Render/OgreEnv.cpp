#include "OgreEnv.h"
#include "SysEventRecorder.h"
#include "SysEvent.h"
#include "OgreWndWrapper.h"
#include "IFrameListener.h"
#include "ICameraFrameListener.h"
#include "Extension/ExtensionUtil.h"

#include "FrameEvent/FrameEvent.h"

#include "Concurrency/AsyncThreadPool.h"

#include "DOM/DocumentMgr.h"

#include "OgreParticleFXPlugin.h"
#include "OgreGLPlugin.h"
#include "Ogre.h"

#include "google/protobuf/stubs/common.h"

#include <vector>
#include <mutex>

class	OgreEnv::Imp : public Ogre::FrameListener
{
public:

	AsyncThreadPool					ThreadPool_;
	std::vector<IFrameListenerSPtr>	ChildList_;
	std::vector<Ogre::Camera*>		UpdateCameraList_;
	std::vector<std::unique_ptr<Ogre::Plugin>>		Plugins_;
	Ogre::RenderWindow*				ContextWnd_{};
	HGLRC							Context_{};
	FrameEventResponser				UIResponser_;
	Ogre::SceneManager*				Smgr_{};
	std::vector<FrameEvent>			FrameEventList_;
	std::mutex						FrameEventMutex_;

public:

	virtual bool frameStarted(const Ogre::FrameEvent& evt) override
	{
		decltype( FrameEventList_ ) tmpList;
		{
			std::unique_lock<std::mutex> lock(FrameEventMutex_);
			tmpList.swap(FrameEventList_);
		}

		for ( auto& curChild : ChildList_ )
		{
			for ( auto& curFE : tmpList )
			{
				curChild->HandleFrameEvent(curFE);
			}
		}

		auto smgrList = Ogre::Root::getSingletonPtr()->getSceneManagerIterator();
		for (auto& curSmgr : smgrList )
		{
			auto cameraList = curSmgr.second->getCameraIterator();
			for (auto& curCamera : cameraList )
			{
				UpdateCameraList_.push_back(curCamera);
			}
		}

		for (auto& curCamera : UpdateCameraList_ )
		{
			ICameraFrameListener::CameraFrameStart(curCamera, evt);
		}

		for (auto& curChild : ChildList_)
		{
			curChild->FrameStart(evt);
		}

		return true;
	}

	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt) override
	{
		for (auto& curCamera : UpdateCameraList_ )
		{
			ICameraFrameListener::CameraFrameQueue(curCamera, evt);
		}

		for (auto& curChild : ChildList_)
		{
			curChild->FrameQueue(evt);
		}

		return true;
	}

	virtual bool frameEnded(const Ogre::FrameEvent& evt) override
	{
		for (auto& curCamera : UpdateCameraList_ )
		{
			ICameraFrameListener::CameraFrameEnd(curCamera, evt);
		}

		for (auto& curChild : ChildList_)
		{
			curChild->FrameEnd(evt);
		}

		UpdateCameraList_.clear();

		return true;
	}
};

OgreEnv::OgreEnv():ImpUPtr_(new Imp)
{ }

OgreEnv::~OgreEnv()
{ }

OgreEnv& OgreEnv::GetInstance()
{
	static OgreEnv sins;
	return  sins;
}

void OgreEnv::Init()
{
	auto& imp_ = *ImpUPtr_;

	auto root = new Ogre::Root("", "");
	
	imp_.Plugins_.emplace_back(std::make_unique<Ogre::ParticleFXPlugin>());
	imp_.Plugins_.emplace_back(std::make_unique<Ogre::GLPlugin>());

	for (auto& curPlugin : imp_.Plugins_ )
	{
		root->installPlugin(curPlugin.get());
	}

	root->addFrameListener(ImpUPtr_.get());

	root->setRenderSystem(root->getAvailableRenderers().front());
	root->initialise(false);

	Ogre::ConfigFile cfg;
	{
		std::ifstream ifs("Data/resources.cfg", std::ios::binary);
		Ogre::DataStreamPtr srcData(new Ogre::FileStreamDataStream(&ifs, false));
		cfg.load(srcData);
	}

	auto sectionItor = cfg.getSectionIterator();
	for ( auto& curSec : sectionItor )
	{
		auto sec = curSec.first;
		for ( auto& curSet : *( curSec.second ) )
		{
			Ogre::ResourceGroupManager::getSingleton().addResourceLocation(curSet.second, curSet.first, sec);
		}
	}

	Ogre::NameValuePairList params;
	params.emplace("title", "ContextWnd");
	params.emplace("hidden", "true");
	params.emplace("FSAA", std::to_string(4));

	imp_.ContextWnd_ = Ogre::Root::getSingletonPtr()->createRenderWindow("contextWnd", 16, 16, false, &params);
	imp_.Context_ = wglGetCurrentContext();

	Ogre::ResourceGroupManager::getSingletonPtr()->initialiseAllResourceGroups();

	ExtensionUtil::Init();

	imp_.ThreadPool_.StartAsync(4);

	imp_.Smgr_ = Ogre::Root::getSingletonPtr()->createSceneManager(Ogre::ST_GENERIC, 1, Ogre::INSTANCING_CULLING_SINGLETHREAD, "RootScene");
}

void OgreEnv::UnInit()
{
	auto& imp_ = *ImpUPtr_;

	imp_.ThreadPool_.Stop();

	imp_.ChildList_.clear();

	DocumentMgr::GetInstance().UnInit();

	ExtensionUtil::UnInit();

	Ogre::Root::getSingletonPtr()->destroySceneManager(imp_.Smgr_);

	delete Ogre::Root::getSingletonPtr();

	imp_.Plugins_.clear();

	imp_.ThreadPool_.JoinAllThreads();

	google::protobuf::ShutdownProtobufLibrary();
}

void OgreEnv::RenderOneFrame()
{
	static auto s_LastTime = 0U;

	auto& root = *Ogre::Root::getSingletonPtr();

	auto curTime = root.getTimer()->getMicroseconds();
	auto frameTime = curTime - s_LastTime;
	auto limitFPS = 60;
	auto limitFrameTime = 1E6 / static_cast<float >(limitFPS);

	if ( frameTime < limitFrameTime )
	{
		return;
	}

	root.renderOneFrame();
}

Ogre::SceneManager* OgreEnv::GetGlobalSmgr() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.Smgr_;
}

OgreWndWrapperUPtr OgreEnv::CreateRenderWindow(uint32_t handle, uint32_t width, uint32_t height)
{
	auto& imp_ = *ImpUPtr_;

	Ogre::NameValuePairList params;
	params.emplace("externalWindowHandle", std::to_string(handle));
	params.emplace("externalGLContext", std::to_string(reinterpret_cast<uint32_t>( imp_.Context_ )));
	params.emplace("FSAA", std::to_string(4));

	auto renderWnd = Ogre::Root::getSingletonPtr()->createRenderWindow(std::to_string(handle), width, height, false, &params);

	return std::make_unique<OgreWndWrapper>(renderWnd);
}

void OgreEnv::PostSysEventTo3D(const OgreWndWrapper& wnd, const SSysEvent& evt)
{
	auto& imp_ = *ImpUPtr_;

	if ( evt.EventType == SSysEvent::EET_Mouse )
	{
		if ( evt.MouseEvt.Moved )
		{
			SysEventRecorder::CachedMouseState() = evt.MouseEvt.State;
		}
	}

	auto smgrList = Ogre::Root::getSingletonPtr()->getSceneManagerIterator();
	for ( auto& curSmgr : smgrList )
	{
		auto cameraList = curSmgr.second->getCameraIterator();
		for ( auto& curCamera : cameraList )
		{
			ICameraFrameListener::CameraPostSysEvent(curCamera, evt);
		}
	}

	for ( auto& curChild : imp_.ChildList_ )
	{
		curChild->HandleSysEvent(evt);
	}
}

void OgreEnv::PostFrameEventTo3D(const OgreWndWrapper& wnd, const FrameEvent& evt)
{
	auto& imp_ = *ImpUPtr_;

	{
		std::unique_lock<std::mutex> lock(imp_.FrameEventMutex_);
		imp_.FrameEventList_.push_back(evt);
	}
}

void OgreEnv::PostFrameEventTo3D(const FrameEvent& evt)
{
	auto& imp_ = *ImpUPtr_;

	{
		std::unique_lock<std::mutex> lock(imp_.FrameEventMutex_);
		imp_.FrameEventList_.push_back(evt);
	}
}

void OgreEnv::PostFrameEventToUI(const FrameEvent& evt)
{
	auto& imp_ = *ImpUPtr_;

	if ( imp_.UIResponser_ )
	{
		imp_.UIResponser_(evt);
	}
}

void OgreEnv::SetUIFrameEventResponser(const FrameEventResponser& responser)
{
	auto& imp_ = *ImpUPtr_;

	imp_.UIResponser_ = responser;
}

void OgreEnv::AddFrameListener( const IFrameListenerSPtr &frameListener )
{
	auto& imp_ = *ImpUPtr_;

	imp_.ChildList_.push_back(frameListener);
}

void OgreEnv::RemoveFrameListener( const IFrameListenerSPtr &frameListener )
{
	auto& imp_ = *ImpUPtr_;

	auto itor = std::remove(imp_.ChildList_.begin(), imp_.ChildList_.end(), frameListener);
	imp_.ChildList_.erase(itor, imp_.ChildList_.end());
}

void OgreEnv::PostThreadTask(const ThreadTask& task)
{
	auto& imp_ = *ImpUPtr_;

	imp_.ThreadPool_.Post(task);
}
