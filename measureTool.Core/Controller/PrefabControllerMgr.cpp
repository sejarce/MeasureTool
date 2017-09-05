#include "PrefabControllerMgr.h"

#include "RootController.h"
#include "DummyController.h"
#include "TestController.h"

PrefabControllerMgr::FrameListenerCreatorMap PrefabControllerMgr::GetPrefabFrameListeners()
{
	FrameListenerCreatorMap ret;

	ret.emplace("default", [](Ogre::RenderWindow* wnd)
	{
		return std::make_shared<RootController>(wnd);
	});

	ret.emplace("dummy", [](Ogre::RenderWindow* wnd)
	{
		return std::make_shared<DummyController>(wnd);
	});

	ret.emplace("test", [](Ogre::RenderWindow* wnd)
	{
		return std::make_shared<TestController>(wnd);
	});

	return ret;
}
