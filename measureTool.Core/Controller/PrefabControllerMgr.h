#pragma once

#include "Render/IFrameListener.h"

namespace Ogre
{
class	RenderWindow;
}

class	PrefabControllerMgr
{
public:

	using	FrameListenerCreator = std::function<IFrameListenerSPtr(Ogre::RenderWindow*)>;
	using	FrameListenerCreatorMap = std::map<std::string, FrameListenerCreator>;

public:

	static	FrameListenerCreatorMap	GetPrefabFrameListeners();
};