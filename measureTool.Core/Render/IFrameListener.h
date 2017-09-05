#pragma once

#include "IFrameListenerFwd.h"
#include "SysEventRecorder.h"
#include "FrameEvent/FrameEventFwd.h"

#include <memory>
#include <functional>
#include <boost/optional.hpp>

namespace Ogre
{
	struct FrameEvent;
}

class IFrameListener : public std::enable_shared_from_this<IFrameListener>
{
	class	Imp;
	std::unique_ptr<Imp>	IFrameListenerUPtr_;

public:

	using	ThreadTask = std::function<void()>;

public:

	IFrameListener();

	virtual ~IFrameListener();

public:

	void	AddChild(const IFrameListenerSPtr& child);

	void	RemoveChild(const IFrameListenerSPtr& child);

public:

	void	FrameStart(const Ogre::FrameEvent& fevt);

	void	FrameQueue(const Ogre::FrameEvent& fevt);

	void	FrameEnd(const Ogre::FrameEvent& fevt);

public:

	void	HandleSysEvent(const SSysEvent& evt);

	const SysEventRecorder&	GetSysEventRecorder() const;

	void	HandleFrameEvent(const FrameEvent& frameEvent);

	void	HandleFrameEventImmediately(const FrameEvent& frameEvent);

	void	HandleFrameEventLazy(const FrameEvent& frameEvent);

	void	PostFrameEventToUI(const FrameEvent& frameEvent);

	template<typename T>
	std::shared_ptr<T>	PopFrameEvent()
	{
		auto frameEvtOpt = _PopFrameEvent(typeid( T ).name());
		if ( frameEvtOpt )
		{
			return ( *frameEvtOpt ).GetEvent<T>();
		}

		return{};
	}

	void	PostThreadTask(const ThreadTask& task);

protected:

	virtual void	_FrameStart(const Ogre::FrameEvent& fevt) {}

	virtual void	_FrameStartPost(const Ogre::FrameEvent& fevt) {}

	virtual void	_FrameQueue(const Ogre::FrameEvent& fevt) {}

	virtual void	_FrameQueuePost(const Ogre::FrameEvent& fevt) {}

	virtual void	_FrameEnd(const Ogre::FrameEvent& fevt) {}

	virtual void	_FrameEndPost(const Ogre::FrameEvent& fevt) {}

	virtual	void	_HandleFrameEventImmediately(const FrameEvent& frameEvent) {}

protected:

	IFrameListenerSPtr	GetParent();

	void			RemoveLazy();

	void			RemoveChildrenLazy();

	void			Unload();

private:

	boost::optional<FrameEvent> _PopFrameEvent(const std::string& frameName);

	void			_RemoveChildLazy(const IFrameListenerSPtr& child);

	void			_SetParent(const IFrameListenerSPtr& parent);
};