#include "IFrameListener.h"

#include "OgreEnv.h"
#include "SysEventRecorder.h"
#include "FrameEvent/FrameEvent.h"

#include <vector>
#include <algorithm>
#include <unordered_map>

class	IFrameListener::Imp
{
public:

	SysEventRecorder							EventRecorder_;
	std::vector<IFrameListenerSPtr >			ChildList_;
	std::vector<IFrameListenerSPtr >			NewChildList_;
	std::vector<IFrameListenerSPtr >			LazyRemoveChildList_;
	IFrameListenerWPtr							Parent_;
	std::unordered_map<std::string, FrameEvent>	FrameEventQueue_;
	std::vector<FrameEvent>						LazyFrameEventQueue_;
};

IFrameListener::IFrameListener():IFrameListenerUPtr_(new Imp)
{

}

IFrameListener::~IFrameListener()
{
	
}

void IFrameListener::AddChild( const IFrameListenerSPtr &child )
{
	auto& imp_ = *IFrameListenerUPtr_;
	child->_SetParent(shared_from_this());
	imp_.NewChildList_.push_back(child);
}

void IFrameListener::RemoveChild( const IFrameListenerSPtr &child )
{
	auto& imp_ = *IFrameListenerUPtr_;

	auto itor = std::remove(imp_.ChildList_.begin(), imp_.ChildList_.end(), child);
	imp_.ChildList_.erase(itor, imp_.ChildList_.end());
}

void IFrameListener::FrameStart( const Ogre::FrameEvent &fevt )
{
	auto& imp_ = *IFrameListenerUPtr_;

	if ( !imp_.NewChildList_.empty() )
	{
		std::copy(imp_.NewChildList_.begin(), imp_.NewChildList_.end(), std::back_inserter(imp_.ChildList_));
		imp_.NewChildList_.clear();
	}

	if ( !imp_.LazyFrameEventQueue_.empty() )
	{
		for ( auto& curFE : imp_.LazyFrameEventQueue_ )
		{
			HandleFrameEvent(curFE);
		}
		imp_.LazyFrameEventQueue_.clear();
	}

	_FrameStart(fevt);

	for (auto& curChild : imp_.ChildList_)
	{
		curChild->FrameStart(fevt);
	}

	_FrameStartPost(fevt);
}

void IFrameListener::FrameQueue( const Ogre::FrameEvent &fevt )
{
	auto& imp_ = *IFrameListenerUPtr_;

	_FrameQueue(fevt);

	for (auto& curChild : imp_.ChildList_)
	{
		curChild->FrameQueue(fevt);
	}

	_FrameQueuePost(fevt);
}

void IFrameListener::FrameEnd( const Ogre::FrameEvent &fevt )
{
	auto& imp_ = *IFrameListenerUPtr_;

	_FrameEnd(fevt);

	for (auto& curChild : imp_.ChildList_)
	{
		curChild->FrameEnd(fevt);
	}

	_FrameEndPost(fevt);

	for ( auto& curChild : imp_.LazyRemoveChildList_ )
	{
		auto itor = std::remove(imp_.ChildList_.begin(), imp_.ChildList_.end(), curChild);
		imp_.ChildList_.erase(itor, imp_.ChildList_.end());
	}

	imp_.LazyRemoveChildList_.clear();

	imp_.EventRecorder_.Reset();
	imp_.FrameEventQueue_.clear();
}

void IFrameListener::HandleSysEvent(const SSysEvent &evt)
{
	auto& imp_ = *IFrameListenerUPtr_;

	imp_.EventRecorder_.TransferEvent(evt);

	for ( auto& curChild : imp_.ChildList_ )
	{
		curChild->HandleSysEvent(evt);
	}
}

const SysEventRecorder& IFrameListener::GetSysEventRecorder() const
{
	auto& imp_ = *IFrameListenerUPtr_;

	return imp_.EventRecorder_;
}

void IFrameListener::HandleFrameEvent(const FrameEvent& frameEvent)
{
	auto& imp_ = *IFrameListenerUPtr_;

	imp_.FrameEventQueue_.emplace(frameEvent.GetEventName(), frameEvent);

	for (auto& curChild : imp_.ChildList_)
	{
		curChild->HandleFrameEvent(frameEvent);
	}
}

void IFrameListener::HandleFrameEventImmediately(const FrameEvent& frameEvent)
{
	_HandleFrameEventImmediately(frameEvent);
}

void IFrameListener::HandleFrameEventLazy(const FrameEvent& frameEvent)
{
	auto& imp_ = *IFrameListenerUPtr_;

	imp_.LazyFrameEventQueue_.push_back(frameEvent);
}

void IFrameListener::PostFrameEventToUI(const FrameEvent& frameEvent)
{
	OgreEnv::GetInstance().PostFrameEventToUI(frameEvent);
}

void IFrameListener::PostThreadTask(const ThreadTask& task)
{
	auto& imp_ = *IFrameListenerUPtr_;

	OgreEnv::GetInstance().PostThreadTask(task);
}

IFrameListenerSPtr IFrameListener::GetParent()
{
	auto& imp_ = *IFrameListenerUPtr_;

	return imp_.Parent_.lock();
}

void IFrameListener::RemoveLazy()
{
	auto& imp_ = *IFrameListenerUPtr_;

	auto parent = imp_.Parent_.lock();
	if ( parent )
	{
		parent->_RemoveChildLazy(shared_from_this());
	}
}

void IFrameListener::RemoveChildrenLazy()
{
	auto& imp_ = *IFrameListenerUPtr_;

	std::copy(imp_.ChildList_.begin(), imp_.ChildList_.end(), std::back_inserter(imp_.LazyRemoveChildList_));
}

void IFrameListener::Unload()
{
	auto& imp_ = *IFrameListenerUPtr_;

	imp_.ChildList_.clear();
	imp_.LazyRemoveChildList_.clear();
	imp_.NewChildList_.clear();
}

boost::optional<FrameEvent> IFrameListener::_PopFrameEvent(const std::string& frameName)
{
	auto& imp_ = *IFrameListenerUPtr_;

	FrameEvent ret;

	auto itor = imp_.FrameEventQueue_.find(frameName);
	if ( itor != imp_.FrameEventQueue_.end() )
	{
		ret = itor->second;
		imp_.FrameEventQueue_.erase(itor);

		return ret;
	}

	return boost::none;
}

void IFrameListener::_SetParent(const IFrameListenerSPtr& parent)
{
	auto& imp_ = *IFrameListenerUPtr_;

	imp_.Parent_ = parent;
}

void IFrameListener::_RemoveChildLazy(const IFrameListenerSPtr& child)
{
	auto& imp_ = *IFrameListenerUPtr_;

	imp_.LazyRemoveChildList_.push_back(child);
}
