#pragma once

#include "FrameEventFwd.h"

#include <memory>
#include <boost/any.hpp>

class	FrameEvent
{
public:

	template<typename T>
	std::shared_ptr<T>	CreateEvent()
	{
		auto ret = std::make_shared<T>();
		Evt_ = ret;

		EvtName_ = typeid( T ).name();

		return ret;
	}

	const std::string&	GetEventName() const
	{
		return EvtName_;
	}

	template<typename T>
	std::shared_ptr<T>	GetEvent() const
	{
		auto ret = boost::any_cast<std::shared_ptr<T>>( Evt_ );

		return ret;
	}

private:

	boost::any	Evt_;
	std::string	EvtName_;
};

template<typename T>
class	TFrameEvent
{
public:

	using	SPtr = std::shared_ptr<T>;

public:

	virtual ~TFrameEvent() {}

public:

	static std::string& StaticFrameEventName()
	{
		static std::string frameEventName;
		if ( frameEventName.empty() )
		{
			frameEventName = typeid( T ).name();
		}

		return frameEventName;
	}

public:

	FrameEvent	ConvertToFrameEvent()
	{
		FrameEvent ret;

		auto fe = ret.CreateEvent<T>();
		*fe = *static_cast<T*>(this);

		return ret;
	}
};