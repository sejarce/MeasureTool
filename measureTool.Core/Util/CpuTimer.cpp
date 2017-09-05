#include "CpuTimer.h"

#include "OgreLogManager.h"

#include <string>

#include <boost/timer/timer.hpp>
#include <boost/chrono.hpp>

class	CpuTimer::Imp
{
public:

	boost::timer::cpu_timer	Ct_;
	std::string				Prefix_;
	bool					Printed_{ false };
	bool					Force_{ false };

public:

	Imp(const std::string& str) :Prefix_(str) {}
};

CpuTimer::CpuTimer(const std::string& prefix, bool force) :ImpUPtr_(std::make_unique<Imp>(prefix))
{
	ImpUPtr_->Force_ = force;
}

CpuTimer::~CpuTimer()
{
	auto& imp_ = *ImpUPtr_;

	if ( imp_.Force_ )
	{
		Print();
	}
}

void CpuTimer::Print()
{
	auto& imp_ = *ImpUPtr_;

	if ( !imp_.Printed_ )
	{
		auto dur = imp_.Ct_.elapsed();
		Ogre::LogManager::getSingleton().logMessage(imp_.Prefix_ + boost::timer::format(dur));

		imp_.Printed_ = true;
	}
}
