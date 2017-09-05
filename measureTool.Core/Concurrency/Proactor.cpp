#include "Proactor.h"

#include <atomic>


class Proactor::Imp
{
public:

	Imp() :Work_(IOS_), Strand_(IOS_)
	{
		StopFlag_ = false;
	}

	boost::asio::io_service			IOS_;
	boost::asio::io_service::work	Work_;
	boost::asio::io_service::strand	Strand_;
	boost::system::error_code		ErrCode_;
	std::atomic_bool				StopFlag_;
};

Proactor::Proactor() :ImpUPtr_(std::make_unique<Imp>())
{

}

Proactor::~Proactor()
{

}

ErrorCode Proactor::Run()
{
	auto& imp_ = *ImpUPtr_;

	if ( imp_.StopFlag_)
	{
		return boost::system::error_code();
	}

	imp_.IOS_.run(imp_.ErrCode_);

	if ( !!imp_.ErrCode_ )
	{
		return imp_.ErrCode_;
	}
	else
	{
		return boost::none;
	}
}

void Proactor::Stop()
{
	auto& imp_ = *ImpUPtr_;

	auto ret = imp_.StopFlag_.exchange(true);

	if ( ret )
	{
		return;
	}

	imp_.IOS_.stop();
}

bool Proactor::IsStop() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.IOS_.stopped();
}

boost::asio::io_service& Proactor::GetProactor()
{
	auto& imp_ = *ImpUPtr_;

	return imp_.IOS_;
}

boost::asio::io_service::strand& Proactor::GetSafeProactor()
{
	auto& imp_ = *ImpUPtr_;

	return imp_.Strand_;
}
