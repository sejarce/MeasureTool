#include "AsyncThreadPool.h"
#include "Proactor.h"

#include <thread>
#include <atomic>

class AsyncThreadPool::Imp
{
public:

	Imp()
	{
		IsStart_ = false;
	}

	std::vector<std::thread>	ThreadGroup_;
	Proactor			Proactor_;
	std::atomic_bool	IsStart_;
};

AsyncThreadPool::AsyncThreadPool() :ImpUPtr_(std::make_unique<Imp>())
{

}

AsyncThreadPool::~AsyncThreadPool()
{
	auto& imp_ = *ImpUPtr_;

	if ( !imp_.Proactor_.IsStop() )
	{
		imp_.Proactor_.Stop();

		JoinAllThreads();
	}
}

bool AsyncThreadPool::StartAsync(uint32_t threadsNumber)
{
	auto& imp_ = *ImpUPtr_;

	if ( threadsNumber < 1 )
	{
		return false;
	}

	auto ret = imp_.IsStart_.exchange(true);
	if ( ret )
	{
		return ret;
	}

	for ( unsigned index = 0; index < threadsNumber; ++index )
	{
		imp_.ThreadGroup_.emplace_back([&imp_]()
		{
			imp_.Proactor_.Run();
		});
	}

	return true;
}

void AsyncThreadPool::JoinAllThreads()
{
	auto& imp_ = *ImpUPtr_;

	for ( auto& curThread : imp_.ThreadGroup_ )
	{
		curThread.join();
	}
}

void AsyncThreadPool::Stop()
{
	auto& imp_ = *ImpUPtr_;

	imp_.Proactor_.Stop();
}

bool AsyncThreadPool::IsStop() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.Proactor_.IsStop();
}

void AsyncThreadPool::Post(const Task& task)
{
	auto& imp_ = *ImpUPtr_;

	imp_.Proactor_.GetProactor().post(task);
}

uint32_t AsyncThreadPool::GetThreadsNumber() const
{
	auto& imp_ = *ImpUPtr_;

	return static_cast<uint32_t>( imp_.ThreadGroup_.size() );
}
