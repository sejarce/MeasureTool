#pragma once

#include <stdint.h>
#include <memory>
#include <functional>

class AsyncThreadPool
{
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	using	Task = std::function<void()>;

public:

	AsyncThreadPool();

	~AsyncThreadPool();

public:

	bool	StartAsync(uint32_t threadsNumber);

	void	JoinAllThreads();

	void	Stop();

	bool	IsStop() const;

	void	Post(const Task& task);

	uint32_t	GetThreadsNumber() const;
};