#pragma once

#include <string>
#include <memory>

class	CpuTimer
{
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	CpuTimer(const std::string& prefix = "", bool force = true);

	~CpuTimer();

public:

	void	Print();
};