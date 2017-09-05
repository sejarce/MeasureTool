#pragma once

#include "ErrorCode.h"

#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0501
#endif // !_WIN32_WINNT

#include <boost/asio.hpp>

#include <memory>


class Proactor
{
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	Proactor();

	~Proactor();

public:

	ErrorCode	Run();

	void		Stop();

	bool		IsStop() const;

	boost::asio::io_service&	GetProactor();

	boost::asio::io_service::strand&	GetSafeProactor();
};