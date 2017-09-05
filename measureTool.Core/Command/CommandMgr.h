#pragma once

#include "ICommandFwd.h"
#include "DOM/IDOMFwd.h"

#include <memory>

class	CommandMgr
{
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	static CommandMgr&	GetInstance();

private:

	CommandMgr();

public:

	~CommandMgr();

public:

	void	PushCommand(const ICommandSPtr& cmd);

	void	RemoveRelativeCommand(const IDOMSPtr& dom);

	void	Redo();

	void	Undo();

	void	Clear();

	void	SetMaxSize(uint32_t size);

	uint32_t	GetMaxSize() const;
};