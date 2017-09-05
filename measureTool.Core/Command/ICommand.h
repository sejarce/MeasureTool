#pragma once

#include "ICommandFwd.h"

#include "DOM/IDOMFwd.h"


class	ICommand
{
public:

	ICommand(const IDOMSPtr& dom);

	virtual ~ICommand() {}

public:

	virtual	void	Execute() {}

	virtual void	Undo() {}

	IDOMSPtr	GetDOM() const;

private:

	std::weak_ptr<IDOM>	DOM_;
};