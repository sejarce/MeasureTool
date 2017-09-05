#include "ICommand.h"

ICommand::ICommand(const IDOMSPtr& dom)
{
	DOM_ = dom;
}

IDOMSPtr ICommand::GetDOM() const
{
	return DOM_.lock();
}
