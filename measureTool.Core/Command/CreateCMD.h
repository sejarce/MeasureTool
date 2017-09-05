#pragma once

#include "ICommand.h"

class	CreateCMD : public ICommand
{
public:

	CreateCMD(const IDOMSPtr& dom);

public:

	virtual void Execute() override;

	virtual void Undo() override;

};