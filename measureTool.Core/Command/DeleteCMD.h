#pragma once

#include "ICommand.h"

class	DeleteCMD : public ICommand
{
public:

	DeleteCMD(const IDOMSPtr& dom);

public:

	virtual void Execute() override;

	virtual void Undo() override;

};