#pragma once

#include "ICommand.h"

class	EditContentCMD : public ICommand
{
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	EditContentCMD(const IDOMSPtr& dom, const std::string& before, const std::string& post);

	~EditContentCMD();

public:

	virtual void Execute() override;

	virtual void Undo() override;

};