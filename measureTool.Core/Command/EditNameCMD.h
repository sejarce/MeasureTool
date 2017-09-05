#pragma once

#include "ICommand.h"

class	EditNameCMD : public ICommand
{
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	EditNameCMD(const IDOMSPtr& dom, const std::wstring& before, const std::wstring& post);

	~EditNameCMD();

public:

	virtual void Execute() override;

	virtual void Undo() override;

};