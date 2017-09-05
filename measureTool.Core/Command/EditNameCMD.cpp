#include "EditNameCMD.h"

#include "DOM/IDOM.h"

class	EditNameCMD::Imp
{
public:

	std::wstring	Before_;
	std::wstring	Post_;
};

EditNameCMD::EditNameCMD(const IDOMSPtr& dom, const std::wstring& before, const std::wstring& post) :ICommand(dom)
, ImpUPtr_(std::make_unique<Imp>())
{
	auto& imp_ = *ImpUPtr_;

	imp_.Before_ = before;
	imp_.Post_ = post;
}

EditNameCMD::~EditNameCMD()
{

}

void EditNameCMD::Execute()
{
	auto& imp_ = *ImpUPtr_;

	auto dom = GetDOM();
	if ( dom )
	{
		dom->SetName(imp_.Post_, false);
	}
}

void EditNameCMD::Undo()
{
	auto& imp_ = *ImpUPtr_;

	auto dom = GetDOM();
	if ( dom )
	{
		dom->SetName(imp_.Before_, false);
	}
}

