#include "EditContentCMD.h"

#include "DOM/IDOM.h"

class	EditContentCMD::Imp
{
public:

	std::string	Before_;
	std::string	Post_;
};

EditContentCMD::EditContentCMD(const IDOMSPtr& dom, const std::string& before, const std::string& post) :ICommand(dom)
, ImpUPtr_(std::make_unique<Imp>())
{
	auto& imp_ = *ImpUPtr_;

	imp_.Before_ = before;
	imp_.Post_ = post;
}

EditContentCMD::~EditContentCMD()
{

}

void EditContentCMD::Execute()
{
	auto& imp_ = *ImpUPtr_;

	auto dom = GetDOM();
	if ( dom )
	{
		
	}
}

void EditContentCMD::Undo()
{
	auto& imp_ = *ImpUPtr_;

	auto dom = GetDOM();
	if ( dom )
	{
		
	}
}

