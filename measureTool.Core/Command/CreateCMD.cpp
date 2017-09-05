#include "CreateCMD.h"

#include "DOM/IDOM.h"

CreateCMD::CreateCMD(const IDOMSPtr& dom) :ICommand(dom)
{

}

void CreateCMD::Execute()
{
	auto dom = GetDOM();
	if ( dom )
	{
		dom->SetState(IDOM::EDS_Delete, false);
	}
}

void CreateCMD::Undo()
{
	auto dom = GetDOM();
	if ( dom )
	{
		dom->SetState(IDOM::EDS_Delete, true);
	}
}

