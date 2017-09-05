#include "DeleteCMD.h"

#include "DOM/IDOM.h"

DeleteCMD::DeleteCMD(const IDOMSPtr& dom) :ICommand(dom)
{

}

void DeleteCMD::Execute()
{
	auto dom = GetDOM();
	if ( dom )
	{
		dom->SetState(IDOM::EDS_Delete, true);
	}
}

void DeleteCMD::Undo()
{
	auto dom = GetDOM();
	if ( dom )
	{
		dom->SetState(IDOM::EDS_Delete, false);
	}
}

