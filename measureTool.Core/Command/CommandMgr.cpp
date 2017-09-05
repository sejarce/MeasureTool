#include "CommandMgr.h"
#include "ICommand.h"

#include <list>
#include <algorithm>

class	CommandMgr::Imp
{
public:

	std::list<ICommandSPtr>	RedoList_;
	std::list<ICommandSPtr>	UndoList_;

	uint32_t				MaxSize_{ 10 };

	void	Resize()
	{
		while ( RedoList_.size() > MaxSize_ )
		{
			RedoList_.pop_front();
		}

		while ( UndoList_.size() > MaxSize_ )
		{
			UndoList_.pop_front();
		}
	}
};

CommandMgr& CommandMgr::GetInstance()
{
	static CommandMgr sIns;

	return sIns;
}

CommandMgr::CommandMgr() :ImpUPtr_(std::make_unique<Imp>())
{

}

CommandMgr::~CommandMgr()
{

}

void CommandMgr::PushCommand(const ICommandSPtr& cmd)
{
	auto& imp_ = *ImpUPtr_;

	imp_.UndoList_.push_back(cmd);

	imp_.Resize();
}

void CommandMgr::RemoveRelativeCommand(const IDOMSPtr& dom)
{
	auto& imp_ = *ImpUPtr_;

	imp_.UndoList_.remove_if([&](const ICommandSPtr& cmd)
	{
		return cmd->GetDOM() == dom;
	});

	imp_.RedoList_.remove_if([&](const ICommandSPtr& cmd)
	{
		return cmd->GetDOM() == dom;
	});
}

void CommandMgr::Redo()
{
	auto& imp_ = *ImpUPtr_;

	if ( imp_.RedoList_.empty() )
	{
		return;
	}

	auto curCmd = imp_.RedoList_.back();
	imp_.RedoList_.pop_back();

	curCmd->Execute();

	imp_.UndoList_.push_back(std::move(curCmd));

	imp_.Resize();
}

void CommandMgr::Undo()
{
	auto& imp_ = *ImpUPtr_;

	if ( imp_.UndoList_.empty() )
	{
		return;
	}

	auto curCmd = std::move(imp_.UndoList_.back());
	imp_.UndoList_.pop_back();

	curCmd->Undo();

	imp_.RedoList_.push_back(std::move(curCmd));

	imp_.Resize();
}

void CommandMgr::Clear()
{
	auto& imp_ = *ImpUPtr_;

	imp_.RedoList_.clear();
	imp_.UndoList_.clear();
}

void CommandMgr::SetMaxSize(uint32_t size)
{
	auto& imp_ = *ImpUPtr_;

	imp_.MaxSize_ = size;

	imp_.Resize();
}

uint32_t CommandMgr::GetMaxSize() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.MaxSize_;
}
